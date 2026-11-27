# SPDX-License-Identifier: GPL-3.0-or-later
"""ELM327 v1.5 command interpreter (DESIGN §7.7.3).

Pure: bytes in, text out. No I/O, no threads, no clock. A transport feeds it what the diagnostic app
sends and writes back the replies; ``Reply.delay`` tells the transport how long the adapter "waits for
the car" so that it can emulate latency and the ``STOPPED`` interruption.

The car side is ``ObdServer.handle(request, functional)``, the same handler the CAN server uses. The
CAN frames that a real adapter would exchange are reported through ``on_frame`` for the bus trace.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Callable

from ..obd.can_transport import ADDR_11BIT, ADDR_29BIT

DEFAULT_IDENTITY = "ELM327 v1.5"
IDENTITIES = ("ELM327 v1.5", "ELM327 v2.1", "ELM327 v1.4b", "ELM327 v1.3a")

PROTOCOLS = {
    "1": "SAE J1850 PWM",
    "2": "SAE J1850 VPW",
    "3": "ISO 9141-2",
    "4": "ISO 14230-4 (KWP 5BAUD)",
    "5": "ISO 14230-4 (KWP FAST)",
    "6": "ISO 15765-4 (CAN 11/500)",
    "7": "ISO 15765-4 (CAN 29/500)",
    "8": "ISO 15765-4 (CAN 11/250)",
    "9": "ISO 15765-4 (CAN 29/250)",
    "A": "SAE J1939 (CAN 29/250)",
    "B": "USER1 CAN (11* /125)",
    "C": "USER2 CAN (11* /50)",
}
CAN_PROTOCOLS = {"6": (False, 500_000), "7": (True, 500_000), "8": (False, 250_000), "9": (True, 250_000)}
SEARCH_S = 1.5  # an unsuccessful protocol search takes a while on a real adapter
ELM_PADDING = 0x00  # padding the ELM327 puts on its own request frames
HEX = set("0123456789ABCDEF")

# Commands accepted and acknowledged with OK although they change nothing here (KWP timing, wake-up
# messages, CAN flow-control tuning, memory, ...): apps send them during initialisation.
_OK_PATTERNS = re.compile(
    r"^(M[01]|V[01]|AR|AL|NL|CFC[01]|CSM[01]|KW[01]|AT[012]|BI|FE|JE|JS|JHF[01]|JTM[15]|"
    r"ST[0-9A-F]{1,2}|IB(10|48|96)|SW[0-9A-F]{2}|WM[0-9A-F]+|IIA[0-9A-F]{2}|SR[0-9A-F]{2}|RA[0-9A-F]{2}|"
    r"TA[0-9A-F]{2}|SI|FI|CF[0-9A-F]{3,8}|CM[0-9A-F]{3,8}|FC.*|CEA[0-9A-F]{0,2}|CV[0-9]{4}|SS|DM1|"
    r"MP[0-9A-F]+|PB[0-9A-F]{4}|SD[0-9A-F]{2}|AMC|AMT[0-9A-F]{2}|LP|IFR[0-2H]|BRD[0-9A-F]{2}|BRT[0-9A-F]{2})$"
)


@dataclass
class CarLink:
    """What the adapter needs from the simulated car."""

    handler: Callable[[bytes, bool], bytes | None]  # ObdServer.handle
    powered: Callable[[], bool]  # ECU on the bus (key on)
    vbatt: Callable[[], float]  # voltage on OBD pin 16
    bus: Callable[[], tuple[bool, int, int | None]]  # (29-bit, bitrate, padding) of the simulated car


@dataclass
class Reply:
    """One command's answer. ``echo`` goes out at once, ``body`` after ``delay`` seconds unless the app
    sends something meanwhile, in which case ``stopped`` is sent instead."""

    echo: str
    body: str
    delay: float = 0.0
    stopped: str = ""
    command: str = ""


@dataclass
class Settings:
    echo: bool = True
    linefeed: bool = False
    spaces: bool = True
    headers: bool = False
    responses: bool = True
    caf: bool = True  # CAN automatic formatting
    dlc: bool = False
    auto: bool = True  # protocol 0 / Ax: search
    protocol: str = "6"  # fixed protocol when not auto, preferred one when auto
    found: str | None = None  # protocol in use after a successful search
    header: int | None = None  # ATSH, None = default functional address
    header_29: bool = False  # header was given in 29-bit form
    priority: int = 0x18  # ATCP (29-bit)
    cra: int | None = None  # ATCRA receive filter
    extra: dict = field(default_factory=dict)


def segment(payload: bytes, padding: int | None) -> list[bytes]:
    """ISO 15765-2 frames for ``payload`` (as an ECU sends them)."""

    def pad(f: bytes) -> bytes:
        return f if padding is None else f + bytes([padding]) * (8 - len(f))

    if len(payload) <= 7:
        return [pad(bytes([len(payload)]) + payload)]
    frames = [bytes([0x10 | (len(payload) >> 8) & 0x0F, len(payload) & 0xFF]) + payload[:6]]
    rest, sn = payload[6:], 1
    while rest:
        frames.append(pad(bytes([0x20 | (sn & 0x0F)]) + rest[:7]))
        rest, sn = rest[7:], sn + 1
    return frames


class Elm327:
    def __init__(self, car: CarLink, identity: str = DEFAULT_IDENTITY, latency_s: float = 0.04,
                 on_frame: Callable[[str, int, bool, bytes], None] | None = None):
        self.car = car
        self.identity = identity
        self.latency_s = latency_s
        self.on_frame = on_frame or (lambda *_: None)
        self.s = Settings()
        self._buf = bytearray()
        self._last = ""
        self.stats = {"commands": 0, "requests": 0, "responses": 0}

    # ------------------------------------------------------------------ input
    def feed(self, data: bytes) -> list[Reply]:
        """Bytes from the app; returns a reply for every complete command (ended by CR)."""
        out = []
        for b in data:
            if b in (0x00, 0x0A):
                continue
            if b == 0x0D:
                line = self._buf.decode("ascii", "replace")
                self._buf.clear()
                out.append(self.command(line))
            elif len(self._buf) < 128:
                self._buf.append(b)
        return out

    def command(self, raw: str) -> Reply:
        s = self.s
        echo = raw + "\r" if s.echo else ""
        cmd = "".join(raw.split()).upper()
        if not cmd:
            cmd = self._last  # a bare CR repeats the last command
            if not cmd:
                return Reply(echo, ">")
        self._last = cmd
        self.stats["commands"] += 1
        delay = 0.0
        if cmd.startswith("AT"):
            lines = self._at(cmd[2:])
        elif cmd.startswith("ST"):
            lines = ["?"]  # STN/OBDLink extensions are not implemented
        else:
            lines, delay = self._obd(cmd)
        eol = "\r\n" if self.s.linefeed else "\r"
        body = "".join(line + eol for line in lines) + eol + ">"
        return Reply(echo, body, delay, "STOPPED" + eol + eol + ">", cmd)

    # ------------------------------------------------------------------ AT commands
    def _at(self, a: str) -> list[str]:
        s = self.s
        if a in ("Z", "WS"):
            self.s = Settings()
            return ["", "", self.identity]
        if a == "D":
            self.s = Settings()
            return ["OK"]
        if a == "I":
            return [self.identity]
        if a == "@1":
            return ["OBDII to RS232 Interpreter"]
        if a == "RV":
            return [f"{self.car.vbatt():.1f}V"]
        if a == "IGN":
            return ["ON" if self.car.powered() else "OFF"]
        if a == "CS":
            return ["T:00 R:00"]
        flags = {"E": "echo", "L": "linefeed", "S": "spaces", "H": "headers", "R": "responses", "CAF": "caf",
                 "D": "dlc"}
        m = re.fullmatch(r"(E|L|S|H|R|CAF|D)([01])", a)
        if m:
            setattr(s, flags[m.group(1)], m.group(2) == "1")
            return ["OK"]
        m = re.fullmatch(r"(SP|TP)(A?)([0-9A-C])", a)
        if m:
            auto, p = bool(m.group(2)) or m.group(3) == "0", m.group(3)
            s.auto, s.found = auto, None
            s.protocol = p if p != "0" else s.protocol
            return ["OK"]
        if a == "DP":
            if s.auto:
                return ["AUTO, " + PROTOCOLS[s.found] if s.found else "AUTO"]
            return [PROTOCOLS[s.protocol]]
        if a == "DPN":
            if s.auto:
                return ["A" + (s.found or "0")]
            return [s.protocol]
        if a == "PC":
            if s.auto:
                s.found = None
            return ["OK"]
        m = re.fullmatch(r"SH([0-9A-F]{3}|[0-9A-F]{6}|[0-9A-F]{8})", a)
        if m:
            h = m.group(1)
            if len(h) == 8:
                s.priority, s.header, s.header_29 = int(h[:2], 16), int(h, 16), True
            elif len(h) == 6:
                s.header, s.header_29 = (s.priority << 24) | int(h, 16), True
            else:
                s.header, s.header_29 = int(h, 16), False
            return ["OK"]
        m = re.fullmatch(r"CP([0-9A-F]{2})", a)
        if m:
            s.priority = int(m.group(1), 16)
            if s.header is not None and s.header_29:
                s.header = (s.priority << 24) | (s.header & 0xFFFFFF)
            return ["OK"]
        m = re.fullmatch(r"CRA([0-9A-F]{3}|[0-9A-F]{8})?", a)
        if m:
            s.cra = int(m.group(1), 16) if m.group(1) else None
            return ["OK"]
        if _OK_PATTERNS.match(a):
            return ["OK"]
        return ["?"]  # incl. monitoring (MA/MR/MT), programmable parameters (PP), buffer dump (BD)

    # ------------------------------------------------------------------ OBD requests
    def _native(self) -> str | None:
        """The protocol the simulated car actually uses."""
        ext, bitrate, _ = self.car.bus()
        for p, (e, br) in CAN_PROTOCOLS.items():
            if e == ext and br == bitrate:
                return p
        return None

    def _obd(self, cmd: str) -> tuple[list[str], float]:
        s = self.s
        if not set(cmd) <= HEX:
            return ["?"], 0.0
        if len(cmd) % 2:  # trailing digit = number of responses to wait for
            cmd = cmd[:-1]
        if not cmd or len(cmd) > 14:
            return ["?"], 0.0
        req = bytes.fromhex(cmd)
        self.stats["requests"] += 1
        lines: list[str] = []
        if s.auto and s.found is None:
            lines.append("SEARCHING...")
            native = self._native()
            if native is None or not self.car.powered():
                return lines + ["UNABLE TO CONNECT"], SEARCH_S
            s.found = native
        proto = s.found if s.auto else s.protocol
        if proto in ("3", "4", "5"):
            return ["BUS INIT: ...ERROR"], SEARCH_S
        if proto not in CAN_PROTOCOLS:
            return ["NO DATA"], self.latency_s
        native = self._native()
        if proto != native:
            same_rate = native is not None and CAN_PROTOCOLS[native][1] == CAN_PROTOCOLS[proto][1]
            return lines + ["NO DATA" if same_rate else "CAN ERROR"], self.latency_s
        if not self.car.powered():
            return lines + ["NO DATA"], self.latency_s

        ext = CAN_PROTOCOLS[proto][0]
        addr = ADDR_29BIT if ext else ADDR_11BIT
        req_id = s.header if s.header is not None else addr["functional"]
        if ext and s.header is not None and not s.header_29:
            req_id = (s.priority << 24) | s.header  # an 11-bit style header on a 29-bit bus
        self.on_frame("rx", req_id, ext, bytes([len(req)]) + req + bytes([ELM_PADDING]) * (7 - len(req)))
        if req_id == addr["functional"]:
            resp = self.car.handler(req, True)
        elif req_id == addr["request"]:
            resp = self.car.handler(req, False)
        else:
            resp = None  # no module at that address
        if resp is None:
            return lines + ["NO DATA"], self.latency_s
        _, _, padding = self.car.bus()
        frames = segment(resp, padding)
        resp_id = addr["response"]
        for i, f in enumerate(frames):
            self.on_frame("tx", resp_id, ext, f)
            if i == 0 and len(frames) > 1:  # the adapter's flow control after the first frame
                fc_id = addr["request"]
                self.on_frame("rx", fc_id, ext, bytes([0x30, 0x00, 0x00]) + bytes([ELM_PADDING]) * 5)
        if s.cra is not None and s.cra != resp_id:
            return lines + ["NO DATA"], self.latency_s
        self.stats["responses"] += 1
        if s.responses:
            lines += self._format(resp, frames, resp_id, ext)
        return lines, self.latency_s

    # ------------------------------------------------------------------ formatting
    def _hex(self, data: bytes) -> str:
        return (" " if self.s.spaces else "").join(f"{b:02X}" for b in data)

    def _format(self, resp: bytes, frames: list[bytes], resp_id: int, ext: bool) -> list[str]:
        s = self.s
        sep = " " if s.spaces else ""
        if s.headers or not s.caf:
            out = []
            for i, f in enumerate(frames):
                shown = f
                if s.caf and len(frames) == 1:
                    shown = f[: 1 + f[0]]  # single frame: PCI and data, no padding
                parts = []
                if s.headers:
                    parts.append(self._hex(resp_id.to_bytes(4, "big")) if ext else f"{resp_id:03X}")
                    if s.dlc:
                        parts.append(str(len(f)))
                parts.append(self._hex(shown))
                out.append(sep.join(parts))
            return out
        if len(frames) == 1:
            return [self._hex(resp)]
        out = [f"{len(resp):03X}"]
        chunks = [resp[:6]] + [resp[i : i + 7] for i in range(6, len(resp), 7)]
        for i, c in enumerate(chunks):
            out.append(f"{i & 0x0F:X}:{sep}{self._hex(c)}")
        return out
