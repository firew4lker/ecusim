"""Minimal OBD-II tester (scan-tool side) over python-can, for testing and the bench tester tool."""

from __future__ import annotations

import time

import can

from .can_transport import CanConfig, open_bus
from .isotp import IsoTpConfig, IsoTpLink
from .pids import MODE01, SUPPORT_PIDS, decode_bitmap


class ObdTimeout(Exception):
    pass


class NegativeResponse(Exception):
    def __init__(self, sid: int, code: int):
        super().__init__(f"negative response to service 0x{sid:02X}: NRC 0x{code:02X}")
        self.sid = sid
        self.code = code


class ObdClient:
    def __init__(self, cfg: CanConfig, bus: can.BusABC | None = None, p2_timeout: float = 1.0):
        self.cfg = cfg
        self._own = bus is None
        self.bus = bus or open_bus(cfg)
        self.p2 = p2_timeout
        self.link = IsoTpLink(IsoTpConfig(padding=cfg.padding))

    def close(self) -> None:
        if self._own:
            self.bus.shutdown()

    def __enter__(self) -> "ObdClient":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------ transport
    def _tx(self, arb_id: int, payload: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=arb_id, is_extended_id=self.cfg.extended, data=payload))

    def request(self, data: bytes, functional: bool = True, timeout: float | None = None) -> bytes:
        """Send a request and return the first complete positive or negative response."""
        a = self.cfg.addr
        timeout = self.p2 if timeout is None else timeout
        self.link = IsoTpLink(IsoTpConfig(padding=self.cfg.padding))
        if functional:
            if len(data) > 7:
                raise ValueError("functional requests must fit in a single frame")
            pad = b"" if self.cfg.padding is None else bytes([self.cfg.padding]) * (7 - len(data))
            self._tx(a["functional"], bytes([len(data)]) + data + pad)
        else:
            self.link.send(data, time.monotonic())
        deadline = time.monotonic() + timeout
        while True:
            now = time.monotonic()
            for payload in self.link.poll(now):
                self._tx(a["request"], payload)
            if self.link.rx_queue:
                resp = self.link.rx_queue.popleft()
                sid = data[0]
                if not resp or not (resp[0] == sid + 0x40 or (resp[0] == 0x7F and len(resp) > 1 and resp[1] == sid)):
                    continue  # late answer to an earlier request: discard, keep waiting
                if (len(resp) > 1 and resp[0] == sid + 0x40 and sid in (0x01, 0x02, 0x06, 0x09)
                        and len(data) > 1 and resp[1] != data[1]):
                    continue  # same service, different PID/info type: stale
                if len(resp) >= 3 and resp[0] == 0x7F and resp[2] == 0x78:
                    deadline = time.monotonic() + 5.0  # response pending
                    continue
                if resp and resp[0] == 0x7F:
                    raise NegativeResponse(resp[1], resp[2] if len(resp) > 2 else 0)
                return resp
            if now > deadline:
                raise ObdTimeout(f"no response to {data.hex(' ')}")
            msg = self.bus.recv(timeout=0.005)
            if msg is not None and msg.arbitration_id == a["response"] and msg.is_extended_id == self.cfg.extended:
                self.link.on_frame(bytes(msg.data), time.monotonic())

    # ------------------------------------------------------------------ OBD services
    def supported_pids(self) -> set[int]:
        found: set[int] = set()
        for base in SUPPORT_PIDS:
            if base and base not in found:
                break
            resp = self.request(bytes([0x01, base]))
            found |= decode_bitmap(resp[2:6], base)
        return found

    def read_pids(self, pids: list[int]) -> dict[int, object]:
        """Read up to six PIDs in one request and decode them."""
        resp = self.request(bytes([0x01, *pids]))
        if resp[0] != 0x41:
            raise ValueError(f"unexpected response {resp.hex(' ')}")
        out: dict[int, object] = {}
        i = 1
        while i < len(resp):
            pid = resp[i]
            if pid in SUPPORT_PIDS:
                n = 4
                out[pid] = resp[i + 1 : i + 1 + n]
            else:
                n = MODE01[pid].length
                out[pid] = MODE01[pid].decode(resp[i + 1 : i + 1 + n])
            i += 1 + n
        return out

    def read_pid(self, pid: int) -> object:
        return self.read_pids([pid])[pid]

    def vehicle_info(self, infotype: int) -> bytes:
        resp = self.request(bytes([0x09, infotype]))
        if resp[:2] != bytes([0x49, infotype]):
            raise ValueError(f"unexpected response {resp.hex(' ')}")
        return resp[3:]  # skip number-of-data-items byte

    def vin(self) -> str:
        return self.vehicle_info(0x02).decode("ascii", "replace")

    def dtcs(self, mode: int = 0x03) -> list[str]:
        """Mode 03 (stored), 07 (pending) or 0A (permanent)."""
        from ..ecu.diag.codes import decode_dtc

        resp = self.request(bytes([mode]))
        n = resp[1]
        return [decode_dtc(resp[2 + 2 * i: 4 + 2 * i]) for i in range(n)]

    def clear_dtcs(self) -> None:
        self.request(b"\x04")

    def readiness(self) -> dict:
        a, b, c, d = self.read_pid(0x01)
        names = {0: "catalyst", 1: "heated catalyst", 2: "EVAP", 3: "secondary air", 4: "A/C", 5: "O2 sensor",
                 6: "O2 heater", 7: "EGR/VVT"}
        out = {"mil": bool(a & 0x80), "dtc_count": a & 0x7F}
        for bit, name in enumerate(("misfire", "fuel system", "components")):
            if b & (1 << bit):
                out[name] = "incomplete" if b & (1 << (bit + 4)) else "complete"
        for bit, name in names.items():
            if c & (1 << bit):
                out[name] = "incomplete" if d & (1 << bit) else "complete"
        return out

    def ecu_name(self) -> str:
        return self.vehicle_info(0x0A).replace(b"\x00", b"").decode("ascii", "replace")
