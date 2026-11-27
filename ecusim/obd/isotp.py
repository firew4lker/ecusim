# SPDX-License-Identifier: GPL-3.0-or-later
"""ISO 15765-2 (ISO-TP) transport, transport-agnostic (DESIGN §7.2).

``IsoTpLink`` is a pure state machine: feed it received CAN payloads with :meth:`on_frame`
and call :meth:`poll` regularly; it returns the CAN payloads to transmit. The same class
serves the ECU (tx 0x7E8 / rx 0x7E0) and the tester (tx 0x7E0 / rx 0x7E8).

Classic CAN only (8-byte frames, 12-bit length, max 4095 bytes).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

SF, FF, CF, FC = 0, 1, 2, 3
FS_CTS, FS_WAIT, FS_OVFLW = 0, 1, 2
MAX_LEN = 4095


def stmin_to_seconds(raw: int) -> float:
    if raw <= 0x7F:
        return raw / 1000.0
    if 0xF1 <= raw <= 0xF9:
        return (raw - 0xF0) / 10000.0
    return 0.127  # reserved values: use the longest defined separation


@dataclass
class IsoTpConfig:
    padding: int | None = 0xAA  # None = DLC-trimmed frames
    rx_block_size: int = 0  # BS we advertise when receiving
    rx_stmin: int = 0  # STmin we advertise when receiving
    n_bs: float = 1.0  # wait for flow control (s)
    n_cr: float = 1.0  # wait for consecutive frame (s)
    max_wait_frames: int = 10


class IsoTpError(Exception):
    pass


class IsoTpLink:
    def __init__(self, config: IsoTpConfig | None = None):
        self.cfg = config or IsoTpConfig()
        self.rx_queue: deque[bytes] = deque()  # completed incoming messages
        self.errors: list[str] = []
        self._out: list[bytes] = []
        # transmit state
        self._tx_data = b""
        self._tx_pos = 0
        self._tx_sn = 0
        self._tx_state = "idle"  # idle | wait_fc | sending
        self._tx_bs = 0
        self._tx_block_count = 0
        self._tx_stmin = 0.0
        self._tx_next = 0.0
        self._tx_deadline = 0.0
        self._tx_waits = 0
        self._tx_pending: deque[bytes] = deque()
        # receive state
        self._rx_buf = bytearray()
        self._rx_len = 0
        self._rx_sn = 0
        self._rx_active = False
        self._rx_deadline = 0.0
        self._rx_block_count = 0

    # ------------------------------------------------------------------ helpers
    def _frame(self, payload: bytes) -> bytes:
        if self.cfg.padding is None:
            return bytes(payload)
        return bytes(payload) + bytes([self.cfg.padding]) * (8 - len(payload))

    def _error(self, msg: str) -> None:
        self.errors.append(msg)

    @property
    def busy(self) -> bool:
        return self._tx_state != "idle"

    # ------------------------------------------------------------------ transmit
    def send(self, data: bytes, now: float) -> None:
        """Queue a message for transmission (queued behind any message in progress)."""
        if not data or len(data) > MAX_LEN:
            raise IsoTpError(f"invalid ISO-TP length {len(data)}")
        self._tx_pending.append(bytes(data))
        if self._tx_state == "idle":
            self._start_next(now)

    def _start_next(self, now: float) -> None:
        if not self._tx_pending:
            return
        data = self._tx_pending.popleft()
        if len(data) <= 7:
            self._out.append(self._frame(bytes([len(data)]) + data))
            self._start_next(now)
            return
        n = len(data)
        self._out.append(self._frame(bytes([0x10 | (n >> 8), n & 0xFF]) + data[:6]))
        self._tx_data = data
        self._tx_pos = 6
        self._tx_sn = 1
        self._tx_state = "wait_fc"
        self._tx_deadline = now + self.cfg.n_bs
        self._tx_waits = 0

    def _abort_tx(self, why: str, now: float) -> None:
        self._error(why)
        self._tx_state = "idle"
        self._tx_data = b""
        self._start_next(now)

    def _on_fc(self, p: bytes, now: float) -> None:
        if self._tx_state != "wait_fc":
            return  # unexpected FC: ignore
        fs = p[0] & 0x0F
        if fs == FS_CTS:
            self._tx_bs = p[1] if len(p) > 1 else 0
            self._tx_stmin = stmin_to_seconds(p[2] if len(p) > 2 else 0)
            self._tx_block_count = 0
            self._tx_state = "sending"
            self._tx_next = now
        elif fs == FS_WAIT:
            self._tx_waits += 1
            if self._tx_waits > self.cfg.max_wait_frames:
                self._abort_tx("too many FC.WAIT", now)
            else:
                self._tx_deadline = now + self.cfg.n_bs
        elif fs == FS_OVFLW:
            self._abort_tx("receiver overflow", now)
        else:
            self._abort_tx(f"invalid flow status {fs}", now)

    def _pump_tx(self, now: float) -> None:
        if self._tx_state == "wait_fc" and now > self._tx_deadline:
            self._abort_tx("N_Bs timeout waiting for flow control", now)
            return
        while self._tx_state == "sending" and now >= self._tx_next:
            chunk = self._tx_data[self._tx_pos : self._tx_pos + 7]
            self._out.append(self._frame(bytes([0x20 | self._tx_sn]) + chunk))
            self._tx_pos += len(chunk)
            self._tx_sn = (self._tx_sn + 1) & 0x0F
            if self._tx_pos >= len(self._tx_data):
                self._tx_state = "idle"
                self._tx_data = b""
                self._start_next(now)
                return
            self._tx_block_count += 1
            if self._tx_bs and self._tx_block_count >= self._tx_bs:
                self._tx_state = "wait_fc"
                self._tx_deadline = now + self.cfg.n_bs
                return
            self._tx_next = now + self._tx_stmin
            if self._tx_stmin > 0.0:
                return

    # ------------------------------------------------------------------ receive
    def _send_fc(self, fs: int) -> None:
        self._out.append(self._frame(bytes([0x30 | fs, self.cfg.rx_block_size, self.cfg.rx_stmin])))

    def on_frame(self, payload: bytes, now: float) -> None:
        if not payload:
            return
        p = bytes(payload)
        kind = p[0] >> 4
        if kind == SF:
            n = p[0] & 0x0F
            if n == 0 or n > 7 or n > len(p) - 1:
                return  # invalid SF (CAN-FD escape not supported)
            if self._rx_active:
                self._error("SF interrupted multi-frame reception")
                self._rx_active = False
            self.rx_queue.append(p[1 : 1 + n])
        elif kind == FF:
            if len(p) < 8:
                return
            n = ((p[0] & 0x0F) << 8) | p[1]
            if n < 8:
                return  # must use SF
            if self._rx_active:
                self._error("FF interrupted multi-frame reception")
            self._rx_buf = bytearray(p[2:8])
            self._rx_len = n
            self._rx_sn = 1
            self._rx_active = True
            self._rx_block_count = 0
            self._rx_deadline = now + self.cfg.n_cr
            self._send_fc(FS_CTS)
        elif kind == CF:
            if not self._rx_active:
                return
            sn = p[0] & 0x0F
            if sn != self._rx_sn:
                self._error(f"wrong sequence number {sn}, expected {self._rx_sn}")
                self._rx_active = False
                return
            self._rx_sn = (self._rx_sn + 1) & 0x0F
            self._rx_buf += p[1:8]
            self._rx_deadline = now + self.cfg.n_cr
            if len(self._rx_buf) >= self._rx_len:
                self.rx_queue.append(bytes(self._rx_buf[: self._rx_len]))
                self._rx_active = False
                return
            self._rx_block_count += 1
            if self.cfg.rx_block_size and self._rx_block_count >= self.cfg.rx_block_size:
                self._rx_block_count = 0
                self._send_fc(FS_CTS)
        elif kind == FC:
            self._on_fc(p, now)

    # ------------------------------------------------------------------ driver
    def poll(self, now: float) -> list[bytes]:
        """Advance timers; return CAN payloads to transmit now."""
        if self._rx_active and now > self._rx_deadline:
            self._error("N_Cr timeout waiting for consecutive frame")
            self._rx_active = False
        self._pump_tx(now)
        out, self._out = self._out, []
        return out

    def next_deadline(self) -> float | None:
        """Earliest time ``poll`` has something to do (for sleeping transports)."""
        times = []
        if self._tx_state == "sending":
            times.append(self._tx_next)
        if self._tx_state == "wait_fc":
            times.append(self._tx_deadline)
        if self._rx_active:
            times.append(self._rx_deadline)
        return min(times) if times else None
