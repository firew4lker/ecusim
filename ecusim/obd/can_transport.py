# SPDX-License-Identifier: GPL-3.0-or-later
"""CAN side of the ECU: python-can bus, ISO 15765-4 addressing, ISO-TP, response scheduling (DESIGN §7.1).

Supported python-can interfaces include socketcan (Linux, incl. vcan), slcan, pcan, kvaser,
gs_usb, virtual (in-process) and udp_multicast (cross-process testing without hardware).
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

import can

from .isotp import IsoTpConfig, IsoTpLink

ADDR_11BIT = {"functional": 0x7DF, "request": 0x7E0, "response": 0x7E8}
ADDR_29BIT = {"functional": 0x18DB33F1, "request": 0x18DA10F1, "response": 0x18DAF110}

# Interfaces where python-can sets the bitrate itself. For socketcan the bitrate is set with `ip link`.
_BITRATE_INTERFACES = {"pcan", "kvaser", "slcan", "gs_usb", "vector", "ixxat", "cantact", "usb2can", "serial",
                       "robotell", "seeedstudio", "canalystii", "systec", "neovi", "nican", "nixnet", "etas", "iscan"}


@dataclass
class CanConfig:
    interface: str = "virtual"
    channel: str = "ecusim"
    bitrate: int = 500_000
    extended: bool = False  # 29-bit identifiers
    padding: int | None = 0xAA
    response_delay_ms: float = 0.0
    tty_baudrate: int | None = None  # serial-line adapters (slcan on ttyUSB/COM); ignored by USB-CDC ttyACM

    @property
    def addr(self) -> dict[str, int]:
        return ADDR_29BIT if self.extended else ADDR_11BIT


def open_bus(cfg: CanConfig, **extra) -> can.BusABC:
    kwargs: dict = {"interface": cfg.interface, "channel": cfg.channel}
    if cfg.interface in _BITRATE_INTERFACES:
        kwargs["bitrate"] = cfg.bitrate
    if cfg.tty_baudrate:
        if cfg.interface == "slcan":
            kwargs["tty_baudrate"] = cfg.tty_baudrate
        elif cfg.interface in ("serial", "robotell", "seeedstudio"):
            kwargs["baudrate"] = cfg.tty_baudrate
    kwargs.update(extra)
    return can.Bus(**kwargs)


@dataclass
class TraceEntry:
    t: float
    direction: str  # "rx" | "tx"
    arb_id: int
    extended: bool
    data: bytes


class EcuCanServer:
    """Runs the ECU's CAN endpoint on its own thread."""

    def __init__(
        self,
        cfg: CanConfig,
        handler: Callable[[bytes, bool], bytes | None],
        powered: Callable[[], bool],
        bus: can.BusABC | None = None,
        trace_len: int = 2000,
    ):
        self.cfg = cfg
        self.handler = handler
        self.powered = powered
        self._bus = bus
        self._own_bus = bus is None
        self.link = IsoTpLink(IsoTpConfig(padding=cfg.padding))
        self.trace: deque[TraceEntry] = deque(maxlen=trace_len)
        self.stats = {"rx": 0, "tx": 0, "requests": 0, "responses": 0, "errors": 0}
        self._pending: list[tuple[float, bytes]] = []
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        if self._bus is None:
            self._bus = open_bus(self.cfg)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ecu-can", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._own_bus and self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    # ------------------------------------------------------------------ internals
    def _send(self, payload: bytes, now: float) -> None:
        a = self.cfg.addr
        msg = can.Message(arbitration_id=a["response"], is_extended_id=self.cfg.extended, data=payload)
        try:
            self._bus.send(msg)
        except can.CanError:
            self.stats["errors"] += 1
            return
        self.stats["tx"] += 1
        self.trace.append(TraceEntry(now, "tx", a["response"], self.cfg.extended, bytes(payload)))

    def _dispatch(self, request: bytes, functional: bool, now: float) -> None:
        self.stats["requests"] += 1
        resp = self.handler(request, functional)
        if resp is None:
            return
        self.stats["responses"] += 1
        due = now + self.cfg.response_delay_ms / 1000.0
        self._pending.append((due, resp))

    def _on_message(self, msg: can.Message, now: float) -> None:
        a = self.cfg.addr
        if msg.is_extended_id != self.cfg.extended or msg.is_error_frame or msg.is_remote_frame:
            return
        if msg.arbitration_id not in (a["functional"], a["request"]):
            return
        self.stats["rx"] += 1
        self.trace.append(TraceEntry(now, "rx", msg.arbitration_id, msg.is_extended_id, bytes(msg.data)))
        if not self.powered():
            return  # key off: the ECU is not on the bus
        data = bytes(msg.data)
        if msg.arbitration_id == a["functional"]:
            # Functional requests must be single frames
            if data and (data[0] >> 4) == 0 and 1 <= (data[0] & 0x0F) <= min(7, len(data) - 1):
                self._dispatch(data[1 : 1 + (data[0] & 0x0F)], True, now)
            return
        self.link.on_frame(data, now)
        while self.link.rx_queue:
            self._dispatch(self.link.rx_queue.popleft(), False, now)

    def _run(self) -> None:
        bus = self._bus
        while not self._stop.is_set():
            now = time.monotonic()
            wait = 0.005
            nd = self.link.next_deadline()
            if nd is not None:
                wait = max(0.0, min(wait, nd - now))
            if self._pending:
                wait = max(0.0, min(wait, min(d for d, _ in self._pending) - now))
            try:
                msg = bus.recv(timeout=wait)
            except can.CanError:
                self.stats["errors"] += 1
                msg = None
            now = time.monotonic()
            if msg is not None:
                self._on_message(msg, now)
            if self._pending:
                ready = [p for p in self._pending if p[0] <= now]
                self._pending = [p for p in self._pending if p[0] > now]
                for _, resp in ready:
                    self.link.send(resp, now)
            for payload in self.link.poll(now):
                self._send(payload, now)
            if self.link.errors:
                self.stats["errors"] += len(self.link.errors)
                self.link.errors.clear()
