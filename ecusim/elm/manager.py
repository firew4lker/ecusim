# SPDX-License-Identifier: GPL-3.0-or-later
"""The emulated ELM327 adapters: settings, switching each transport on and off, persistence, traces
(DESIGN §7.7.2). Qt-free."""

from __future__ import annotations

import copy
import json
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

from ..obd.can_transport import TraceEntry
from .emulator import DEFAULT_IDENTITY, CarLink, Elm327
from .transports import (BluetoothTransport, PtyTransport, SerialTransport, TcpTransport, Transport,
                         bluetooth_unavailable)

KINDS = ("tcp", "pty", "serial", "bt")

DEFAULTS: dict = {
    "identity": DEFAULT_IDENTITY,
    "latency_ms": 40.0,
    "tcp": {"enabled": False, "port": 35000, "network": False},
    "pty": {"enabled": False, "link": ""},
    "serial": {"enabled": False, "port": "", "baud": 38400},
    "bt": {"enabled": False, "name": "OBDII", "rename": False},
}


def unavailable(kind: str) -> str:
    """Why a transport can't be used on this computer ("" if it can)."""
    if kind == "pty" and sys.platform == "win32":
        return "Windows has no virtual serial ports: install com0com and use an existing serial port"
    if kind == "bt":
        return bluetooth_unavailable()
    return ""


@dataclass
class LogLine:
    t: float
    kind: str  # transport
    who: str  # "app", "elm", "info"
    text: str


class ElmManager:
    def __init__(self, car: CarLink, config_path: Path | str | None = None, trace_len: int = 2000):
        self.car = car
        self.config_path = Path(config_path) if config_path else None
        self.settings = copy.deepcopy(DEFAULTS)
        self._saved = self._load()
        self._merge(self.settings, self._saved)
        self.session_only: set[str] = set()  # enabled/disabled from the command line, not saved
        self.transports: dict[str, Transport] = {}
        self.trace: deque[TraceEntry] = deque(maxlen=trace_len)
        self.log: deque[LogLine] = deque(maxlen=500)
        self.stats = {"requests": 0, "responses": 0}

    # ------------------------------------------------------------------ settings
    @staticmethod
    def _merge(dst: dict, src: dict) -> None:
        for k, v in src.items():
            if k in dst and isinstance(dst[k], dict) and isinstance(v, dict):
                dst[k].update({kk: vv for kk, vv in v.items() if kk in dst[k]})
            elif k in dst and not isinstance(dst[k], dict):
                dst[k] = v

    def _load(self) -> dict:
        if self.config_path is None:
            return {}
        try:
            data = json.loads(self.config_path.read_text())
            return data.get("elm", {}) if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def save(self) -> None:
        if self.config_path is None:
            return
        out = copy.deepcopy(self.settings)
        for kind in self.session_only:  # keep what was saved before the command-line override
            out[kind] = copy.deepcopy(DEFAULTS[kind])
            self._merge(out, {kind: self._saved.get(kind, {})})
        try:
            data = json.loads(self.config_path.read_text())
            if not isinstance(data, dict):
                data = {}
        except (OSError, ValueError):
            data = {}
        data["elm"] = out
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.config_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2))
        tmp.replace(self.config_path)
        self._saved = copy.deepcopy(out)

    def update(self, kind: str | None = None, **values) -> None:
        """Change settings (``kind`` None for the shared options). Takes effect on the next start."""
        target = self.settings if kind is None else self.settings[kind]
        target.update(values)
        if kind is None:
            for t in self.transports.values():
                t.emu.identity = self.settings["identity"]
                t.emu.latency_s = self.settings["latency_ms"] / 1000.0

    # ------------------------------------------------------------------ switching
    def _make(self, kind: str) -> Transport:
        s = self.settings
        emu = Elm327(self.car, s["identity"], s["latency_ms"] / 1000.0, on_frame=self._frame)
        c = s[kind]
        if kind == "tcp":
            return TcpTransport(emu, "0.0.0.0" if c["network"] else "127.0.0.1", int(c["port"]), log=self._log)
        if kind == "pty":
            return PtyTransport(emu, c["link"], log=self._log)
        if kind == "serial":
            return SerialTransport(emu, c["port"], int(c["baud"]), log=self._log)
        return BluetoothTransport(emu, c["name"], bool(c["rename"]), log=self._log)

    def set_enabled(self, kind: str, on: bool, session_only: bool = False) -> None:
        """Switch a transport on or off. Raises (and keeps the error for ``status``) if it can't start."""
        if session_only:
            self.session_only.add(kind)
        else:
            self.session_only.discard(kind)
        self.settings[kind]["enabled"] = on
        old = self.transports.pop(kind, None)
        if old is not None:
            old.stop()
        if not on:
            return
        reason = unavailable(kind)
        t = self._make(kind)
        self.transports[kind] = t
        if reason:
            t.error = reason
            raise RuntimeError(reason)
        t.start()

    def start_enabled(self) -> list[str]:
        """Start every transport the settings switch on; returns error messages."""
        errors = []
        for kind in KINDS:
            if self.settings[kind]["enabled"] and kind not in self.transports:
                try:
                    self.set_enabled(kind, True, session_only=kind in self.session_only)
                except Exception as e:
                    errors.append(f"ELM327 {kind}: {e}")
        return errors

    def stop_all(self) -> None:
        for t in self.transports.values():
            t.stop()
        self.transports.clear()

    def is_running(self, kind: str) -> bool:
        t = self.transports.get(kind)
        return t is not None and t.running

    def status(self, kind: str) -> str:
        t = self.transports.get(kind)
        if t is None:
            return unavailable(kind) or "off"
        return t.status()

    def note(self, kind: str) -> str:
        t = self.transports.get(kind)
        return t.note if t is not None else ""

    def counters(self, kind: str) -> dict:
        t = self.transports.get(kind)
        return dict(t.emu.stats) if t is not None else {"commands": 0, "requests": 0, "responses": 0}

    # ------------------------------------------------------------------ traces
    def _frame(self, direction: str, arb_id: int, extended: bool, data: bytes) -> None:
        self.trace.append(TraceEntry(time.monotonic(), direction, arb_id, extended, data))

    def _log(self, kind: str, who: str, text: str) -> None:
        self.log.append(LogLine(time.monotonic(), kind, who, text))
