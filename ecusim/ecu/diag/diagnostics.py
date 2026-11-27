# SPDX-License-Identifier: GPL-3.0-or-later
"""On-board diagnostics executive: drive/warm-up cycles, monitors, DTCs, counters, readiness (DESIGN §6)."""

from __future__ import annotations

from typing import TYPE_CHECKING

from ...core.signals import Readings
from .dtc import DtcManager
from .iumpr import Iumpr
from .mode06 import Mode06Store
from .monitors import Monitor, build_monitors

if TYPE_CHECKING:
    from ..ecm import ECM

MIL_OFF, MIL_ON, MIL_FLASH = 0, 1, 2


class Diagnostics:
    def __init__(self, ecm: "ECM"):
        self.ecm = ecm
        self.readings = Readings()
        self.dtc = DtcManager(capture=ecm.obd_values)
        self.mode06 = Mode06Store()
        self.iumpr = Iumpr()
        self.monitors: list[Monitor] = build_monitors()
        self._by_key = {m.key: m for m in self.monitors}
        for m in self.monitors:
            if hasattr(m, "declare"):
                m.declare(self)
        # counters since DTCs cleared (J1979 PIDs 21, 30, 31, 4D, 4E)
        self.warmups_since_clear = 0
        self.dist_since_clear_km = 0.0
        self.time_since_clear_s = 0.0
        self.dist_mil_km = 0.0
        self.time_mil_s = 0.0
        # cycle state
        self.cycle_active = False
        self.engine_started_this_cycle = False
        self._ect_start: float | None = None
        self._ect_max = -50.0
        self._warmup_done = False
        self.trips = 0

    def monitor(self, key: str) -> Monitor:
        return self._by_key[key]

    # ------------------------------------------------------------------ cycle events
    def power_up(self) -> None:
        self.cycle_active = True
        self.engine_started_this_cycle = False
        self._ect_start = None
        self._ect_max = -50.0
        self._warmup_done = False
        for m in self.monitors:
            m.new_cycle()
        self.iumpr.new_cycle()

    def engine_started(self, ect_c: float) -> None:
        if not self.engine_started_this_cycle:
            self.engine_started_this_cycle = True
            self._ect_start = ect_c
            self.iumpr.engine_started()

    def power_down(self) -> None:
        if not self.cycle_active:
            return
        self.cycle_active = False
        if self.engine_started_this_cycle:
            for m in self.monitors:
                m.end_cycle(self)
            self.dtc.end_trip()
            self.trips += 1

    # ------------------------------------------------------------------ step
    def step(self, dt: float, r: Readings) -> None:
        self.readings = r
        e = self.ecm
        running = e.mode == "run"
        if running:
            self.time_since_clear_s += dt
            km = r.vss_kmh * dt / 3600.0
            self.dist_since_clear_km += km
            if self.dtc.mil_on:
                self.dist_mil_km += km
                self.time_mil_s += dt
            if self._ect_start is not None and not self._warmup_done:
                self._ect_max = max(self._ect_max, r.ect_c)
                if self._ect_max >= 70.0 and self._ect_max - self._ect_start >= 22.0:
                    self._warmup_done = True
                    self.warmups_since_clear = min(255, self.warmups_since_clear + 1)
                    self.dtc.warmup_cycle()
        for m in self.monitors:
            if m.supported:
                m.step(self, dt)
        self.iumpr.step(self, dt)

    # ------------------------------------------------------------------ outputs
    @property
    def mil(self) -> int:
        misfire = self._by_key["misfire"]
        if getattr(misfire, "cat_damage", False):
            return MIL_FLASH
        return MIL_ON if self.dtc.mil_on else MIL_OFF

    def readiness_pid01(self) -> tuple[int, int, int, int]:
        a = (0x80 if self.dtc.mil_on else 0) | min(len(self.dtc.confirmed), 0x7F)
        b = c = d = 0
        for m in self.monitors:
            if not m.supported:
                continue
            if m.continuous:
                b |= 1 << m.bit
                if not m.ready:
                    b |= 1 << (m.bit + 4)
            else:
                c |= 1 << m.bit
                if not m.ready:
                    d |= 1 << m.bit
        return a, b, c, d

    def readiness_pid41(self) -> tuple[int, int, int, int]:
        b = c = d = 0
        for m in self.monitors:
            if not m.supported:
                continue
            if m.continuous:
                if m.enabled_cycle:
                    b |= 1 << m.bit
                if not m.ready_this_cycle:
                    b |= 1 << (m.bit + 4)
            else:
                if m.enabled_cycle:
                    c |= 1 << m.bit
                if not m.ready_this_cycle:
                    d |= 1 << m.bit
        return 0, b, c, d

    # ------------------------------------------------------------------ Mode 04
    def clear(self) -> None:
        self.dtc.clear()
        self.mode06.clear()
        for m in self.monitors:
            m.clear()
        self.warmups_since_clear = 0
        self.dist_since_clear_km = 0.0
        self.time_since_clear_s = 0.0
        self.dist_mil_km = 0.0
        self.time_mil_s = 0.0

    # ------------------------------------------------------------------ persistence
    def export_kam(self) -> dict:
        return {
            "dtc": self.dtc.export(),
            "mode06": self.mode06.export(),
            "monitors": {m.key: m.export() for m in self.monitors},
            "iumpr": self.iumpr.export(),
            "counters": {
                "warmups_since_clear": self.warmups_since_clear,
                "dist_since_clear_km": self.dist_since_clear_km,
                "time_since_clear_s": self.time_since_clear_s,
                "dist_mil_km": self.dist_mil_km,
                "time_mil_s": self.time_mil_s,
            },
            "trips": self.trips,
        }

    def load_kam(self, d: dict) -> None:
        self.dtc.load(d.get("dtc", {}))
        self.mode06.load(d.get("mode06", {}))
        for key, md in d.get("monitors", {}).items():
            if key in self._by_key:
                self._by_key[key].load(md)
        self.iumpr.load(d.get("iumpr", {}))
        for k, v in d.get("counters", {}).items():
            if hasattr(self, k):
                setattr(self, k, type(getattr(self, k))(v))
        self.trips = int(d.get("trips", 0))
