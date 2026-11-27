# SPDX-License-Identifier: GPL-3.0-or-later
"""Ignition timing (DESIGN §5.5)."""

from __future__ import annotations

from ..core.tables import clamp
from . import cal


class SparkControl:
    def __init__(self) -> None:
        self.advance = 0.0
        self.base = 0.0
        self.idle_correction = 0.0
        self.knock_retard = [0.0, 0.0, 0.0, 0.0]
        self.knock_events = 0

    def knock_control(self, dt: float, knock: float, rpm: float, running: bool, failed: bool) -> None:
        """Retard on knock, creep back slowly; fixed safety retard if the knock sensor is dead."""
        if failed:
            self.knock_retard = [cal.KNOCK_FAILSAFE_DEG] * 4
            return
        background = 0.25 + 0.1 * rpm / 1000.0
        knocking = running and knock > background + cal.KNOCK_MARGIN
        for i in range(4):
            if knocking:
                self.knock_retard[i] = min(cal.KNOCK_MAX_DEG, self.knock_retard[i] + cal.KNOCK_STEP_DEG)
            else:
                self.knock_retard[i] = max(0.0, self.knock_retard[i] - cal.KNOCK_RECOVER_DEG_S * dt)
        if knocking:
            self.knock_events += 1

    def step(
        self,
        mode: str,
        rpm: float,
        load_rel: float,
        ect: float,
        iat: float,
        idle_active: bool,
        idle_err_rpm: float,
        cat_heat: float,
    ) -> float:
        if mode == "crank":
            self.advance = cal.CRANK_SPARK
            self.idle_correction = 0.0
            return self.advance
        if mode != "run":
            self.advance = 0.0
            return 0.0

        base = cal.SPARK_BASE(rpm, load_rel)
        part_load = clamp((load_rel - 0.2) / 0.2, 0.0, 1.0)
        base += 4.0 * clamp((60.0 - ect) / 60.0, 0.0, 1.0) * part_load
        base -= 0.15 * max(0.0, iat - 40.0)
        self.base = base

        if idle_active:
            idle = cal.IDLE_SPARK(ect)
            idle += (cal.CAT_HEAT_SPARK - idle) * cat_heat
            self.idle_correction = clamp(
                cal.IDLE_SPARK_KP * idle_err_rpm, -cal.IDLE_SPARK_LIMIT, cal.IDLE_SPARK_LIMIT
            )
            adv = min(idle + self.idle_correction, base + 5.0)
        else:
            self.idle_correction = 0.0
            adv = base - 0.5 * cat_heat * max(0.0, base - cal.CAT_HEAT_SPARK)
        adv -= max(self.knock_retard)
        self.advance = clamp(adv, -10.0, 50.0)
        return self.advance
