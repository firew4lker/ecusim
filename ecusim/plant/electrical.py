# SPDX-License-Identifier: GPL-3.0-or-later
"""Battery, alternator, starter current and electrical loads (DESIGN §4.8)."""

from __future__ import annotations

import math

from ..calib.engine import EngineSpec
from ..core.tables import clamp


class Electrical:
    def __init__(self, spec: EngineSpec):
        self.spec = spec
        self.soc = 0.9
        self.bus_v = self.ocv
        self.load_a = 0.0
        self.alt_a = 0.0
        self.alt_torque_nm = 0.0
        self.run_time = 0.0
        # Fault hooks
        self.alternator_failed = False
        self.regulator_v: float | None = None
        self.weak_battery = 0.0  # 0..1, raises internal resistance

    @property
    def ocv(self) -> float:
        return 11.8 + 0.9 * self.soc

    def step(
        self,
        dt: float,
        key_on: bool,
        starter_on: bool,
        rpm: float,
        omega: float,
        starter_frac: float,
        fan_level: int,
        wb_heater: bool,
        nb_heater: bool,
        lights: bool,
    ) -> None:
        s = self.spec
        load = 0.3  # always-on keep-alive
        if key_on:
            load += 12.0
            load += 3.0 if wb_heater else 0.0
            load += 1.5 if nb_heater else 0.0
            load += 12.0 if fan_level == 1 else 22.0 if fan_level == 2 else 0.0
            load += 10.0 if lights else 0.0
        if starter_on:
            load += 40.0 + s.starter_current_a * starter_frac
        self.load_a = load

        r_int = s.battery_r_ohm * (1.0 + 6.0 * self.weak_battery)
        running = rpm > 500.0 and not starter_on
        self.run_time = self.run_time + dt if running else 0.0
        alt_max = 0.0 if self.alternator_failed else 90.0 * clamp((rpm - 500.0) / 1500.0, 0.0, 1.0) + (
            40.0 if rpm > 650.0 else 0.0
        )
        alt_max = min(alt_max, 90.0)
        setpoint = self.regulator_v if self.regulator_v is not None else s.alternator_setpoint_v

        if running and alt_max > 0.0:
            charge = 2.0 + 25.0 * math.exp(-self.run_time / 90.0) * (1.0 - self.soc + 0.1)
            demand = load + charge
            if demand <= alt_max:
                self.alt_a = demand
                self.bus_v = setpoint
                batt_i = charge
            else:
                self.alt_a = alt_max
                batt_i = alt_max - load  # negative = discharging
                self.bus_v = self.ocv + batt_i * r_int
        else:
            self.alt_a = 0.0
            batt_i = -load
            self.bus_v = self.ocv - load * r_int

        self.soc = clamp(self.soc + batt_i * dt / (3600.0 * s.battery_capacity_ah), 0.0, 1.0)
        p_alt = self.alt_a * self.bus_v
        self.alt_torque_nm = p_alt / (0.55 * omega) if omega > 20.0 else 0.0
