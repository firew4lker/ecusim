# SPDX-License-Identifier: GPL-3.0-or-later
"""Fuel delivery: pump/rail, injectors, port wall-wetting (x-tau), vaporisation (DESIGN §4.2)."""

from __future__ import annotations

import math
import random

from ..calib.engine import INJ_DEADTIME_MS, EngineSpec
from ..core.tables import clamp, lowpass_alpha


def film_params(t_port_c: float, map_pa: float) -> tuple[float, float]:
    """Fraction of injected fuel that lands on the port wall, and its evaporation time constant (s)."""
    warm = clamp(t_port_c / 80.0, 0.0, 1.0)
    x = 0.55 - 0.35 * warm + 0.10 * (map_pa / 1.0e5)
    tau = 0.25 + 1.5 * (1.0 - warm)
    return x, tau


def vaporisation(t_port_c: float) -> float:
    """Fraction of fuel arriving in the cylinder that actually takes part in combustion."""
    return 0.88 + 0.12 * clamp((t_port_c + 20.0) / 90.0, 0.0, 1.0)


class FuelSystem:
    def __init__(self, spec: EngineSpec, rng: random.Random | None = None):
        self.spec = spec
        self.rng = rng or random.Random(0)
        self.disturbance = 0.0  # slow fuel-delivery wander (volatility, rail ripple, temperature)
        self.rail_kpa = 0.0
        self.films = [0.0, 0.0, 0.0, 0.0]  # kg on each port wall
        self.injected = [0.0] * 4  # kg/s per cylinder
        self.arriving = [0.0] * 4  # kg/s per cylinder reaching the cylinder
        self.x = 0.0
        self.tau = 0.0
        # Fault hooks
        self.injector_health = [1.0] * 4  # flow factor
        self.injector_leak_kgps = [0.0] * 4
        self.pressure_factor = 1.0

    def step(
        self,
        dt: float,
        pw_ms: list[float],
        rpm: float,
        vbatt: float,
        map_pa: float,
        baro_pa: float,
        t_port_c: float,
        pump_on: bool,
    ) -> None:
        s = self.spec
        target = s.rail_kpa * self.pressure_factor if pump_on else 0.0
        tau_rail = 0.3 if pump_on else 60.0  # check valve holds pressure for a while
        self.rail_kpa += (target - self.rail_kpa) * lowpass_alpha(dt, tau_rail)

        dp = self.rail_kpa + (baro_pa - map_pa) / 1000.0
        flow = s.injector_flow_mg_ms * math.sqrt(max(dp, 0.0) / s.rail_kpa)
        dead = INJ_DEADTIME_MS(vbatt)
        tau_d, sigma_d = 4.0, 0.015
        self.disturbance += -self.disturbance * dt / tau_d + sigma_d * math.sqrt(2.0 * dt / tau_d) * self.rng.gauss(
            0.0, 1.0
        )
        wander = 1.0 + self.disturbance
        inj_per_s = rpm / 120.0
        total = 0.0
        for i in range(4):
            mg = max(0.0, pw_ms[i] - dead) * flow * self.injector_health[i] * wander if self.rail_kpa > 50.0 else 0.0
            rate = mg * 1.0e-6 * inj_per_s + self.injector_leak_kgps[i]
            self.injected[i] = rate
            total += rate

        x, tau = film_params(t_port_c, map_pa)
        if rpm < 50.0:
            tau = 60.0  # no airflow to strip the film
        self.x, self.tau = x, tau
        for i in range(4):
            evap = self.films[i] / tau
            self.films[i] = max(0.0, self.films[i] + dt * (x * self.injected[i] - evap))
            self.arriving[i] = (1.0 - x) * self.injected[i] + (evap if rpm >= 50.0 else 0.0)

    @property
    def film_kg(self) -> float:
        return sum(self.films)
