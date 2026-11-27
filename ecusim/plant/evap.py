"""Evaporative emission system: tank vapour space, carbon canister, purge and vent valves, leaks (DESIGN §4.7)."""

from __future__ import annotations

import math

from ..core.tables import clamp

R_AIR = 287.05
CANISTER_CAPACITY_G = 100.0
PURGE_MAX_GPS = 0.8  # purge valve flow at full duty and >= 30 kPa manifold vacuum
HC_PER_AIR = 0.3  # g of fuel vapour per g of purge air from a saturated canister
VENT_AREA_M2 = 1.0e-4  # open vent / filler path
CD = 0.6


def orifice_kgps(area_m2: float, dp_pa: float, rho: float = 1.2) -> float:
    """Signed flow into the tank for a pressure difference (atmosphere - tank)."""
    if area_m2 <= 0.0 or dp_pa == 0.0:
        return 0.0
    return math.copysign(CD * area_m2 * math.sqrt(2.0 * rho * abs(dp_pa)), dp_pa)


class EvapSystem:
    def __init__(self) -> None:
        self.tank_pa = 0.0  # relative to atmosphere
        self.canister_load = 0.3  # 0..1
        self.purge_air_gps = 0.0
        self.purge_fuel_gps = 0.0
        # Fault hooks
        self.leak_diameter_mm = 0.0
        self.vent_stuck_open = False
        self.purge_stuck: str | None = None  # None | "open" | "closed"

    def step(self, dt: float, purge_duty: float, vent_closed: bool, running: bool, manifold_vac_kpa: float,
             fuel_l: float, tank_l: float, fuel_temp_c: float) -> None:
        duty = clamp(purge_duty, 0.0, 1.0)
        if self.purge_stuck == "open":
            duty = max(duty, 0.6)
        elif self.purge_stuck == "closed":
            duty = 0.0
        vac = max(0.0, manifold_vac_kpa) if running else 0.0
        q_air = duty * PURGE_MAX_GPS * math.sqrt(clamp(vac / 30.0, 0.0, 1.0))  # g/s

        gen = 0.0005 * math.exp((fuel_temp_c - 20.0) / 15.0) * (0.5 + 0.5 * (1.0 - fuel_l / tank_l))  # g/s
        vent_open = (not vent_closed) or self.vent_stuck_open
        volume = max(0.002, (tank_l - fuel_l) / 1000.0) + 0.004
        k = R_AIR * (273.15 + fuel_temp_c) / volume  # Pa per kg

        leak_area = math.pi * (self.leak_diameter_mm / 2000.0) ** 2
        if vent_open:
            # Vented: the tank sits at atmosphere; vapour goes into the canister, purge pulls fresh air through it
            self.tank_pa = 0.0
            self.canister_load = clamp(self.canister_load + gen * dt / CANISTER_CAPACITY_G, 0.0, 1.0)
        else:
            # Sealed for the leak test: purge draws gas out of the tank side, leaks let air back in
            m_dot = gen / 1000.0 - q_air / 1000.0 + orifice_kgps(leak_area, -self.tank_pa)
            self.tank_pa = clamp(self.tank_pa + k * m_dot * dt, -6000.0, 6000.0)
        hc = q_air * HC_PER_AIR * self.canister_load
        self.canister_load = clamp(self.canister_load - hc * dt / CANISTER_CAPACITY_G, 0.0, 1.0)
        self.purge_air_gps = q_air
        self.purge_fuel_gps = hc
