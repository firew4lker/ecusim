# SPDX-License-Identifier: GPL-3.0-or-later
"""Coolant, oil, under-hood and intake air temperatures (DESIGN §4.5)."""

from __future__ import annotations

from ..calib.engine import EngineSpec
from ..core.tables import clamp, lowpass_alpha


class Thermal:
    def __init__(self, spec: EngineSpec, ambient_c: float):
        self.spec = spec
        self.soak(ambient_c)
        self.thermostat = 0.0
        self.q_radiator = 0.0
        # Fault hooks
        self.thermostat_stuck: float | None = None  # fixed opening 0..1 if stuck
        self.fan_failed = False

    def soak(self, temp_c: float) -> None:
        self.coolant_c = temp_c
        self.oil_c = temp_c
        self.underhood_c = temp_c
        self.iat_c = temp_c

    def charge_temp_c(self, air_gps: float) -> float:
        return self.iat_c + (self.coolant_c - self.iat_c) * 0.2 / (1.0 + air_gps / 15.0)

    def step(
        self,
        dt: float,
        running: bool,
        q_fuel_w: float,
        friction_w: float,
        ambient_c: float,
        vehicle_kmh: float,
        fan_level: int,
        air_gps: float,
        heater_w_k: float = 0.0,
    ) -> None:
        s = self.spec
        tc = self.coolant_c

        if self.thermostat_stuck is not None:
            opening = self.thermostat_stuck
        else:
            opening = clamp((tc - s.thermostat_open_c) / (s.thermostat_full_c - s.thermostat_open_c), 0.0, 1.0)
        self.thermostat = opening
        fan = 0 if self.fan_failed else fan_level
        ua_rad = 50.0 + min(8.0 * vehicle_kmh, 900.0) + (300.0 if fan == 1 else 550.0 if fan == 2 else 0.0)
        pumping = running
        q_rad = opening * ua_rad * (tc - ambient_c) if pumping else 0.0
        self.q_radiator = q_rad
        ua_block = (s.block_ua_running_w_k + 0.3 * vehicle_kmh) if running else s.block_ua_off_w_k

        q_cool = s.coolant_fuel_fraction * q_fuel_w
        dq_c = (
            q_cool
            - ua_block * (tc - ambient_c)
            - s.coolant_oil_ua_w_k * (tc - self.oil_c)
            - q_rad
            - heater_w_k * (tc - ambient_c)
        )
        q_oil = 0.7 * friction_w + 0.03 * q_fuel_w
        ua_co = s.coolant_oil_ua_w_k if running else 0.25 * s.coolant_oil_ua_w_k
        dq_o = q_oil + ua_co * (tc - self.oil_c) - s.oil_ambient_ua_w_k * (self.oil_c - ambient_c)
        self.coolant_c += dt * dq_c / s.coolant_heat_capacity_j_k
        self.oil_c += dt * dq_o / s.oil_heat_capacity_j_k

        uh_target = ambient_c + 0.45 * (self.coolant_c - ambient_c) / (1.0 + vehicle_kmh / 20.0)
        self.underhood_c += (uh_target - self.underhood_c) * lowpass_alpha(dt, 200.0)
        iat_target = ambient_c + (self.underhood_c - ambient_c) * 0.35 / (1.0 + air_gps / 8.0)
        self.iat_c += (iat_target - self.iat_c) * lowpass_alpha(dt, 20.0)
