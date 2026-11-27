# SPDX-License-Identifier: GPL-3.0-or-later
"""Exhaust: transport delay, gas temperature, manifold, catalyst thermal + oxygen storage (DESIGN §4.6)."""

from __future__ import annotations

import math
from collections import deque

from ..calib.engine import EngineSpec
from ..core.tables import clamp, lowpass_alpha

CP_EXH = 1150.0  # J/(kg K)
LAMBDA_AIR = 9.99


class Exhaust:
    def __init__(self, spec: EngineSpec, ambient_c: float, dt: float):
        self.spec = spec
        self.dt = dt
        self._buf: deque[float] = deque([LAMBDA_AIR] * 80, maxlen=80)
        self.lambda_port = LAMBDA_AIR
        self.lambda_b1s1 = LAMBDA_AIR  # gas at the upstream sensor (delayed)
        self.lambda_post = LAMBDA_AIR  # gas after the catalyst
        self.egt_c = ambient_c
        self.wall_c = ambient_c
        self.cat_in_c = ambient_c
        self.cat_c = ambient_c
        self.osc_state = 0.5
        self.cat_conversion = 0.0
        self.flow = 0.0
        self.p_exh_pa = 101300.0
        # Fault hooks
        self.cat_health = 1.0
        self.pre_sensor_leak = 0.0  # fraction of ambient air diluting the gas at B1S1

    def step(
        self,
        dt: float,
        exh_flow: float,
        air_flow: float,
        fuel_flow_total: float,
        misfire_mean: float,
        retard_deg: float,
        coolant_c: float,
        ambient_c: float,
        baro_pa: float,
        combusting: bool,
    ) -> None:
        self.flow = exh_flow
        self.p_exh_pa = baro_pa + 1000.0 + 2.5e6 * exh_flow * exh_flow

        # Mixed lambda at the exhaust port (all fuel, burned or not)
        if fuel_flow_total > 1.0e-7 and air_flow > 0.0:
            lam = air_flow / (fuel_flow_total * self.spec.afr_stoich)
            lam *= 1.0 + 0.6 * misfire_mean  # unburned charge reads lean on the sensor
        else:
            lam = LAMBDA_AIR
        lam = min(lam, LAMBDA_AIR)
        self.lambda_port = lam
        self._buf.append(lam)

        # Transport delay port -> B1S1
        gps = exh_flow * 1000.0
        delay = 0.04 + 0.2 * (2.6 / max(gps, 1.0)) if gps > 0.05 else 0.0
        idx = int(round(delay / self.dt))
        idx = min(max(idx, 0), len(self._buf) - 1)
        lam_s = self._buf[-1 - idx]
        if self.pre_sensor_leak > 0.0 and lam_s < LAMBDA_AIR:
            lam_s = lam_s / max(1.0 - self.pre_sensor_leak, 0.05)
        self.lambda_b1s1 = min(lam_s, LAMBDA_AIR)

        # Exhaust gas temperature at the port
        warm = clamp((coolant_c - 20.0) / 70.0, 0.0, 1.0)
        if combusting and gps > 0.05:
            lam_c = self.lambda_port
            egt_ss = ambient_c + 330.0 + 500.0 * math.sqrt(min(1.0, gps / 95.0)) + 9.0 * retard_deg
            egt_ss -= 120.0 * (1.0 - warm)
            if lam_c < 1.0:
                egt_ss -= 400.0 * (1.0 - lam_c)
            egt_ss += 300.0 * misfire_mean
        elif gps > 0.05:
            egt_ss = 0.5 * (self.wall_c + ambient_c)  # pumping air through
        else:
            egt_ss = self.wall_c
        self.egt_c += (egt_ss - self.egt_c) * lowpass_alpha(dt, 0.5)

        # Exhaust manifold wall and catalyst inlet gas temperature
        eps = 0.35
        g_cp = exh_flow * CP_EXH
        q_wall = eps * g_cp * (self.egt_c - self.wall_c) - 1.5 * (self.wall_c - ambient_c)
        self.wall_c += dt * q_wall / self.spec.exh_manifold_heat_capacity_j_k
        self.cat_in_c = self.egt_c - eps * (self.egt_c - self.wall_c) if gps > 0.05 else self.wall_c

        # Catalyst front zone
        conv = 1.0 / (1.0 + math.exp(-(self.cat_c - 270.0) / 20.0))
        self.cat_conversion = conv * self.cat_health
        exo = g_cp * self.cat_conversion * (50.0 + 3000.0 * misfire_mean) if combusting else 0.0
        q_cat = 0.8 * g_cp * (self.cat_in_c - self.cat_c) + exo - 0.5 * (self.cat_c - ambient_c)
        self.cat_c += dt * q_cat / self.spec.cat_front_heat_capacity_j_k

        # Oxygen storage
        act = clamp((self.cat_c - 200.0) / 150.0, 0.0, 1.0)
        cap = self.spec.cat_osc_nominal_g * self.cat_health * act
        lam_in = self.lambda_port
        if gps > 0.05 and cap > 1.0e-4:
            flux = gps * 0.232 * (1.0 - 1.0 / max(lam_in, 0.5))
            self.osc_state += dt * flux / cap - dt * (self.osc_state - 0.5) / 20.0
            self.osc_state = clamp(self.osc_state, 0.0, 1.0)
        s = self.osc_state
        eff = act * self.cat_health
        if lam_in >= 1.0:
            c = eff * (1.0 - s ** 8)
        else:
            c = eff * (1.0 - (1.0 - s) ** 8)
        self.lambda_post = min(LAMBDA_AIR, 1.0 + (lam_in - 1.0) * (1.0 - c) - 0.003 * eff)
