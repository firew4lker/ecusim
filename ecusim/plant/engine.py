# SPDX-License-Identifier: GPL-3.0-or-later
"""Combustion torque, friction, pumping, starter and crankshaft rotation (DESIGN §4.3, §4.4)."""

from __future__ import annotations

import math
import random

from ..calib.engine import ETA_LAMBDA, ETA_LOAD, EngineSpec, fmep_bar
from ..core.tables import clamp

TWO_PI = 2.0 * math.pi
RPM_PER_RADS = 60.0 / TWO_PI
FULL_CHARGE_G = 1.184 * 0.3995  # g of air per cylinder at STP


class EngineCore:
    def __init__(self, spec: EngineSpec, rng: random.Random):
        self.spec = spec
        self.rng = rng
        self.omega = 0.0
        self.revs = 0.0
        self.lambda_comb = [9.99] * 4
        self.misfire = [0.0] * 4  # effective misfire fraction this step
        self.torque_ind = 0.0
        self.torque_brake = 0.0
        self.torque_friction = 0.0
        self.torque_pump = 0.0
        self.torque_starter = 0.0
        self.torque_net = 0.0
        self.fuel_burned = 0.0  # kg/s vaporised fuel that burned
        self.friction_power = 0.0
        self.load_rel = 0.0
        # Fault hooks
        self.misfire_fault = [0.0] * 4  # 0..1 probability per event
        self.compression = [1.0] * 4
        self.coil_failed = [False] * 4
        self.plug_wear = [0.0] * 4  # 0..1: misfire probability rising with load
        self.knock_intensity = 0.0
        self.misfire_fueled_mean = 0.0
        self.klsa = 99.0  # set by the plant each step

    @property
    def rpm(self) -> float:
        return self.omega * RPM_PER_RADS

    def step(
        self,
        dt: float,
        cyl_air: float,
        arriving: list[float],
        vap: float,
        spark_deg: float,
        mbt_deg: float,
        coolant_c: float,
        oil_c: float,
        map_pa: float,
        p_exh_pa: float,
        starter_on: bool,
        vbatt: float,
        accessory_nm: float,
        load_nm: float,
        ignition: bool = True,
        integrate: bool = True,
    ) -> None:
        s = self.spec
        omega = self.omega
        rpm = omega * RPM_PER_RADS
        warm = clamp((coolant_c - 20.0) / 70.0, 0.0, 1.0)

        cycles = rpm / 120.0  # cycles per second per cylinder
        air_cyl_g = (cyl_air / 4.0) / cycles * 1000.0 if cycles > 0.1 else 0.0
        self.load_rel = air_cyl_g / FULL_CHARGE_G

        eta_ind = s.eta_ind_peak * ETA_LOAD(self.load_rel) * (0.92 + 0.08 * warm)
        delta = min(60.0, abs(mbt_deg - spark_deg))
        eta_spark = max(0.0, math.cos(math.radians(1.8 * delta)))

        power = 0.0
        burned = 0.0
        firing = rpm > 70.0 and ignition
        air_i = cyl_air / 4.0
        fueled = [
            firing and air_i > 0.0 and arriving[i] * vap > 1.0e-9 and air_i / (arriving[i] * vap * s.afr_stoich) <= 2.5
            for i in range(4)
        ]
        some_fire = any(fueled)
        for i in range(4):
            fv = arriving[i] * vap
            if not fueled[i]:
                # No fuel. If every cylinder is unfuelled it is a fuel cut (nothing to detect); if only this
                # one is dead while the others fire, the crankshaft sees the missing power stroke: a misfire.
                lam = 9.99
                eff = 0.0
                mis = 1.0 if some_fire else 0.0
            else:
                lam = min(9.99, air_i / (fv * s.afr_stoich))
                eff = ETA_LAMBDA(lam)
                lean_mis = clamp((lam - 1.40) / 0.2, 0.0, 1.0)
                plug = self.plug_wear[i] * (0.02 + 0.6 * self.load_rel ** 2)
                weak = clamp((0.85 - self.compression[i]) * 2.0, 0.0, 1.0) * clamp(1.2 - self.load_rel, 0.0, 1.0)
                mis = 1.0 if self.coil_failed[i] else clamp(self.misfire_fault[i] + lean_mis + plug + weak, 0.0, 1.0)
            self.lambda_comb[i] = lam
            self.misfire[i] = mis
            if eff > 0.0:
                frac = (1.0 - mis) * self.compression[i]
                power += (air_i / s.afr_stoich) * s.lhv_j_per_kg * eta_ind * eta_spark * eff * frac
                burned += min(fv, air_i / s.afr_stoich) * (1.0 - mis)
        self.fuel_burned = burned
        # unburned fuel+air from fuelled cylinders that misfired (an unfuelled cylinder only pumps air)
        self.misfire_fueled_mean = sum(self.misfire[i] for i in range(4) if fueled[i]) / 4.0
        over = spark_deg - self.klsa
        self.knock_intensity = clamp(over, 0.0, 15.0) * 0.4 * clamp(self.load_rel / 0.6, 0.0, 1.5) if firing else 0.0
        if over > 0.0 and firing:
            power *= 1.0 - 0.01 * min(over, 15.0)

        t_ind = power / max(omega, 10.0)
        if t_ind > 0.0:
            cov = 0.012 + 0.025 * (1.0 - warm) + (0.02 if self.load_rel < 0.3 else 0.0)
            t_ind *= 1.0 + self.rng.gauss(0.0, cov)
        self.torque_ind = t_ind

        vd_4pi = s.displacement_m3 / (4.0 * math.pi)
        oil_fac = 1.0 + 0.5 * clamp((90.0 - oil_c) / 105.0, 0.0, 1.2)
        moving = clamp(omega / 2.0, 0.0, 1.0)
        t_fric = fmep_bar(rpm) * 1.0e5 * oil_fac * vd_4pi * moving
        t_fric += s.compression_loss_nm * moving * clamp((500.0 - rpm) / 300.0, 0.0, 1.0)
        t_pump = max(0.0, p_exh_pa - map_pa) * vd_4pi * clamp(omega / 10.0, 0.0, 1.0)
        self.torque_friction = t_fric
        self.torque_pump = t_pump
        self.friction_power = t_fric * omega

        if starter_on:
            noload = s.starter_noload_rpm / RPM_PER_RADS
            t_st = s.starter_stall_nm * (vbatt / 12.0) * max(0.0, 1.0 - omega / noload)
        else:
            t_st = 0.0
        self.torque_starter = t_st

        t_brake = t_ind - t_fric - t_pump - accessory_nm * moving
        self.torque_brake = t_brake
        self.torque_net = t_brake + t_st  # everything except the clutch
        if integrate:
            self.set_speed(omega + (self.torque_net - load_nm) / s.inertia_kgm2 * dt, dt)

    def set_speed(self, omega: float, dt: float) -> None:
        """Apply the new crank speed (from own integration or from the coupled driveline)."""
        self.omega = max(0.0, omega)
        self.revs += self.omega * dt / TWO_PI
