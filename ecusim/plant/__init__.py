# SPDX-License-Identifier: GPL-3.0-or-later
"""The physical engine (true state). The ECU never reads this directly."""

from __future__ import annotations

import random

from ..calib.engine import SPEC, VE_RPM, EngineSpec, klsa_deg, mbt_deg
from ..core.signals import Actuators, DriverInputs, Environment
from ..core.tables import clamp, lowpass_alpha
from .air import AirPath
from .electrical import Electrical
from .engine import EngineCore
from .evap import EvapSystem
from .exhaust import Exhaust
from .fuel import FuelSystem, vaporisation
from .thermal import Thermal
from .vehicle import Vehicle

__all__ = ["Plant"]


class Plant:
    def __init__(self, env: Environment, rng: random.Random, dt: float, spec: EngineSpec | None = None):
        self.spec = spec or SPEC
        self.env = env
        self.rng = rng
        baro_pa = env.baro_kpa * 1000.0
        self.air = AirPath(self.spec, baro_pa)
        self.fuel = FuelSystem(self.spec, rng)
        self.engine = EngineCore(self.spec, rng)
        self.exhaust = Exhaust(self.spec, env.ambient_c, dt)
        self.thermal = Thermal(self.spec, env.ambient_c)
        self.elec = Electrical(self.spec)
        self.cam_deg = 0.0
        self.mbt = 0.0
        self.klsa = 0.0
        self.vap = 1.0
        self.vehicle = Vehicle()
        self.vehicle_kmh = 0.0
        self.tank_l = 45.0
        self.fuel_l = 27.0
        self.starter_interlock = True  # starter needs clutch pressed or neutral
        self.starter_engaged = False
        self.evap = EvapSystem()
        # Fault hooks
        self.cam_offset_deg = 0.0  # timing chain stretch: true cam phase vs crank
        self.cam_sensor_offset_deg = 0.0  # what the CMP reads on top (chain stretch shows up here too)
        self.vvt_stuck_deg: float | None = None
        self.fuel_pressure_factor = 1.0
        self.limp_throttle = 0.08  # default (spring) opening with the throttle motor unpowered

    # ----------------------------------------------------------------- setup helpers
    def soak(self, temp_c: float) -> None:
        """Bring every temperature to ``temp_c`` (engine left standing long enough)."""
        self.thermal.soak(temp_c)
        ex = self.exhaust
        ex.egt_c = ex.wall_c = ex.cat_in_c = ex.cat_c = temp_c

    def set_warm(self, coolant_c: float = 90.0, oil_c: float = 95.0, cat_c: float = 450.0) -> None:
        """Hot-soak state, e.g. a restart a few minutes after a drive."""
        t = self.thermal
        t.coolant_c, t.oil_c = coolant_c, oil_c
        t.underhood_c = self.env.ambient_c + 0.45 * (coolant_c - self.env.ambient_c)
        t.iat_c = self.env.ambient_c + 0.35 * (t.underhood_c - self.env.ambient_c)
        ex = self.exhaust
        ex.cat_c = cat_c
        ex.wall_c = ex.egt_c = ex.cat_in_c = 0.8 * cat_c

    # ----------------------------------------------------------------- simulation
    def step(self, dt: float, act: Actuators, drv: DriverInputs) -> None:
        env = self.env
        baro_pa = env.baro_kpa * 1000.0
        th = self.thermal
        eng = self.engine
        rpm = eng.rpm
        key_on = drv.ecu_powered

        air_gps = self.air.throttle_flow * 1000.0
        t_up_k = th.iat_c + 273.15
        t_charge_k = th.charge_temp_c(air_gps) + 273.15

        # Intake cam phaser (oil-pressure driven, slow when cold)
        cam_target = act.vvt_target_deg if rpm > 600.0 and key_on else 0.0
        tau_cam = 0.15 if th.oil_c > 60.0 else 0.5
        if self.vvt_stuck_deg is not None:
            self.cam_deg = self.vvt_stuck_deg
        else:
            self.cam_deg += (cam_target - self.cam_deg) * lowpass_alpha(dt, tau_cam)
        self.cam_sensor_offset_deg = self.cam_offset_deg

        cam_eff = self.cam_deg - 0.5 * self.cam_offset_deg
        ve = VE_RPM(rpm) * (1.0 + 0.0015 * cam_eff * clamp((rpm - 1500.0) / 2000.0, 0.0, 1.0))
        ve *= 1.0 - 0.004 * abs(self.cam_offset_deg)
        if not key_on:
            throttle_cmd = 0.0
        elif not act.throttle_enable:
            throttle_cmd = self.limp_throttle
        else:
            throttle_cmd = act.throttle_target

        # EVAP: purge flow depends on manifold vacuum from the previous step
        fuel_temp = env.ambient_c + 0.3 * (th.underhood_c - env.ambient_c)
        self.evap.step(dt, act.purge_duty if key_on else 0.0, act.vent_closed and key_on, rpm > 300.0,
                       env.baro_kpa - self.air.map_pa / 1000.0, self.fuel_l, self.tank_l, fuel_temp)
        self.air.step(dt, throttle_cmd, rpm, t_up_k, t_charge_k, baro_pa, ve, self.evap.purge_air_gps / 1000.0)

        self.fuel.step(
            dt,
            act.pw_ms if key_on else [0.0] * 4,
            rpm,
            self.elec.bus_v,
            self.air.map_pa,
            baro_pa,
            th.coolant_c,
            act.fuel_pump and key_on,
        )
        self.vap = vaporisation(th.coolant_c)
        if self.evap.purge_fuel_gps > 0.0 and rpm > 50.0:
            share = self.evap.purge_fuel_gps / 1000.0 / 4.0
            self.fuel.arriving = [a + share for a in self.fuel.arriving]

        cycles = rpm / 120.0
        air_cyl_g = (self.air.cyl_flow / 4.0) / cycles * 1000.0 if cycles > 0.1 else 0.0
        load_rel = air_cyl_g / (1.184 * 0.3995)
        self.mbt = mbt_deg(rpm, load_rel, th.coolant_c)
        self.klsa = klsa_deg(rpm, load_rel, th.coolant_c, th.iat_c, env.fuel_octane)
        eng.klsa = self.klsa
        spark = act.spark_deg

        veh = self.vehicle
        if drv.gear != veh.gear and not veh.shift(drv.gear, drv.clutch):
            drv.gear = veh.gear  # refused (clutch not pressed): stays in the old gear
        starter_on = drv.starter and (
            not self.starter_interlock or veh.gear == 0 or drv.clutch >= veh.spec.shift_clutch_min
        )
        self.starter_engaged = starter_on
        accessory = self.elec.alt_torque_nm + 1.0 + (6.0 if drv.ac and rpm > 500 else 0.0)
        eng.step(
            dt,
            self.air.cyl_flow,
            self.fuel.arriving,
            self.vap,
            spark,
            self.mbt,
            th.coolant_c,
            th.oil_c,
            self.air.map_pa,
            self.exhaust.p_exh_pa,
            starter_on,
            self.elec.bus_v,
            accessory,
            0.0,
            key_on,
            integrate=False,
        )
        veh.step(dt, eng, drv.clutch, drv.brake, drv.handbrake)
        self.vehicle_kmh = veh.kmh
        self.fuel_l = max(0.0, self.fuel_l - sum(self.fuel.injected) / 0.745 * dt)
        self.fuel.pressure_factor = self.fuel_pressure_factor if self.fuel_l > 0.05 else 0.0

        fuel_total = sum(self.fuel.arriving)
        combusting = eng.fuel_burned > 0.0
        mis_mean = eng.misfire_fueled_mean
        self.exhaust.step(
            dt,
            self.air.cyl_flow + fuel_total,
            self.air.cyl_flow,
            fuel_total,
            mis_mean,
            max(0.0, self.mbt - spark),
            th.coolant_c,
            env.ambient_c,
            baro_pa,
            combusting,
        )

        running = eng.rpm > 300.0
        th.step(
            dt,
            running,
            eng.fuel_burned * self.spec.lhv_j_per_kg,
            eng.friction_power,
            env.ambient_c,
            abs(self.vehicle_kmh),
            act.fan if key_on else 0,
            air_gps,
        )

        noload = self.spec.starter_noload_rpm
        starter_frac = clamp(1.0 - eng.rpm / noload, 0.0, 1.0)
        self.elec.step(
            dt,
            key_on,
            starter_on,
            eng.rpm,
            eng.omega,
            starter_frac,
            act.fan if key_on else 0,
            act.wb_heater and key_on,
            act.nb_heater and key_on,
            drv.lights,
        )
