# SPDX-License-Identifier: GPL-3.0-or-later
"""Sensor layer: converts the plant's true state into what each sensor reports (DESIGN §3.1).

Each sensor has its own dynamics, tolerance and noise. Faults are applied here (signal faults in
engineering units, circuit faults on the wire voltage) so the ECU only ever sees ``Readings`` and
has to work out what is wrong by itself.
"""

from __future__ import annotations

import math
import random

from ..core.signals import Actuators, DriverInputs, Readings
from ..core.tables import clamp, lowpass_alpha
from ..plant import Plant
from . import transfer as tf
from .faults import GAIN, NOISY, OFFSET, OPEN, SHORT_GND, SHORT_V, SLOW, STUCK, SensorFault


def narrowband_volts(lam: float) -> float:
    """Zirconia switching curve, ~0.45 V at stoichiometry."""
    z = (lam - 0.999) * 400.0
    z = clamp(z, -50.0, 50.0)
    return 0.08 + 0.80 / (1.0 + math.exp(z))


def wideband_pump_ma(lam: float) -> float:
    """Approximate pump current of a 5-wire planar wideband sensor."""
    if lam >= 1.0:
        return 2.5 * (1.0 - 1.0 / lam) * 1.6  # ~2.5 mA in free air
    return -3.0 * (1.0 / lam - 1.0) * 1.8


def poisson(rng: random.Random, lam: float) -> int:
    if lam <= 0.0:
        return 0
    limit = math.exp(-lam)
    k, p = 0, 1.0
    while True:
        p *= rng.random()
        if p <= limit:
            return k
        k += 1


# Channels that accept faults, for the fault catalogue
SENSOR_CHANNELS = ("maf", "map", "iat", "ect", "tps_a", "tps_b", "app_d", "app_e", "wb", "nb", "knock", "ckp",
                   "cmp", "tank", "fuel_level", "vss")


class Sensors:
    def __init__(self, rng: random.Random):
        self.rng = rng
        self.r = Readings()
        self.faults: dict[str, SensorFault] = {}
        # heater/driver hardware faults (set by the fault manager)
        self.wb_heater_failed = False
        self.nb_heater_failed = False
        self.maf_contamination = 0.0  # dirty hot film: under-reads, more at high flow
        self._init = False
        self._hold: dict[str, float] = {}
        self._slow: dict[str, float] = {}
        self._wb_lambda = 1.0
        self._wb_elem = 15.0
        self._nb_volts = 0.45
        self._nb_elem = 15.0
        self._maf = 0.0
        self._ect = 15.0
        self._iat = 15.0
        self._oil = 15.0

    # ------------------------------------------------------------------ fault helpers
    def _sig(self, name: str, value: float, dt: float) -> float:
        """Signal-level fault in engineering units."""
        f = self.faults.get(name)
        if f is None or f.mode in (OPEN, SHORT_GND, SHORT_V):
            self._hold[name] = value
            self._slow[name] = value
            return value
        if f.mode == OFFSET:
            return value + f.amount
        if f.mode == GAIN:
            return value * (1.0 + f.amount)
        if f.mode == STUCK:
            return self._hold.setdefault(name, value)
        if f.mode == SLOW:
            s = self._slow.get(name, value)
            s += (value - s) * lowpass_alpha(dt, max(f.amount, 0.01))
            self._slow[name] = s
            return s
        if f.mode == NOISY:
            return value + self.rng.gauss(0.0, f.amount)
        return value

    def _wire(self, name: str, volts: float) -> float:
        """Circuit fault on the signal wire."""
        f = self.faults.get(name)
        if f is not None:
            if f.mode == OPEN:
                return tf.OPEN_VOLTS.get(name, 0.0)
            if f.mode == SHORT_GND:
                return 0.0
            if f.mode == SHORT_V:
                return tf.VREF
        return clamp(volts, 0.0, tf.VREF)

    def _circuit_bad(self, name: str) -> bool:
        f = self.faults.get(name)
        return f is not None and f.mode in (OPEN, SHORT_GND, SHORT_V)

    # ------------------------------------------------------------------ update
    def update(self, dt: float, plant: Plant, act: Actuators, drv: DriverInputs) -> Readings:
        g = self.rng.gauss
        r = self.r
        th = plant.thermal
        if not self._init:
            self._ect = th.coolant_c
            self._iat = th.iat_c
            self._oil = th.oil_c
            self._wb_elem = plant.exhaust.egt_c
            self._nb_elem = plant.exhaust.cat_c
            self._init = True

        # MAF (hot film, ~15 ms, +/- pulsation noise at low flow)
        true_maf = plant.air.throttle_flow * 1000.0 * plant.spec.maf_gain
        self._maf += (true_maf - self._maf) * lowpass_alpha(dt, 0.015)
        maf = max(0.0, self._maf * (1.0 + g(0.0, 0.006)) + g(0.0, 0.02))
        if self.maf_contamination:
            maf *= 1.0 - self.maf_contamination * (0.5 + 0.5 * min(1.0, maf / 40.0))
        r.maf_v = self._wire("maf", tf.maf_to_v(max(0.0, self._sig("maf", maf, dt))))
        r.maf_gps = tf.v_to_maf(r.maf_v)

        mapk = self._sig("map", plant.air.map_pa / 1000.0 + g(0.0, 0.15), dt)
        r.map_v = self._wire("map", tf.map_to_v(mapk))
        r.map_kpa = tf.v_to_map(r.map_v)

        self._iat += (th.iat_c - self._iat) * lowpass_alpha(dt, 4.0)
        r.iat_v = self._wire("iat", tf.ntc_to_v(self._sig("iat", self._iat + g(0.0, 0.05), dt)))
        r.iat_c = tf.v_to_ntc(r.iat_v)
        self._ect += (th.coolant_c - self._ect) * lowpass_alpha(dt, 3.0)
        r.ect_v = self._wire("ect", tf.ntc_to_v(self._sig("ect", self._ect + g(0.0, 0.05), dt)))
        r.ect_c = tf.v_to_ntc(r.ect_v)
        self._oil += (th.oil_c - self._oil) * lowpass_alpha(dt, 5.0)
        r.oil_c = self._oil

        x = plant.air.throttle
        r.tps_a_v = self._wire("tps_a", tf.tps_a_to_v(self._sig("tps_a", x, dt)) + g(0.0, 0.002))
        r.tps_b_v = self._wire("tps_b", tf.tps_b_to_v(self._sig("tps_b", x, dt)) + g(0.0, 0.002))
        r.tps_a_pct = r.tps_a_v / tf.VREF * 100.0
        r.tps_b_pct = r.tps_b_v / tf.VREF * 100.0
        pedal = clamp(drv.pedal, 0.0, 1.0)
        r.app_d_v = self._wire("app_d", tf.app_d_to_v(self._sig("app_d", pedal, dt)) + g(0.0, 0.002))
        r.app_e_v = self._wire("app_e", tf.app_e_to_v(self._sig("app_e", pedal, dt)) + g(0.0, 0.002))
        r.app_d_pct = r.app_d_v / tf.VREF * 100.0
        r.app_e_pct = r.app_e_v / tf.VREF * 100.0

        rpm = plant.engine.rpm
        ckp_bad = "ckp" in self.faults
        r.ckp_ok = not ckp_bad
        r.rpm = 0.0 if ckp_bad else rpm + (g(0.0, 2.0) if rpm > 100.0 else 0.0)
        if not ckp_bad:
            r.crank_revs = plant.engine.revs
        r.cmp_ok = "cmp" not in self.faults
        r.cam_active = r.cmp_ok and rpm > 50.0
        r.cam_deg = self._sig("cmp_angle", plant.cam_deg + plant.cam_sensor_offset_deg + g(0.0, 0.2), dt)

        # Wideband B1S1: heater-controlled element; output valid only when hot
        ex = plant.exhaust
        powered = drv.ecu_powered
        heat = act.wb_heater and powered and not self.wb_heater_failed
        if heat:
            self._wb_elem += (780.0 - self._wb_elem) * lowpass_alpha(dt, 5.5)
        else:
            self._wb_elem += (0.8 * ex.egt_c - self._wb_elem) * lowpass_alpha(dt, 20.0)
        r.wb_element_c = self._wb_elem
        r.wb_heater_ok = not self.wb_heater_failed
        gas = min(ex.lambda_b1s1, 3.0)  # pump current saturates: free air reads ~3
        self._wb_lambda += (gas - self._wb_lambda) * lowpass_alpha(dt, 0.10)
        wb = self._sig("wb", self._wb_lambda * (1.0 + g(0.0, 0.002)), dt)
        r.wb_signal_ok = not self._circuit_bad("wb")
        r.wb_lambda = clamp(wb, 0.6, 4.0) if r.wb_signal_ok else 4.0
        r.wb_pump_ma = wideband_pump_ma(r.wb_lambda)

        # Narrowband B1S2 after the catalyst
        heat = act.nb_heater and powered and not self.nb_heater_failed
        if heat:
            self._nb_elem += (650.0 - self._nb_elem) * lowpass_alpha(dt, 8.0)
        else:
            self._nb_elem += (ex.cat_c - self._nb_elem) * lowpass_alpha(dt, 30.0)
        r.nb_element_c = self._nb_elem
        r.nb_heater_ok = not self.nb_heater_failed
        activity = clamp((self._nb_elem - 250.0) / 150.0, 0.0, 1.0)
        v_target = 0.45 + (narrowband_volts(ex.lambda_post) - 0.45) * activity
        self._nb_volts += (v_target - self._nb_volts) * lowpass_alpha(dt, 0.15)
        nb = self._sig("nb", self._nb_volts + g(0.0, 0.004), dt)
        f = self.faults.get("nb")
        if f is not None and f.mode == SHORT_V:
            nb = 1.1  # shorted to the heater supply side: pegged high
        elif f is not None and f.mode in (OPEN, SHORT_GND):
            nb = self._wire("nb", nb)
        r.nb_volts = clamp(nb, 0.0, 1.1)

        r.vbatt = plant.elec.bus_v + g(0.0, 0.02)
        r.vss_kmh = 0.0 if "vss" in self.faults else abs(plant.vehicle_kmh)
        lvl = self._sig("fuel_level", plant.fuel_l / plant.tank_l * 100.0 + g(0.0, 0.2), dt)
        r.fuel_level_v = self._wire("fuel_level", tf.level_to_v(lvl))
        r.fuel_level_pct = clamp(tf.v_to_level(r.fuel_level_v), 0.0, 100.0)
        r.tank_v = self._wire("tank", tf.tank_to_v(self._sig("tank", plant.evap.tank_pa + g(0.0, 3.0), dt)))
        r.tank_pa = tf.v_to_tank(r.tank_v)
        r.brake_switch = drv.brake > 0.02
        r.clutch_switch = drv.clutch > 0.8

        # Knock sensor: mechanical background noise plus knock energy
        r.knock_ok = "knock" not in self.faults
        if r.knock_ok and rpm > 300.0:
            r.knock = 0.2 + 0.1 * rpm / 1000.0 + abs(g(0.0, 0.05)) + plant.engine.knock_intensity
        else:
            r.knock = 0.0

        # Driver-stage feedback (flyback / current sense)
        r.injector_ok = [h > 0.0 for h in plant.fuel.injector_health]
        r.coil_ok = [not c for c in plant.engine.coil_failed]
        r.fan_ok = not plant.thermal.fan_failed

        # Misfire detection from crankshaft segment times (per-cylinder deceleration)
        events = rpm / 120.0 * dt
        mis = plant.engine.misfire
        if not ckp_bad:
            r.misfire_counts = [c + poisson(self.rng, mis[i] * events) for i, c in enumerate(r.misfire_counts)]
        return r
