# SPDX-License-Identifier: GPL-3.0-or-later
"""Simulation loop: driver -> sensors -> ECU -> plant, at a fixed 10 ms step (DESIGN §3.2)."""

from __future__ import annotations

import random
from typing import Callable

from .core.signals import KEY_ON, KEY_START, DriverInputs, Environment
from .ecu.ecm import ECM, MODE_RUN
from .faults.manager import FaultManager
from .plant import Plant
from .sensors.sensors import Sensors

DT = 0.01


class Simulator:
    def __init__(self, ambient_c: float = 15.0, baro_kpa: float = 101.3, seed: int = 1):
        self.dt = DT
        self.t = 0.0
        self.rng = random.Random(seed)
        self.env = Environment(ambient_c=ambient_c, baro_kpa=baro_kpa)
        self.plant = Plant(self.env, self.rng, DT)
        self.plant.soak(ambient_c)
        self.sensors = Sensors(self.rng)
        self.ecu = ECM()
        self.driver = DriverInputs()
        self.faults = FaultManager()
        self.readings = self.sensors.update(DT, self.plant, self.ecu.act, self.driver)
        self._start_req: tuple[float, float] | None = None

    # ------------------------------------------------------------------ stepping
    def step(self) -> None:
        if self._start_req is not None:
            self._advance_start()
        f = self.faults
        if f.active or f._applied or self.sensors.faults:
            f.apply(self)
        self.readings = self.sensors.update(self.dt, self.plant, self.ecu.act, self.driver)
        act = self.ecu.step(self.dt, self.readings, self.driver.key)
        self.plant.step(self.dt, act, self.driver)
        self.t += self.dt

    def run(self, seconds: float, every: Callable[["Simulator"], None] | None = None) -> None:
        n = int(round(seconds / self.dt))
        for _ in range(n):
            self.step()
            if every is not None:
                every(self)

    # ------------------------------------------------------------------ driver helpers
    def start_engine(self, prime_s: float = 1.0, max_crank_s: float = 5.0) -> float:
        """Key on, wait for the pump prime, crank until the ECU reports the engine running.

        Returns the cranking time in seconds, or ``-1`` if the engine did not start.
        """
        self.driver.key = KEY_ON
        self.run(prime_s)
        self.driver.key = KEY_START
        t0 = self.t
        while self.t - t0 < max_crank_s:
            self.step()
            if self.ecu.mode == MODE_RUN:
                break
        crank = self.t - t0
        self.driver.key = KEY_ON
        return crank if self.ecu.mode == MODE_RUN else -1.0

    def request_start(self, prime_s: float = 1.0, max_crank_s: float = 5.0) -> None:
        """Non-blocking start: key on, wait for the prime, crank until running (or give up)."""
        if self.ecu.mode == MODE_RUN:
            return
        self.driver.key = KEY_ON
        self._start_req = (self.t + prime_s, self.t + prime_s + max_crank_s)

    def _advance_start(self) -> None:
        t_crank, t_give_up = self._start_req
        if self.ecu.mode == MODE_RUN or self.t >= t_give_up or self.driver.key not in (KEY_ON, KEY_START):
            if self.driver.key == KEY_START:
                self.driver.key = KEY_ON
            self._start_req = None
        elif self.t >= t_crank:
            self.driver.key = KEY_START

    # ------------------------------------------------------------------ observation
    def snapshot(self) -> dict[str, float]:
        p = self.plant
        e = self.ecu
        r = self.readings
        f = e.fuel
        return {
            "t": self.t,
            "mode": {"off": 0, "koeo": 1, "crank": 2, "run": 3}[e.mode],
            "rpm": p.engine.rpm,
            "rpm_target": e.air.target_rpm,
            "kmh": p.vehicle.kmh,
            "gear": float(p.vehicle.gear),
            "clutch": self.driver.clutch * 100.0,
            "brake": self.driver.brake * 100.0,
            "clutch_locked": float(p.vehicle.locked),
            "clutch_torque": p.vehicle.clutch_torque,
            "odometer_km": p.vehicle.odometer_m / 1000.0,
            "trip_km": p.vehicle.trip_m / 1000.0,
            "key": float({"off": 0, "acc": 1, "on": 2, "start": 3}[self.driver.key]),
            "starter": float(p.starter_engaged),
            "alt_a": p.elec.alt_a,
            "shift_refused": float(p.vehicle.last_shift_refused),
            "handbrake": float(self.driver.handbrake),
            "lights": float(self.driver.lights),
            "ac": float(self.driver.ac),
            "ambient_c": self.env.ambient_c,
            "fuel_pct": p.fuel_l / p.tank_l * 100.0,
            "mil": float(e.diag.mil),
            "epc": float(e.epc),
            "limp": float(e.limp),
            "knock_retard": max(e.spark.knock_retard),
            "purge_pct": e.act.purge_duty * 100.0,
            "tank_pa": r.tank_pa,
            "canister": p.evap.canister_load,
            "faults_active": float(len(self.faults.active)),
            "fuel_l": p.fuel_l,
            "pedal": self.driver.pedal * 100.0,
            "throttle_pct": p.air.throttle * 100.0,
            "tps_a_pct": r.tps_a_pct,
            "map_kpa": r.map_kpa,
            "maf_gps": r.maf_gps,
            "maf_true_gps": p.air.throttle_flow * 1000.0,
            "cyl_flow_gps": p.air.cyl_flow * 1000.0,
            "calc_load_pct": e.calc_load * 100.0,
            "abs_load_pct": e.load_rel * 100.0,
            "ect_c": r.ect_c,
            "coolant_true_c": p.thermal.coolant_c,
            "oil_c": p.thermal.oil_c,
            "iat_c": r.iat_c,
            "thermostat": p.thermal.thermostat,
            "fan": float(e.fan),
            "spark_deg": e.act.spark_deg,
            "mbt_deg": p.mbt,
            "pw_ms": f.pw_ms,
            "fuel_status": float(f.status),
            "lambda_cmd": f.lambda_cmd,
            "lambda_wb": r.wb_lambda,
            "lambda_true": p.exhaust.lambda_port,
            "lambda_comb": sum(p.engine.lambda_comb) / 4.0,
            "stft_pct": f.stft * 100.0,
            "ltft_pct": f.ltft_active * 100.0,
            "dtrim": f.dtrim,
            "wb_ready": float(e.wb_ready),
            "nb_ready": float(e.nb_ready),
            "nb_volts": r.nb_volts,
            "cat_c": p.exhaust.cat_c,
            "cat_est_c": e.cat_est_c,
            "egt_c": p.exhaust.egt_c,
            "osc_state": p.exhaust.osc_state,
            "cat_heat": e.cat_heat,
            "vbatt": r.vbatt,
            "torque_ind": p.engine.torque_ind,
            "torque_brake": p.engine.torque_brake,
            "vvt_deg": p.cam_deg,
            "dfco": float(f.dfco),
            "pe": float(f.pe),
        }
