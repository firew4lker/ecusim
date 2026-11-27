# SPDX-License-Identifier: GPL-3.0-or-later
"""Engine control module: operating modes and glue between the sub-controllers (DESIGN §5)."""

from __future__ import annotations

import math

from ..calib.engine import mbt_deg
from ..core.signals import KEY_START, Actuators, Readings
from ..core.tables import clamp, lowpass_alpha
from ..plant.air import R_AIR
from . import cal
from .airctl import AirControl
from .diag import Diagnostics
from .fuel import FuelControl
from .inputs import InputProcessor
from .purge import PurgeControl
from .spark import SparkControl

MODE_OFF = "off"
MODE_KOEO = "koeo"
MODE_CRANK = "crank"
MODE_RUN = "run"


class ECM:
    def __init__(self, region: str = "eu") -> None:
        self.region = region
        self.fuel = FuelControl()
        self.air = AirControl()
        self.spark = SparkControl()
        self.purge = PurgeControl()
        self.inputs = InputProcessor()
        self.act = Actuators()
        self.mode = MODE_OFF
        self.t = 0.0
        self.baro_kpa = 101.3
        self.ambient_est_c = 15.0
        self.cat_est_c = 15.0
        self._baro_stable = 0.0
        self._last_r = Readings()
        self.evap_request: tuple[bool, float] | None = None  # (vent closed, purge duty) from the EVAP monitor
        self.purge_inhibit = False  # set by the fuel-system monitor's intrusive check
        self.diag = Diagnostics(self)
        self._power_down()

    # ------------------------------------------------------------------ helpers
    def _power_down(self) -> None:
        if hasattr(self, "diag"):
            self.diag.power_down()
        self.mode = MODE_OFF
        self.t_key = 0.0
        self.t_run = 0.0
        self.ect_start = 15.0
        self.iat_start = 15.0
        self.synced = False
        self._revs_start: float | None = None
        self._map_prev: float | None = None
        self._dmap = 0.0
        self.cyl_flow_gps = 0.0
        self.air_cyl_g = 0.0
        self.load_rel = 0.0
        self.calc_load = 0.0
        self.pedal_pct = 0.0
        self.cat_heat = 0.0
        self.wb_ready = False
        self.nb_ready = False
        self._nb_dew_timer = 0.0
        self.fan = 0
        self.evap_request = None
        self.purge_inhibit = False
        self.fuel.reset()
        self.air.reset()
        self.purge.reset()
        self.inputs.reset()

    @property
    def running(self) -> bool:
        return self.mode == MODE_RUN

    @property
    def limp(self) -> bool:
        return self.inputs.tps_limp

    @property
    def epc(self) -> bool:
        return self.inputs.epc

    # ------------------------------------------------------------------ main step
    def step(self, dt: float, raw: Readings, key: str) -> Actuators:
        self.t += dt
        act = self.act
        if key not in ("on", KEY_START):
            if self.mode != MODE_OFF:
                self._power_down()
            # engine-off catalyst cool-down model keeps running on keep-alive power
            self.cat_est_c += (self.ambient_est_c - self.cat_est_c) * lowpass_alpha(dt, 600.0)
            act.throttle_target = 0.0
            act.throttle_enable = True
            act.pw_ms = [0.0] * 4
            act.spark_deg = 0.0
            act.fuel_pump = act.wb_heater = act.nb_heater = act.alternator_enable = False
            act.fan = 0
            act.vvt_target_deg = 0.0
            act.purge_duty = 0.0
            act.vent_closed = False
            self._last_r = raw
            return act

        if self.mode == MODE_OFF:
            self.mode = MODE_KOEO
            self.t_key = 0.0
            self.diag.power_up()
            self._baro_stable = 0.0
            if raw.rpm < 5.0 and abs(raw.ect_c - raw.iat_c) < 7.0:
                self.ambient_est_c = raw.iat_c
        self.t_key += dt

        # ---- validated inputs (circuit checks, failsafes); control uses these
        r = self.inputs.process(dt, raw, self, key)
        inp = self.inputs

        # ---- crank/cam synchronisation (without a cam signal: slower, batch fire / waste spark)
        sync_revs = cal.SYNC_REVS * (2.0 if inp.cmp_failed or not raw.cmp_ok else 1.0)
        if r.rpm > 20.0:
            if self._revs_start is None:
                self._revs_start = r.crank_revs
            self.synced = (r.crank_revs - self._revs_start) >= sync_revs
        else:
            self._revs_start = None
            self.synced = False

        # ---- mode transitions
        if self.mode == MODE_KOEO:
            # Learn baro from MAP only once the manifold has equalised with the engine stopped
            steady = r.rpm < 5.0 and abs(self._dmap) < 1.0 and 60.0 < r.map_kpa < 110.0 and not inp.map_failed
            self._baro_stable = self._baro_stable + dt if steady else 0.0
            if self._baro_stable > 0.5:
                self.baro_kpa += (r.map_kpa - self.baro_kpa) * lowpass_alpha(dt, 0.2)
            if r.rpm > 30.0 and key == KEY_START:
                self.mode = MODE_CRANK
                self.ect_start = r.ect_c
                self.iat_start = r.iat_c
                self.fuel.reset()
                self.air.reset()
        elif self.mode == MODE_CRANK:
            if r.rpm > cal.RUN_RPM and self.synced:
                self.mode = MODE_RUN
                self.t_run = 0.0
                self.diag.engine_started(r.ect_c)
            elif r.rpm < 20.0:
                self.mode = MODE_KOEO
        elif self.mode == MODE_RUN:
            self.t_run += dt
            if r.rpm < cal.STALL_RPM and key != KEY_START:
                self.mode = MODE_KOEO
                self.t_run = 0.0
                self.fuel.reset()
                self.air.reset()
                self.purge.reset()

        # ---- air charge estimate (speed-density backup when the MAF has failed)
        rpm = r.rpm
        if self._map_prev is None:
            self._map_prev = r.map_kpa
        rate = (r.map_kpa - self._map_prev) / dt
        self._map_prev = r.map_kpa
        self._dmap += (clamp(rate, -400.0, 400.0) - self._dmap) * lowpass_alpha(dt, 0.08)
        t_charge_k = r.iat_c + (r.ect_c - r.iat_c) * 0.2 + 273.15
        if rpm > 500.0 and self.mode == MODE_RUN and not inp.maf_failed:
            dmap = self._dmap if abs(self._dmap) > 30.0 else 0.0  # ignore sensor noise; real transients are fast
            fill = 0.0 if inp.map_failed else cal.MANIFOLD_VOLUME_M3 / (R_AIR * (r.iat_c + 273.15)) * dmap * 1.0e6
            cyl = max(0.0, r.maf_gps - clamp(fill, -0.5 * r.maf_gps, 0.5 * r.maf_gps))
        elif rpm > 20.0:
            cyl = (
                cal.VE_NOMINAL(rpm)
                * cal.DISPLACEMENT_M3
                * (rpm / 120.0)
                * max(0.0, r.map_kpa * 1000.0 - 6000.0)
                / (R_AIR * t_charge_k)
                * 1000.0
            )
        else:
            cyl = 0.0
        self.cyl_flow_gps = cyl
        cycles = rpm / 120.0
        self.air_cyl_g = cyl / (4.0 * cycles) if cycles > 0.25 else 0.0
        self.load_rel = self.air_cyl_g / cal.FULL_CHARGE_G
        peak = (
            cal.VE_NOMINAL(rpm) * cal.DISPLACEMENT_M3 * (rpm / 120.0) * (101325.0 - 6000.0) / (R_AIR * 298.15) * 1000.0
        )
        self.calc_load = clamp(cyl / peak, 0.0, 1.0) if peak > 0.01 else 0.0

        self.pedal_pct = 0.0 if self.limp else clamp((r.app_d_pct - 15.0) / 72.0 * 100.0, 0.0, 100.0)

        # ---- catalyst heating request
        eligible = self.mode == MODE_RUN and cal.CAT_HEAT_ECT_MIN <= self.ect_start <= cal.CAT_HEAT_ECT_MAX
        want = eligible and cal.CAT_HEAT_START_S <= self.t_run < cal.CAT_HEAT_END_S and self.air.idle_active
        if want:
            self.cat_heat = min(1.0, self.cat_heat + dt / 1.5)
        else:
            self.cat_heat = max(0.0, self.cat_heat - dt / 5.0)

        # ---- O2 sensor heaters / readiness
        self.wb_ready = r.wb_element_c > cal.WB_READY_C and r.wb_signal_ok
        if self.mode == MODE_RUN and self.cat_est_c > cal.NB_DEWPOINT_CAT_C:
            self._nb_dew_timer += dt
        elif self.mode != MODE_RUN:
            self._nb_dew_timer = 0.0
        nb_heater = self._nb_dew_timer > cal.NB_DEWPOINT_HOLD_S
        self.nb_ready = nb_heater and r.nb_element_c > cal.NB_READY_C

        # ---- throttle / idle (motor off in limp home: the blade springs to its default opening)
        act.throttle_enable = not self.limp
        act.throttle_target = self.air.step(
            dt, self.mode, rpm, r.ect_c, self.t_run, self.pedal_pct, self.cat_heat, r.map_kpa, self.baro_kpa,
            r.iat_c, False, r.vss_kmh, r.clutch_switch, self.fuel.dfco,
        )

        # ---- spark with knock control
        self.spark.knock_control(dt, raw.knock, rpm, self.mode == MODE_RUN, inp.knock_failed)
        act.spark_deg = self.spark.step(
            self.mode, rpm, self.load_rel, r.ect_c, r.iat_c, self.air.idle_active, self.air.err, self.cat_heat
        )

        # ---- EVAP purge (or the leak test's own valve commands)
        purge_ok = (
            self.mode == MODE_RUN and self.fuel.closed_loop and r.ect_c > 70.0 and self.t_run > cal.PURGE_MIN_RUN_S
            and not self.fuel.dfco and not self.fuel.pe and not inp.wb_failed and not self.purge_inhibit
        )
        override = self.evap_request[1] if self.evap_request is not None else None
        act.purge_duty = self.purge.step(dt, purge_ok, r.map_kpa, self.baro_kpa, self.fuel.stft,
                                         self.cyl_flow_gps / cal.AFR_STOICH, override)
        act.vent_closed = bool(self.evap_request and self.evap_request[0])

        # ---- fuel
        self.fuel.step(
            dt, self.t, self.mode, self.synced, rpm, self.cyl_flow_gps, self.load_rel, r.ect_c, self.ect_start,
            self.t_run, self.pedal_pct, self.air.idle_active, r.vss_kmh, r.map_kpa, self.baro_kpa, r.vbatt,
            r.wb_lambda * (4 - sum(inp.dead_cyl)) / 4.0 if any(inp.dead_cyl) else r.wb_lambda,
            self.wb_ready, r.nb_volts, self.nb_ready, self.cat_est_c, self.cat_heat,
            wb_failed=inp.wb_failed,
            purge_fuel_gps=self.purge.fuel_est_gps,
            ltft_hold=self.purge.owns_error(self.fuel.stft_i),
            rev_limit=cal.LIMP_REV_LIMIT if self.limp else None,
        )
        pw = self.fuel.pw_ms
        act.pw_ms = [0.0 if inp.coil_cut[i] else pw for i in range(4)]  # no fuel into a cylinder without spark
        if self.mode == MODE_CRANK and not self.synced:
            act.spark_deg = 0.0

        # ---- modelled catalyst temperature (reported as PID 3C)
        if self.mode == MODE_RUN:
            warm = clamp((r.ect_c - 20.0) / 70.0, 0.0, 1.0)
            retard = max(0.0, mbt_deg(rpm, self.load_rel, r.ect_c) - act.spark_deg)
            egt = self.ambient_est_c + 300.0 + 500.0 * math.sqrt(min(1.0, self.cyl_flow_gps / 95.0)) + 7.0 * retard
            egt -= 120.0 * (1.0 - warm)
            if self.fuel.dfco:
                egt = 0.5 * (self.cat_est_c + self.ambient_est_c)
            tau = 45.0 * math.sqrt(3.0 / max(self.cyl_flow_gps, 1.0))
            target = 0.85 * egt + 40.0 * (1.0 if self.cat_est_c > 300.0 else 0.0)
            self.cat_est_c += (target - self.cat_est_c) * lowpass_alpha(dt, tau)
        else:
            self.cat_est_c += (self.ambient_est_c - self.cat_est_c) * lowpass_alpha(dt, 600.0)

        # ---- misc actuators
        act.fuel_pump = self.t_key < 2.0 or rpm > 20.0
        act.wb_heater = self.mode == MODE_RUN and self.t_run > cal.WB_HEATER_DELAY_S
        act.nb_heater = nb_heater
        act.alternator_enable = self.mode == MODE_RUN
        ect = r.ect_c
        if inp.ect_failed and self.mode != MODE_OFF:
            self.fan = 2  # no coolant temperature: run the fan
        elif ect >= cal.FAN_HIGH_ON or (self.fan == 2 and ect > cal.FAN_HIGH_OFF):
            self.fan = 2
        elif ect >= cal.FAN_LOW_ON or (self.fan >= 1 and ect > cal.FAN_LOW_OFF):
            self.fan = 1
        else:
            self.fan = 0
        act.fan = self.fan
        if self.mode == MODE_RUN and r.oil_c > cal.VVT_MIN_OIL_C and rpm > 1000.0 and not self.air.idle_active:
            act.vvt_target_deg = cal.VVT_TARGET(rpm, self.load_rel)
        else:
            act.vvt_target_deg = 0.0
        self._last_r = raw
        self.diag.step(dt, r)
        return act

    # ------------------------------------------------------------------ OBD view
    def obd_values(self, r: Readings | None = None) -> dict:
        """Everything generic OBD reports, in engineering units (also what a freeze frame stores)."""
        r = r or self._last_r
        f = self.fuel
        d = self.diag
        running = self.mode == MODE_RUN
        return {
            "running": running,
            "rpm": r.rpm if r.rpm > 30.0 else 0.0,
            "mon01": d.readiness_pid01(),
            "mon41": d.readiness_pid41(),
            "fuel_status": f.status,
            "calc_load": self.calc_load * 100.0,
            "abs_load": self.load_rel * 100.0,
            "ect": r.ect_c,
            "stft": f.stft * 100.0,
            "ltft": f.ltft_active * 100.0,
            "map": r.map_kpa,
            "vss": r.vss_kmh,
            "spark": self.act.spark_deg,
            "iat": r.iat_c,
            "maf": r.maf_gps,
            "tps_a": r.tps_a_pct,
            "tps_b": r.tps_b_pct,
            "tps_rel": max(0.0, r.tps_a_pct - 14.0),
            "app_d": r.app_d_pct,
            "app_e": r.app_e_pct,
            "throttle_cmd": self.act.throttle_target * 100.0 if self.act.throttle_enable else 0.0,
            "nb_volts": r.nb_volts,
            "nb_ready": self.nb_ready,
            "nb_trim_pct": -f.dtrim * 100.0,
            "wb_ready": self.wb_ready,
            "wb_lambda": r.wb_lambda,
            "wb_pump_ma": r.wb_pump_ma,
            "lambda_cmd": f.lambda_cmd if running else 0.0,
            "obd_standard": 0x06,
            "run_time": self.t_run if running else 0.0,
            "dist_mil": d.dist_mil_km,
            "dist_clear": d.dist_since_clear_km,
            "time_mil": d.time_mil_s / 60.0,
            "time_clear": d.time_since_clear_s / 60.0,
            "warmups": d.warmups_since_clear,
            "purge": self.act.purge_duty * 100.0,
            "fuel_level": r.fuel_level_pct,
            "evap_pa": r.tank_pa,
            "baro": self.baro_kpa,
            "cat_temp": self.cat_est_c,
            "vbatt": r.vbatt,
            "ambient": self.ambient_est_c,
            "oil": r.oil_c,
            "fuel_rate": f.injected_gps * 3.6 / 0.745 if running else 0.0,
        }

    # ------------------------------------------------------------------ keep-alive memory
    def export_kam(self) -> dict:
        return {"ltft": list(self.fuel.ltft), "purge_conc": self.purge.conc, "baro_kpa": self.baro_kpa,
                "diag": self.diag.export_kam()}

    def import_kam(self, d: dict) -> None:
        if "ltft" in d:
            self.fuel.ltft = [float(x) for x in d["ltft"]][:4]
        if "purge_conc" in d:
            self.purge.conc = float(d["purge_conc"])
        if "baro_kpa" in d:
            self.baro_kpa = float(d["baro_kpa"])
        if "diag" in d:
            self.diag.load_kam(d["diag"])
