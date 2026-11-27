"""ECU input processing: circuit range checks, plausibility, debouncing, failsafe substitution
(DESIGN §6.3 comprehensive components, §5.1 limp home).

The ECU controls with the *validated* readings returned by :meth:`InputProcessor.process`; scan
tools still see the raw values (a broken coolant sensor reads -40 °C in Mode 01, as on a real car).
"""

from __future__ import annotations

import dataclasses
import math
from typing import TYPE_CHECKING

from ..core.signals import Readings
from ..core.tables import clamp
from ..plant.air import R_AIR, psi
from ..sensors import transfer as tf
from . import cal

if TYPE_CHECKING:
    from .ecm import ECM

DEBOUNCE_S = 0.5
PASS_REPORT_S = 5.0


class Check:
    """Debounced fault condition reporting one DTC."""

    def __init__(self, code: str, debounce: float = DEBOUNCE_S, pass_after: float = PASS_REPORT_S):
        self.code = code
        self.debounce = debounce
        self.pass_after = pass_after
        self.t_bad = 0.0
        self.t_good = 0.0
        self.active = False

    def update(self, dt: float, bad: bool, report, testable: bool = True) -> bool:
        if not testable:
            self.t_bad = 0.0
            self.t_good = 0.0
            return self.active
        if bad:
            self.t_good = 0.0
            self.t_bad += dt
            if self.t_bad >= self.debounce:
                if not self.active or self.t_bad >= self.debounce + 5.0:
                    report(self.code, True)
                    self.t_bad = self.debounce
                self.active = True
        else:
            self.t_bad = 0.0
            self.t_good += dt
            if self.t_good >= self.pass_after:
                report(self.code, False)
                self.t_good = 0.0
                self.active = False
        return self.active


class InputProcessor:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        c = Check
        self.checks = {
            "maf_lo": c("P0102"), "maf_hi": c("P0103"),
            "map_lo": c("P0107"), "map_hi": c("P0108"),
            "ect_lo": c("P0117"), "ect_hi": c("P0118"),
            "iat_lo": c("P0112"), "iat_hi": c("P0113"),
            "tpsa_lo": c("P0122", 0.1), "tpsa_hi": c("P0123", 0.1),
            "tpsb_lo": c("P0222", 0.1), "tpsb_hi": c("P0223", 0.1),
            "tps_corr": c("P2135", 0.2), "tps_track": c("P2101", 1.0),
            "appd_lo": c("P2122", 0.1), "appd_hi": c("P2123", 0.1),
            "appe_lo": c("P2127", 0.1), "appe_hi": c("P2128", 0.1),
            "app_corr": c("P2138", 0.2),
            "wb_circ": c("P0130", 1.0), "wb_heat": c("P0030", 2.0), "nb_heat": c("P0036", 2.0),
            "knock": c("P0327", 2.0), "cmp": c("P0340", 1.0),
            "tank_lo": c("P0452"), "tank_hi": c("P0453"),
            "fan": c("P0480", 2.0),
            "vss": c("P0500", 8.0, 20.0),
            "maf_rat": c("P0101", 10.0, 10.0),
            "level": c("P0461", 1.0, 60.0), "level_circ": c("P0461", 1.0),
            "tank_rat": c("P0451", 10.0, 10.0),
            "map_rat": c("P0106", 5.0, 10.0),
            **{f"inj{i}": c(f"P020{i + 1}", 0.5) for i in range(4)},
            **{f"coil{i}": c(f"P035{i + 1}", 0.5) for i in range(4)},
        }
        self.ckp_check = Check("P0335", 2.0)
        self.maf_failed = self.map_failed = self.ect_failed = self.iat_failed = False
        self.tps_limp = False
        self.app_scale = 1.0
        self.app_failed = False
        self.wb_failed = False
        self.knock_failed = False
        self.cmp_failed = False
        self.coil_cut = [False] * 4
        self.ect_stuck_ref: float | None = None
        self.ect_stuck_t = 0.0
        self.maf_verified_t = 1e9  # seconds since the MAF last agreed with the throttle model
        self.ect_model: float | None = None  # modelled coolant temperature (failsafe and plausibility)
        self.dead_cyl = [False] * 4
        self.fuel_used_g = 0.0
        self.level_ref: float | None = None
        self.raw = Readings()

    @property
    def epc(self) -> bool:
        """Electronic power control lamp: throttle/pedal faults."""
        return self.tps_limp or self.app_failed or self.app_scale < 1.0

    def process(self, dt: float, r: Readings, ecm: "ECM", key: str) -> Readings:
        self.raw = r
        rep = ecm.diag.dtc.report
        k = self.checks
        running = ecm.mode == "run"
        v = dataclasses.replace(r)

        # ---- MAF / MAP circuits
        maf_bad = k["maf_lo"].update(dt, r.maf_v < 0.2, rep) | k["maf_hi"].update(dt, r.maf_v > 4.8, rep)
        map_bad = k["map_lo"].update(dt, r.map_v < 0.2, rep) | k["map_hi"].update(dt, r.map_v > 4.85, rep)

        # ---- MAF rationality against the throttle-body flow model (needs a good TPS and MAP)
        # ---- Airflow sensor arbitration: MAF vs throttle model vs MAP.
        # With the throttle choked (pressure ratio < 0.6) the throttle-body model depends only on the
        # throttle angle, so it can referee between the MAF and the MAP.
        maf_suspect = False
        tps_agree = abs(r.tps_a_v + r.tps_b_v - tf.VREF) < 0.15
        steady = running and r.rpm > 600.0 and abs(ecm._dmap) < 3.0
        if steady and not maf_bad and not map_bad and not self.tps_limp and tps_agree:
            pr = clamp(r.map_kpa / max(ecm.baro_kpa, 50.0), 0.0, 0.98)
            opening = clamp(tf.v_to_tps_a(r.tps_a_v), 0.0, 1.0)
            if pr < 0.7 and opening > 0.08:
                cda = cal.THROTTLE_CDA_REST + cal.THROTTLE_CDA_FULL * (1.0 - math.cos(opening * math.pi / 2.0))
                iat = r.iat_c if 0.1 < r.iat_v < 4.9 else 25.0  # failsafe if the IAT circuit is bad
                model = cda * ecm.baro_kpa * 1000.0 / math.sqrt(R_AIR * (iat + 273.15)) * psi(pr) * 1000.0
                ratio = r.maf_gps / model if model > 0.5 else 1.0
                maf_suspect = ratio < 0.8 or ratio > 1.25
                k["maf_rat"].update(dt, maf_suspect, rep)
                if not maf_suspect:
                    self.maf_verified_t = 0.0
        self.maf_verified_t += dt
        if running and not maf_bad and not map_bad and r.rpm > 700.0 and abs(ecm._dmap) < 5.0 \
                and not maf_suspect and not k["maf_rat"].active and self.maf_verified_t < 60.0:
            iat = r.iat_c if 0.1 < r.iat_v < 4.9 else 25.0
            k_cyl = cal.VE_NOMINAL(r.rpm) * cal.DISPLACEMENT_M3 * (r.rpm / 120.0) / (R_AIR * (iat + 273.15 + 15.0))
            pred = (6000.0 + r.maf_gps / 1000.0 / k_cyl) / 1000.0
            k["map_rat"].update(dt, abs(r.map_kpa - pred) > max(15.0, 0.3 * pred), rep)
        elif running and not map_bad and ecm.air.idle_active and r.vss_kmh < 1.0 and r.rpm > 600.0 \
                and ecm.t_run > 5.0:
            # A closed-throttle idle always pulls vacuum: MAP near barometric means the sensor is stuck
            k["map_rat"].update(dt, r.map_kpa > 0.85 * ecm.baro_kpa, rep)
        self.maf_failed = maf_bad
        self.map_failed = map_bad
        if map_bad:
            # MAP substitute from the airflow model
            cyl = r.maf_gps if not maf_bad else 3.0
            k_cyl = cal.VE_NOMINAL(r.rpm) * cal.DISPLACEMENT_M3 * (r.rpm / 120.0) / (R_AIR * 310.0)
            v.map_kpa = clamp((6000.0 + cyl / 1000.0 / k_cyl) / 1000.0, 20.0, ecm.baro_kpa) if r.rpm > 200 else \
                ecm.baro_kpa

        # ---- temperatures
        ect_bad = k["ect_lo"].update(dt, r.ect_v < 0.1, rep) | k["ect_hi"].update(dt, r.ect_v > 4.9, rep)
        iat_bad = k["iat_lo"].update(dt, r.iat_v < 0.1, rep) | k["iat_hi"].update(dt, r.iat_v > 4.9, rep)
        self.ect_failed, self.iat_failed = ect_bad, iat_bad
        # Coolant model: starts at intake air temperature, warms with the fuel energy burned
        if self.ect_model is None:
            self.ect_model = r.iat_c if not iat_bad else 20.0
        if running:
            q = ecm.fuel.injected_gps * 43.0e3 * 0.27
            self.ect_model += dt * (q - 15.0 * (self.ect_model - ecm.ambient_est_c)) / 24000.0
            self.ect_model = min(self.ect_model, 92.0)
        if ect_bad:
            v.ect_c = self.ect_model
        if iat_bad:
            v.iat_c = 25.0
        # ECT stuck (P0116): cold start, the model has warmed 25 °C but the sensor has not moved
        if running and not ect_bad and ecm.ect_start < 50.0 and self.ect_stuck_t < 1e9:
            if self.ect_stuck_ref is None:
                self.ect_stuck_ref = (r.ect_c, self.ect_model)
            ect0, model0 = self.ect_stuck_ref
            if abs(r.ect_c - ect0) > 3.0:
                rep("P0116", False)
                self.ect_stuck_t = 1e9
            elif self.ect_model - model0 > 25.0:
                rep("P0116", True)
                self.ect_stuck_t = 1e9

        # ---- throttle position (dual track) and actuator tracking
        a_bad = k["tpsa_lo"].update(dt, r.tps_a_v < 0.2, rep) | k["tpsa_hi"].update(dt, r.tps_a_v > 4.8, rep)
        b_bad = k["tpsb_lo"].update(dt, r.tps_b_v < 0.2, rep) | k["tpsb_hi"].update(dt, r.tps_b_v > 4.8, rep)
        corr = k["tps_corr"].update(dt, abs(r.tps_a_v + r.tps_b_v - tf.VREF) > 0.3, rep, testable=not (a_bad or b_bad))
        meas = tf.v_to_tps_a(r.tps_a_v) if not a_bad else tf.v_to_tps_b(r.tps_b_v)
        target = ecm.act.throttle_target
        track = k["tps_track"].update(dt, abs(meas - target) > 0.08, rep,
                                      testable=ecm.act.throttle_enable and ecm.mode in ("run", "koeo"))
        self.tps_limp = self.tps_limp or a_bad or b_bad or corr or track  # latched until key off

        # ---- accelerator pedal (dual track 2:1)
        d_bad = k["appd_lo"].update(dt, r.app_d_v < 0.3, rep) | k["appd_hi"].update(dt, r.app_d_v > 4.8, rep)
        e_bad = k["appe_lo"].update(dt, r.app_e_v < 0.15, rep) | k["appe_hi"].update(dt, r.app_e_v > 4.8, rep)
        app_corr = k["app_corr"].update(dt, abs(r.app_d_v - 2.0 * r.app_e_v) > 0.3, rep,
                                        testable=not (d_bad or e_bad))
        pd = clamp(tf.v_to_app_d(r.app_d_v), 0.0, 1.0)
        pe = clamp(tf.v_to_app_e(r.app_e_v), 0.0, 1.0)
        if d_bad and e_bad:
            pedal, self.app_failed, self.app_scale = 0.0, True, 0.0
        elif d_bad or e_bad or app_corr:
            # Reduced power: trust the lower track, cap the demand
            good = pe if d_bad else pd if e_bad else min(pd, pe)
            pedal, self.app_failed, self.app_scale = min(good, 0.4), False, 0.4
        else:
            pedal, self.app_failed, self.app_scale = pd, False, 1.0
        v.app_d_pct = 15.0 + 72.0 * pedal  # ECU's arbitrated pedal, in D-track scaling

        # ---- oxygen sensors
        wb_circ = k["wb_circ"].update(dt, not r.wb_signal_ok, rep, testable=running)
        wb_heat = k["wb_heat"].update(dt, not r.wb_heater_ok, rep, testable=ecm.act.wb_heater)
        k["nb_heat"].update(dt, not r.nb_heater_ok, rep, testable=ecm.act.nb_heater)
        self.wb_failed = wb_circ or wb_heat

        # ---- knock, cam, crank
        self.knock_failed = k["knock"].update(dt, not r.knock_ok, rep, testable=running and r.rpm > 1500.0)
        self.cmp_failed = k["cmp"].update(dt, not r.cmp_ok, rep, testable=r.rpm > 300.0)
        # Cam pulses but no crank teeth: the crank sensor is dead (engine cannot sync or run)
        self.ckp_check.update(dt, r.rpm < 1.0, rep, testable=r.cam_active)

        # ---- EVAP pressure sensor, fan, VSS, fuel level
        t_lo = k["tank_lo"].update(dt, r.tank_v < 0.2, rep)
        t_hi = k["tank_hi"].update(dt, r.tank_v > 4.8, rep)
        # Vented tank must read close to atmospheric
        k["tank_rat"].update(dt, abs(r.tank_pa) > 500.0, rep,
                             testable=not (t_lo or t_hi) and not ecm.act.vent_closed and ecm.evap_request is None)
        k["fan"].update(dt, not r.fan_ok, rep, testable=ecm.act.fan > 0)
        k["vss"].update(dt, r.vss_kmh < 2.0, rep,
                        testable=running and r.rpm > 1800.0 and ecm.load_rel > 0.3 and not r.clutch_switch)
        k["level_circ"].update(dt, r.fuel_level_v < 0.2 or r.fuel_level_v > 4.8, rep)
        if running:
            self.fuel_used_g += ecm.fuel.injected_gps * dt
            if self.level_ref is None:
                self.level_ref = r.fuel_level_pct
            if self.fuel_used_g > 3000.0:
                k["level"].update(1.0, abs(r.fuel_level_pct - self.level_ref) < 1.0, rep)
                self.fuel_used_g = 0.0
                self.level_ref = r.fuel_level_pct

        # ---- injector and ignition driver stages
        for i in range(4):
            inj_bad = k[f"inj{i}"].update(dt, not r.injector_ok[i], rep, testable=running)
            self.coil_cut[i] = k[f"coil{i}"].update(dt, not r.coil_ok[i], rep, testable=running)
            self.dead_cyl[i] = inj_bad or self.coil_cut[i]
        return v
