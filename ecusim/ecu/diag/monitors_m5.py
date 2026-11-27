# SPDX-License-Identifier: GPL-3.0-or-later
"""Non-continuous monitors (DESIGN §6.2): catalyst, O2 sensors, O2 heaters, EVAP leak test, VVT.

Each runs at most once per drive cycle once its enable conditions are met, reports DTC results,
updates Mode 06 and explains in ``hint`` what it is still waiting for (the drive-cycle helper).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .mode06 import UAS_MS, UAS_MV, UAS_RAW
from .monitors import Monitor

if TYPE_CHECKING:
    from .diagnostics import Diagnostics


def _warm_cl(diag: "Diagnostics") -> bool:
    e = diag.ecm
    return e.mode == "run" and e.fuel.closed_loop and diag.readings.ect_c > 70.0


def _misfiring(diag: "Diagnostics") -> bool:
    """Catalyst and O2 tests are meaningless while cylinders misfire."""
    m = diag.monitor("misfire")
    return m.last_pct > 1.0 or m.cat_damage


# ====================================================================== catalyst
CAT_EVAL_S = 40.0
CAT_INDEX_LIMIT = 0.5


class CatalystMonitor(Monitor):
    key, name, bit = "catalyst", "Catalyst", 0

    def __init__(self) -> None:
        super().__init__()
        self.new_cycle()

    def declare(self, diag: "Diagnostics") -> None:
        diag.mode06.declare(0x21, [(0x80, UAS_RAW, 0, CAT_INDEX_LIMIT * 1000)])

    def new_cycle(self) -> None:
        super().new_cycle()
        self.t_ok = 0.0
        self.up = 0
        self.down = 0
        self._up_sign = 0
        self._nb_rich: bool | None = None
        self._prev_vss: float | None = None
        self.index = 0.0

    def step(self, diag: "Diagnostics", dt: float) -> None:
        if self.complete_this_cycle:
            self.hint = ""
            return
        e, r = diag.ecm, diag.readings
        accel = 0.0 if self._prev_vss is None else (r.vss_kmh - self._prev_vss) / 3.6 / dt
        self._prev_vss = r.vss_kmh
        ok = (
            _warm_cl(diag) and e.cat_est_c > 450.0 and e.nb_ready and 40.0 <= r.vss_kmh <= 110.0
            and 0.12 < e.load_rel < 0.75 and abs(accel) < 0.8 and e.evap_request is None and not _misfiring(diag)
            and abs(e.fuel.stft + e.fuel.ltft_active) < 0.2  # fuelling within authority
        )
        if not ok:
            self.hint = (f"needs a steady 40-110 km/h cruise, warm engine in closed loop "
                         f"({self.t_ok:.0f}/{CAT_EVAL_S:.0f} s)")
            return
        self.enabled_this_cycle = True
        self.t_ok += dt
        err = e.fuel.wb_filt - e.fuel.lambda_target_cl
        sign = 1 if err > 0 else -1
        if self._up_sign and sign != self._up_sign:
            self.up += 1
        self._up_sign = sign
        if self._nb_rich is None:
            self._nb_rich = r.nb_volts > 0.45
        elif self._nb_rich and r.nb_volts < 0.35:
            self._nb_rich = False
            self.down += 1
        elif not self._nb_rich and r.nb_volts > 0.6:
            self._nb_rich = True
            self.down += 1
        self.hint = f"evaluating oxygen storage ({self.t_ok:.0f}/{CAT_EVAL_S:.0f} s)"
        if self.t_ok >= CAT_EVAL_S:
            self.index = self.down / max(self.up, 1)
            diag.mode06.set(0x21, 0x80, self.index * 1000.0)
            diag.dtc.report("P0420", self.index > CAT_INDEX_LIMIT)
            self.completed()


# ====================================================================== O2 sensors
B1S1_RESP_LIMIT_MS = 900.0
B1S1_AIR_LAMBDA = 2.8  # 90 % of the saturated free-air reading
B1S2_RESP_LIMIT_MS = 1500.0


class O2SensorMonitor(Monitor):
    key, name, bit = "o2", "O2 sensor", 5

    def __init__(self) -> None:
        super().__init__()
        self.new_cycle()

    def declare(self, diag: "Diagnostics") -> None:
        diag.mode06.declare(0x01, [(0x80, UAS_MS, 0, B1S1_RESP_LIMIT_MS)])
        diag.mode06.declare(0x02, [(0x07, UAS_MV, 0, 200), (0x08, UAS_MV, 600, 1100),
                                   (0x80, UAS_MS, 0, B1S2_RESP_LIMIT_MS)])

    def new_cycle(self) -> None:
        super().new_cycle()
        self._was_cut = False
        self._t_cut = -1.0
        self.b1s1_ms: float | None = None
        self.b1s2_ms: float | None = None
        self._cl_time = 0.0
        self.nb_min = 9.9
        self.nb_max = -9.9
        self._stuck_t = {"P2195": 0.0, "P2196": 0.0}
        self._wb_hist: list[float] = []
        self._b1s2_done = False
        self._activity_done = False
        self._nb_no_lean = False  # a long fuel cut happened and the downstream sensor never went lean

    def step(self, diag: "Diagnostics", dt: float) -> None:
        e, r = diag.ecm, diag.readings
        f = e.fuel
        if e.mode != "run":
            self.hint = "needs the engine running"
            return
        warm = r.ect_c > 70.0
        # --- fuel-cut response tests (B1S1 lean response, B1S2 lean response)
        cut = f.dfco
        if cut and not self._was_cut and warm and e.wb_ready and e.nb_ready:
            self._t_cut = 0.0
        self._was_cut = cut
        if self._t_cut >= 0.0:
            if not cut:
                self._t_cut = -1.0
            else:
                self._t_cut += dt
                if self.b1s1_ms is None and r.wb_lambda > B1S1_AIR_LAMBDA:
                    self.b1s1_ms = self._t_cut * 1000.0
                    diag.mode06.set(0x01, 0x80, self.b1s1_ms)
                    diag.dtc.report("P0133", self.b1s1_ms > B1S1_RESP_LIMIT_MS)
                if self.b1s2_ms is None and r.nb_volts < 0.2:
                    self.b1s2_ms = self._t_cut * 1000.0
                    diag.mode06.set(0x02, 0x80, self.b1s2_ms)
                    diag.dtc.report("P0139", self.b1s2_ms > B1S2_RESP_LIMIT_MS)
                if self._t_cut > 3.0:
                    if self.b1s2_ms is None:
                        self._nb_no_lean = True
                    if self.b1s1_ms is None:
                        self.b1s1_ms = 3000.0
                        diag.mode06.set(0x01, 0x80, self.b1s1_ms)
                        diag.dtc.report("P0133", True)
                    self._t_cut = -1.0
        if e.nb_ready and warm:
            self.nb_min = min(self.nb_min, r.nb_volts)
            self.nb_max = max(self.nb_max, r.nb_volts)
        if not (_warm_cl(diag) or (warm and cut)) or _misfiring(diag):
            self.hint = self._hint()
            return
        self.enabled_this_cycle = True
        if f.closed_loop:
            self._cl_time += dt
            # --- B1S1 activity: forced modulation must show on the sensor
            self._wb_hist.append(r.wb_lambda)
            if len(self._wb_hist) >= 2000:
                n = len(self._wb_hist)
                mean = sum(self._wb_hist) / n
                std = (sum((x - mean) ** 2 for x in self._wb_hist) / n) ** 0.5
                diag.dtc.report("P0134", std < 0.002)
                self._wb_hist = []
                self._activity_done = True
            # --- stuck lean / rich: downstream disagrees while the trims are at their limit
            trims_ok = abs(f.stft + f.ltft_active) < 0.2 and self.nb_max - self.nb_min > 0.3
            for code, cond in (
                # upstream reads lean -> engine really rich -> downstream rich and its trim at the lean limit
                ("P2195", f.dtrim >= 0.0199 and r.nb_volts > 0.75),
                ("P2196", f.dtrim <= -0.0199 and r.nb_volts < 0.2),
            ):
                self._stuck_t[code] = self._stuck_t[code] + dt if (cond and trims_ok) else 0.0
                if self._stuck_t[code] > 30.0:
                    diag.dtc.report(code, True)
                    self._stuck_t[code] = 0.0
        # --- B1S2 range / activity once a fuel cut and enough closed loop have been seen
        if not self._b1s2_done and self.b1s2_ms is not None and self._cl_time > 60.0:
            diag.mode06.set(0x02, 0x07, self.nb_min * 1000.0)
            diag.mode06.set(0x02, 0x08, self.nb_max * 1000.0)
            low, high = self.nb_max < 0.3, self.nb_min > 0.8
            diag.dtc.report("P0137", low)
            diag.dtc.report("P0138", high)
            diag.dtc.report("P0140", self.nb_max - self.nb_min < 0.05 and not (low or high))
            for code in ("P2195", "P2196"):
                if self._stuck_t[code] == 0.0:
                    diag.dtc.report(code, False)
            self._b1s2_done = True
        elif not self._b1s2_done and self._nb_no_lean and self._cl_time > 60.0:
            # downstream never went lean during a long fuel cut
            diag.mode06.set(0x02, 0x07, self.nb_min * 1000.0)
            diag.mode06.set(0x02, 0x08, self.nb_max * 1000.0)
            if self.nb_min > 0.8:
                code = "P0138"
            elif self.nb_max < 0.3:
                code = "P0137"
            else:
                code = "P0140" if self.nb_max - self.nb_min < 0.05 else "P0139"
            diag.dtc.report(code, True)
            self._b1s2_done = True
        if self.b1s1_ms is not None and self._b1s2_done and not self.complete_this_cycle:
            self.completed()
        self.hint = self._hint()

    def _hint(self) -> str:
        if self.complete_this_cycle:
            return ""
        if self.b1s1_ms is None:
            return "needs a deceleration fuel cut with a warm engine (lift off above 1800 rpm, or coast in gear)"
        return f"needs more closed-loop running after the fuel cut ({min(self._cl_time, 60):.0f}/60 s)"


# ====================================================================== O2 heaters
WB_HEAT_LIMIT_S = 25.0
NB_HEAT_LIMIT_S = 90.0


class O2HeaterMonitor(Monitor):
    key, name, bit = "o2_heater", "O2 sensor heater", 6

    def __init__(self) -> None:
        super().__init__()
        self.new_cycle()

    def declare(self, diag: "Diagnostics") -> None:
        diag.mode06.declare(0x41, [(0x80, UAS_MS, 0, WB_HEAT_LIMIT_S * 1000)])
        diag.mode06.declare(0x42, [(0x80, UAS_MS, 0, NB_HEAT_LIMIT_S * 1000)])

    def new_cycle(self) -> None:
        super().new_cycle()
        self._wb_t: float | None = None
        self._nb_t: float | None = None
        self.wb_done = self.nb_done = False
        self._wb_skip = self._nb_skip = False

    def step(self, diag: "Diagnostics", dt: float) -> None:
        e, r = diag.ecm, diag.readings
        if e.mode != "run":
            return
        act = e.act
        if not self.wb_done and not self._wb_skip:
            if self._wb_t is None and act.wb_heater:
                if r.wb_element_c < 300.0:
                    self._wb_t = 0.0
                else:
                    self._wb_skip = True  # sensor still hot from the last drive: not testable
            elif self._wb_t is not None:
                self.enabled_this_cycle = True
                self._wb_t += dt
                if e.wb_ready or self._wb_t > WB_HEAT_LIMIT_S * 1.6:
                    diag.mode06.set(0x41, 0x80, self._wb_t * 1000.0)
                    diag.dtc.report("P0135", self._wb_t > WB_HEAT_LIMIT_S)
                    self.wb_done = True
        if not self.nb_done and not self._nb_skip:
            if self._nb_t is None and act.nb_heater:
                if r.nb_element_c < 250.0:
                    self._nb_t = 0.0
                else:
                    self._nb_skip = True
            elif self._nb_t is not None:
                self.enabled_this_cycle = True
                self._nb_t += dt
                if e.nb_ready or self._nb_t > NB_HEAT_LIMIT_S * 1.4:
                    diag.mode06.set(0x42, 0x80, self._nb_t * 1000.0)
                    diag.dtc.report("P0141", self._nb_t > NB_HEAT_LIMIT_S)
                    self.nb_done = True
        if self.wb_done and self.nb_done and not self.complete_this_cycle:
            self.completed()
        if self.complete_this_cycle:
            self.hint = ""
        elif self._wb_skip or self._nb_skip:
            self.hint = "needs a cold start (the sensors were still hot when their heaters switched on)"
        else:
            self.hint = "timing the sensor heaters after a cold start"


# ====================================================================== VVT
VVT_ERR_LIMIT = 8.0
PARK_OFFSET_LIMIT = 6.0


class VvtMonitor(Monitor):
    key, name, bit = "egr_vvt", "EGR/VVT system", 7

    def __init__(self) -> None:
        super().__init__()
        self.new_cycle()

    def declare(self, diag: "Diagnostics") -> None:
        diag.mode06.declare(0x35, [(0x80, UAS_RAW, 0, VVT_ERR_LIMIT * 10), (0x81, UAS_RAW, 0, PARK_OFFSET_LIMIT * 10)])

    def new_cycle(self) -> None:
        super().new_cycle()
        self._park_sum = 0.0
        self._park_n = 0
        self._err_sum = 0.0
        self._err_n = 0
        self._bad_t = 0.0
        self.park_offset: float | None = None

    def step(self, diag: "Diagnostics", dt: float) -> None:
        e, r = diag.ecm, diag.readings
        if e.mode != "run" or r.oil_c < 60.0 or self.complete_this_cycle:
            self.hint = "" if self.complete_this_cycle else "needs warm oil (above 60 °C)"
            return
        target = e.act.vvt_target_deg
        err = r.cam_deg - target
        if target == 0.0 and e.air.idle_active and r.rpm < 1100.0:
            self._park_sum += r.cam_deg
            self._park_n += 1
            if self._park_n >= 500:
                self.park_offset = self._park_sum / self._park_n
                diag.mode06.set(0x35, 0x81, abs(self.park_offset) * 10.0)
                self._park_sum, self._park_n = 0.0, 0
        elif target > 5.0:
            self.enabled_this_cycle = True
            self._err_sum += err
            self._err_n += 1
            self._bad_t = self._bad_t + dt if abs(err) > VVT_ERR_LIMIT else 0.0
        if not self._err_n:
            self.hint = "needs part-load driving with the cam phaser active (above 1000 rpm, off idle)"
            return
        n = self._err_n
        mean_err = self._err_sum / n
        constant_offset = (self.park_offset is not None and abs(self.park_offset) > PARK_OFFSET_LIMIT
                           and abs(mean_err - self.park_offset) < 3.0)
        if self._bad_t > 5.0 and not constant_offset:
            diag.dtc.report("P0011" if mean_err > 0 else "P0012", True)
            self._bad_t = 0.0
        if n * dt >= 30.0:
            diag.mode06.set(0x35, 0x80, abs(mean_err) * 10.0)
            if self.park_offset is not None:
                diag.dtc.report("P0016", constant_offset)
                if constant_offset:
                    pass
                elif abs(self.park_offset) > VVT_ERR_LIMIT:
                    # cam does not return to its park position: phaser stuck
                    diag.dtc.report("P0011" if self.park_offset > 0 else "P0012", True)
                elif abs(mean_err) > VVT_ERR_LIMIT:
                    diag.dtc.report("P0011" if mean_err > 0 else "P0012", True)
                else:
                    diag.dtc.report("P0011", False)
                    diag.dtc.report("P0012", False)
                self.completed()
        if self.complete_this_cycle:
            self.hint = ""
        elif n * dt < 30.0:
            self.hint = f"checking cam phasing ({n * dt:.0f}/30 s)"
        else:
            # the rest position is only measured at warm idle; without it P0016 cannot be told from P0011/P0012
            self.hint = (f"cam phasing checked; now idle for {self._park_n * dt:.0f}/5 s "
                         "to check the cam's rest position")


# ====================================================================== EVAP leak test
EVAP_MIN_RUN_S = 300.0
EVAP_TARGET_PA = -2000.0
EVAP_PULL_TIMEOUT_S = 30.0
EVAP_HOLD_S = 20.0
LEAK_1MM_PA_S = 50.0
LEAK_05MM_PA_S = 15.0


class EvapMonitor(Monitor):
    key, name, bit = "evap", "EVAP system", 2

    def __init__(self) -> None:
        super().__init__()
        self.new_cycle()

    def declare(self, diag: "Diagnostics") -> None:
        diag.mode06.declare(0x39, [(0x80, UAS_RAW, 1500, 6000)])  # vacuum reached (Pa)
        diag.mode06.declare(0x3B, [(0x80, UAS_RAW, 0, LEAK_1MM_PA_S * 10)])  # decay 0.1 Pa/s, 0.040"
        diag.mode06.declare(0x3C, [(0x80, UAS_RAW, 0, LEAK_05MM_PA_S * 10)])  # decay, 0.020"
        diag.mode06.declare(0x3D, [(0x80, UAS_RAW, 100, 2000)])  # purge flow: idle MAF drop (mg/s)

    def new_cycle(self) -> None:
        super().new_cycle()
        self.phase = "wait"
        self._t = 0.0
        self._p_start = 0.0
        self._p_min = 0.0
        self.decay: float | None = None
        self.sealed: bool | None = None
        self._cold_start: bool | None = None
        # purge flow check
        self._pf_state = "armed"
        self._pf_before: list[float] = []
        self._pf_after: list[float] = []
        self.purge_flow_ok: bool | None = None

    def _abort(self, diag: "Diagnostics") -> None:
        diag.ecm.evap_request = None
        self.phase = "wait"

    def _purge_flow_check(self, diag: "Diagnostics") -> None:
        e, r = diag.ecm, diag.readings
        idle = e.air.idle_active and r.vss_kmh < 1.0
        if self._pf_state == "armed":
            if idle and e.act.purge_duty == 0.0 and _warm_cl(diag):
                self._pf_before.append(r.maf_gps)
                self._pf_before = self._pf_before[-200:]
            elif idle and e.act.purge_duty > 0.0 and len(self._pf_before) >= 200:
                self._pf_state = "measuring"
            elif not idle:
                self._pf_before = []
        elif self._pf_state == "measuring":
            if not idle:
                self._pf_state, self._pf_before, self._pf_after = "armed", [], []
            elif e.act.purge_duty >= 0.3:
                self._pf_after.append(r.maf_gps)
                if len(self._pf_after) >= 300:
                    drop = sum(self._pf_before) / 200 - sum(self._pf_after[-150:]) / 150
                    diag.mode06.set(0x3D, 0x80, max(0.0, drop * 1000.0))
                    self.purge_flow_ok = drop > 0.10
                    self._pf_state = "done"

    def step(self, diag: "Diagnostics", dt: float) -> None:
        e, r = diag.ecm, diag.readings
        if e.mode != "run":
            self.hint = "needs the engine running"
            return
        if self._cold_start is None:
            self._cold_start = abs(e.ect_start - e.iat_start) < 7.0 and 4.0 <= e.ambient_est_c <= 35.0
        self._purge_flow_check(diag)
        if self.complete_this_cycle:
            self.hint = ""
            return
        if not self._cold_start:
            self.hint = "needs a cold start (coolant within 7 °C of intake air, ambient 4-35 °C)"
            return
        steady = (e.fuel.closed_loop and r.ect_c > 70.0 and not e.fuel.dfco and not e.fuel.pe
                  and (e.air.idle_active and r.vss_kmh < 1.0 or 40.0 < r.vss_kmh < 110.0)
                  and 15.0 <= r.fuel_level_pct <= 85.0 and 0.2 < r.tank_v < 4.8)
        if self.phase == "wait":
            if e.t_run < EVAP_MIN_RUN_S:
                self.hint = f"waiting for 5 minutes of running ({e.t_run:.0f}/{EVAP_MIN_RUN_S:.0f} s)"
                return
            if not steady:
                self.hint = "needs steady idle or 40-110 km/h cruise, fuel 15-85 %, closed loop"
                return
            self.enabled_this_cycle = True
            self.phase, self._t, self._p_min = "seal", 0.0, 0.0
            self._p_start = None
        if not steady:
            self._abort(diag)
            self.hint = "test interrupted, will retry when driving is steady"
            return
        self._t += dt
        p = r.tank_pa
        if self.phase == "seal":
            # Vent and purge closed: fuel vapour should build a little pressure if the system holds
            e.evap_request = (True, 0.0)
            if self._p_start is None:
                self._p_start = p
            self.hint = f"leak test: checking the vent seals ({p:.0f} Pa)"
            if p < -1000.0:
                # Vacuum builds with the purge valve commanded shut: it is leaking through
                diag.dtc.report("P0496", True)
                self._finish(diag)
                return
            if self._t >= 15.0:
                self.sealed = p - self._p_start > 12.0
                self.phase, self._t = "pull", 0.0
            return
        if self.phase == "pull":
            e.evap_request = (True, 0.3)
            self._p_min = min(self._p_min, p)
            self.hint = f"leak test: pulling vacuum ({p:.0f} Pa)"
            if p <= EVAP_TARGET_PA:
                self.phase, self._t, self._p_start = "hold", 0.0, p
            elif self._t > EVAP_PULL_TIMEOUT_S:
                diag.mode06.set(0x39, 0x80, max(0.0, -self._p_min))
                if self._p_min < -100.0:
                    code = "P0455"  # some vacuum but cannot hold it: large leak
                elif self.sealed:
                    code = "P0441"  # system seals but purge pulls nothing: purge flow fault
                else:
                    code = "P0446"  # cannot seal at all: vent valve
                diag.dtc.report(code, True)
                self._finish(diag)
        elif self.phase == "hold":
            e.evap_request = (True, 0.0)
            self.hint = f"leak test: holding vacuum ({p:.0f} Pa, {self._t:.0f}/{EVAP_HOLD_S:.0f} s)"
            if self._t >= EVAP_HOLD_S:
                self.decay = (p - self._p_start) / self._t
                diag.mode06.set(0x39, 0x80, max(0.0, -self._p_min if self._p_min < 0 else -self._p_start))
                diag.mode06.set(0x3B, 0x80, max(0.0, self.decay) * 10.0)
                diag.mode06.set(0x3C, 0x80, max(0.0, self.decay) * 10.0)
                purge_leaking = p - self._p_start < -500.0
                diag.dtc.report("P0496", purge_leaking)
                diag.dtc.report("P0455", False)
                diag.dtc.report("P0446", False)
                if self.purge_flow_ok:
                    diag.dtc.report("P0441", False)
                diag.dtc.report("P0442", self.decay > LEAK_1MM_PA_S)
                if diag.ecm.region == "us":
                    diag.dtc.report("P0456", LEAK_05MM_PA_S < self.decay <= LEAK_1MM_PA_S)
                self._finish(diag)

    def _finish(self, diag: "Diagnostics") -> None:
        diag.ecm.evap_request = None
        self.phase = "done"
        self.completed()
