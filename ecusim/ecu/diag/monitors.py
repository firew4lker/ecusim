"""Readiness monitors (DESIGN §6.2). M4: misfire, fuel system, comprehensive components.
Catalyst, EVAP, O2 sensor, O2 heater and EGR/VVT monitors are declared (supported) and arrive in M5."""

from __future__ import annotations

from typing import TYPE_CHECKING

from ...core.tables import interp1
from .mode06 import UAS_COUNTS, UAS_PCT_SIGNED

if TYPE_CHECKING:
    from .diagnostics import Diagnostics

FORCE_AUTO = "auto"
FORCE_COMPLETE = "complete"
FORCE_INCOMPLETE = "incomplete"


class Monitor:
    key = ""
    name = ""
    continuous = False
    bit = 0  # bit position in PID 01 byte B (continuous) or C/D (non-continuous)
    supported = True

    def __init__(self) -> None:
        self.force = FORCE_AUTO
        self.complete_since_clear = False
        self.complete_this_cycle = False
        self.enabled_this_cycle = False
        self.hint = ""

    # readiness as reported
    @property
    def ready(self) -> bool:
        if self.force == FORCE_COMPLETE:
            return True
        if self.force == FORCE_INCOMPLETE:
            return False
        return self.complete_since_clear

    @property
    def ready_this_cycle(self) -> bool:
        if self.force == FORCE_COMPLETE:
            return True
        if self.force == FORCE_INCOMPLETE:
            return False
        return self.complete_this_cycle

    @property
    def enabled_cycle(self) -> bool:
        return self.enabled_this_cycle or self.force == FORCE_COMPLETE

    def completed(self) -> None:
        self.complete_since_clear = True
        self.complete_this_cycle = True

    def new_cycle(self) -> None:
        self.complete_this_cycle = False
        self.enabled_this_cycle = False

    def end_cycle(self, diag: "Diagnostics") -> None:
        pass

    def clear(self) -> None:
        self.complete_since_clear = False
        self.complete_this_cycle = False

    def step(self, diag: "Diagnostics", dt: float) -> None:
        pass

    def export(self) -> dict:
        return {"complete": self.complete_since_clear, "force": self.force}

    def load(self, d: dict) -> None:
        self.complete_since_clear = bool(d.get("complete", False))
        self.force = d.get("force", FORCE_AUTO)


# ====================================================================== misfire
MISFIRE_CODES = ["P0301", "P0302", "P0303", "P0304"]
EMISSION_THRESHOLD_PCT = 2.0


def cat_damage_threshold_pct(load_rel: float) -> float:
    return interp1([0.2, 0.8], [15.0, 6.0], load_rel)


class MisfireMonitor(Monitor):
    key = "misfire"
    name = "Misfire"
    continuous = True
    bit = 0

    def __init__(self) -> None:
        super().__init__()
        self._last_counts: list[int] | None = None
        self._last_revs = 0.0
        self._w200 = [0.0, [0, 0, 0, 0]]
        self._w1000 = [0.0, [0, 0, 0, 0]]
        self.cycle_counts = [0, 0, 0, 0]
        self.ewma = [0.0, 0.0, 0.0, 0.0]
        self.cat_damage = False
        self.last_pct = 0.0

    def declare(self, diag: "Diagnostics") -> None:
        for mid in range(0xA1, 0xA6):
            diag.mode06.declare(mid, [(0x0B, UAS_COUNTS, 0, 0), (0x0C, UAS_COUNTS, 0, 0)])

    @staticmethod
    def _codes(counts: list[int]) -> list[str]:
        total = sum(counts)
        cyl = [i for i, c in enumerate(counts) if c > 0 and c >= 0.2 * total]
        codes = [MISFIRE_CODES[i] for i in cyl]
        if len(cyl) != 1:
            codes.append("P0300")
        return codes

    def _update_mode06(self, diag: "Diagnostics") -> None:
        m = diag.mode06
        for i in range(4):
            m.set(0xA2 + i, 0x0B, self.ewma[i])
            m.set(0xA2 + i, 0x0C, self.cycle_counts[i])
        m.set(0xA1, 0x0B, sum(self.ewma))
        m.set(0xA1, 0x0C, sum(self.cycle_counts))

    def step(self, diag: "Diagnostics", dt: float) -> None:
        r, e = diag.readings, diag.ecm
        counts = list(r.misfire_counts)
        if self._last_counts is None:
            self._last_counts, self._last_revs = counts, r.crank_revs
            return
        d_counts = [a - b for a, b in zip(counts, self._last_counts)]
        d_revs = r.crank_revs - self._last_revs
        self._last_counts, self._last_revs = counts, r.crank_revs

        enabled = (
            e.mode == "run"
            and 450.0 < r.rpm < 6500.0
            and r.ect_c > -7.0
            and not e.fuel.dfco
            and not e.fuel.rev_cut
            and e.t_run > 2.0
        )
        if not enabled:
            self.hint = "needs the engine running 450-6500 rpm, not in fuel cut"
            self.cat_damage = False
            return
        self.enabled_this_cycle = True
        self.hint = "" if self.complete_since_clear else "evaluating 1000-revolution window"
        for i in range(4):
            self._w200[1][i] += d_counts[i]
            self._w1000[1][i] += d_counts[i]
            self.cycle_counts[i] += d_counts[i]
        self._w200[0] += d_revs
        self._w1000[0] += d_revs

        if self._w200[0] >= 200.0:
            revs, c = self._w200
            pct = sum(c) / (revs * 2.0) * 100.0
            self.cat_damage = pct > cat_damage_threshold_pct(e.load_rel)
            if self.cat_damage:
                for code in self._codes(c):
                    diag.dtc.report(code, True)
            self._w200 = [0.0, [0, 0, 0, 0]]

        if self._w1000[0] >= 1000.0:
            revs, c = self._w1000
            pct = sum(c) / (revs * 2.0) * 100.0
            self.last_pct = pct
            if pct > EMISSION_THRESHOLD_PCT:
                for code in self._codes(c):
                    diag.dtc.report(code, True)
            else:
                for code in MISFIRE_CODES + ["P0300"]:
                    diag.dtc.report(code, False)
            self.completed()
            self._update_mode06(diag)
            self._w1000 = [0.0, [0, 0, 0, 0]]

    def end_cycle(self, diag: "Diagnostics") -> None:
        if self.enabled_this_cycle:
            self.ewma = [0.1 * c + 0.9 * w for c, w in zip(self.cycle_counts, self.ewma)]
        self.cycle_counts = [0, 0, 0, 0]
        self.cat_damage = False
        self._w200 = [0.0, [0, 0, 0, 0]]
        self._w1000 = [0.0, [0, 0, 0, 0]]
        self._last_counts = None

    def clear(self) -> None:
        super().clear()
        self.ewma = [0.0, 0.0, 0.0, 0.0]
        self.cycle_counts = [0, 0, 0, 0]

    def export(self) -> dict:
        return {**super().export(), "ewma": self.ewma}

    def load(self, d: dict) -> None:
        super().load(d)
        self.ewma = [float(x) for x in d.get("ewma", [0, 0, 0, 0])]


# ====================================================================== fuel system
TRIM_FAIL = 0.24
TRIM_OK = 0.20


class FuelSystemMonitor(Monitor):
    key = "fuel"
    name = "Fuel system"
    continuous = True
    bit = 1

    def __init__(self) -> None:
        super().__init__()
        self._lean = self._rich = self._ok = 0.0
        self._inhibit = 0.0

    def declare(self, diag: "Diagnostics") -> None:
        diag.mode06.declare(0x81, [(0x80, UAS_PCT_SIGNED, -TRIM_FAIL * 10000, TRIM_FAIL * 10000)])

    def step(self, diag: "Diagnostics", dt: float) -> None:
        e = diag.ecm
        f = e.fuel
        if not (e.mode == "run" and f.closed_loop and diag.readings.ect_c > 70.0):
            self.hint = "needs closed loop with a warm engine"
            return
        self.enabled_this_cycle = True
        total = f.stft + f.ltft_active
        # Purge vapour learning can hide a fuelling fault: if it is carrying a big correction, switch
        # purge off for a while and judge the trims on their own (intrusive check)
        base = e.cyl_flow_gps / 14.7
        purge_share = e.purge.fuel_est_gps / base if base > 0.05 else 0.0
        if self._inhibit > 0.0:
            self._inhibit -= dt
            e.purge_inhibit = self._inhibit > 0.0
        elif purge_share > 0.12:
            self._inhibit = 60.0
            e.purge_inhibit = True
        diag.mode06.set(0x81, 0x80, total * 10000.0)
        self._lean = self._lean + dt if total >= TRIM_FAIL else 0.0
        self._rich = self._rich + dt if total <= -TRIM_FAIL else 0.0
        self._ok = self._ok + dt if abs(total) < TRIM_OK else 0.0
        if self._lean > 30.0:
            diag.dtc.report("P0171", True)
            self._lean = 0.0
            self.completed()
        if self._rich > 30.0:
            diag.dtc.report("P0172", True)
            self._rich = 0.0
            self.completed()
        if self._ok > 60.0:
            diag.dtc.report("P0171", False)
            diag.dtc.report("P0172", False)
            self._ok = 0.0
            self.completed()
        self.hint = "" if self.complete_this_cycle else f"closed loop {self._ok:.0f}/60 s"

    def new_cycle(self) -> None:
        super().new_cycle()
        self._lean = self._rich = self._ok = 0.0
        self._inhibit = 0.0


# ====================================================================== comprehensive components
class ComponentsMonitor(Monitor):
    key = "components"
    name = "Comprehensive components"
    continuous = True
    bit = 2

    def __init__(self) -> None:
        super().__init__()
        self._v_low = self._v_high = self._v_ok = 0.0
        self._idle_err: list[float] = []
        self._idle_t = 0.0
        self._t_model = None  # modelled coolant temperature for the thermostat test
        self._thermo_done = False

    def new_cycle(self) -> None:
        super().new_cycle()
        self._v_low = self._v_high = self._v_ok = 0.0
        self._idle_err = []
        self._idle_t = 0.0
        self._t_model = None
        self._thermo_done = False

    def _voltage(self, diag: "Diagnostics", dt: float) -> None:
        v = diag.readings.vbatt
        self._v_low = self._v_low + dt if v < 11.8 else 0.0
        self._v_high = self._v_high + dt if v > 15.5 else 0.0
        self._v_ok = self._v_ok + dt if 11.8 <= v <= 15.5 else 0.0
        if self._v_low > 30.0:
            diag.dtc.report("P0562", True)
            self._v_low = 0.0
        if self._v_high > 10.0:
            diag.dtc.report("P0563", True)
            self._v_high = 0.0
        if self._v_ok > 60.0:
            diag.dtc.report("P0562", False)
            diag.dtc.report("P0563", False)
            self._v_ok = 0.0
            self.completed()

    def _idle(self, diag: "Diagnostics", dt: float) -> None:
        e, r = diag.ecm, diag.readings
        if not (e.air.idle_active and r.vss_kmh < 1.0 and r.ect_c > 70.0 and e.t_run > 60.0 and e.cat_heat == 0.0
                and not e.limp and not e.inputs.ect_failed and not e.inputs.checks["vss"].active):
            self._idle_t = 0.0
            self._idle_err = []
            return
        self._idle_t += dt
        self._idle_err.append(e.air.target_rpm - r.rpm)
        if self._idle_t >= 10.0:
            err = sum(self._idle_err) / len(self._idle_err)
            if err > 100.0:
                diag.dtc.report("P0506", True)
            elif err < -200.0:
                diag.dtc.report("P0507", True)
            elif abs(err) < 50.0:
                diag.dtc.report("P0506", False)
                diag.dtc.report("P0507", False)
            self._idle_t = 0.0
            self._idle_err = []

    def _thermostat(self, diag: "Diagnostics", dt: float) -> None:
        """P0128: modelled coolant would be warm but the sensor says it isn't."""
        e, r = diag.ecm, diag.readings
        if self._thermo_done:
            return
        if self._t_model is None:
            if e.ect_start > 50.0:
                self._thermo_done = True  # warm start: test not applicable
                return
            self._t_model = r.ect_c
        q = e.fuel.injected_gps * 43.0e3 * 0.27
        self._t_model += dt * (q - 15.0 * (self._t_model - e.ambient_est_c)) / 24000.0
        if r.ect_c >= 75.0:
            diag.dtc.report("P0128", False)
            self._thermo_done = True
        elif self._t_model >= 90.0 and r.ect_c < 70.0:
            diag.dtc.report("P0128", True)
            self._thermo_done = True

    def step(self, diag: "Diagnostics", dt: float) -> None:
        if diag.ecm.mode != "run":
            return
        self.enabled_this_cycle = True
        self._voltage(diag, dt)
        self._idle(diag, dt)
        self._thermostat(diag, dt)
        self.hint = "" if self.complete_this_cycle else "running rationality checks"


# ====================================================================== non-continuous (M5)
class UnsupportedMonitor(Monitor):
    supported = False


def _mk(base, key: str, name: str, bit: int) -> Monitor:
    cls = type(name.replace(" ", "") + "Monitor", (base,), {"key": key, "name": name, "bit": bit})
    return cls()


def build_monitors() -> list[Monitor]:
    from .monitors_m5 import CatalystMonitor, EvapMonitor, O2HeaterMonitor, O2SensorMonitor, VvtMonitor

    return [
        MisfireMonitor(),
        FuelSystemMonitor(),
        ComponentsMonitor(),
        CatalystMonitor(),
        _mk(UnsupportedMonitor, "heated_catalyst", "Heated catalyst", 1),
        EvapMonitor(),
        _mk(UnsupportedMonitor, "secondary_air", "Secondary air", 3),
        _mk(UnsupportedMonitor, "ac", "A/C refrigerant", 4),
        O2SensorMonitor(),
        O2HeaterMonitor(),
        VvtMonitor(),
    ]
