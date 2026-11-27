"""In-use monitor performance ratio counters (Mode 09 PID 08, spark ignition)."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .diagnostics import Diagnostics

# Mode 09 PID 08 order (20 counters): OBDCOND, IGNCNTR, then numerator/denominator pairs
GROUPS = ["catalyst_b1", "catalyst_b2", "o2_b1", "o2_b2", "egr_vvt", "secondary_air", "evap", "o2_sec_b1",
          "o2_sec_b2"]
GROUP_MONITOR = {"catalyst_b1": "catalyst", "o2_b1": "o2", "egr_vvt": "egr_vvt", "evap": "evap", "o2_sec_b1": "o2"}


class Iumpr:
    def __init__(self) -> None:
        self.obd_cond = 0
        self.ign_cntr = 0
        self.num = {g: 0 for g in GROUPS}
        self.den = {g: 0 for g in GROUPS}
        self._reset_cycle()

    def _reset_cycle(self) -> None:
        self._run_s = 0.0
        self._fast_s = 0.0
        self._idle_s = 0.0
        self._idle_ok = False
        self._den_done = False
        self._num_done: set[str] = set()

    def engine_started(self) -> None:
        self.ign_cntr = min(65535, self.ign_cntr + 1)

    def new_cycle(self) -> None:
        self._reset_cycle()

    def step(self, diag: "Diagnostics", dt: float) -> None:
        e, r = diag.ecm, diag.readings
        if e.mode != "run":
            return
        self._run_s += dt
        self._fast_s += dt if r.vss_kmh >= 40.0 else 0.0
        if r.vss_kmh < 1.6 and e.pedal_pct < 1.0:
            self._idle_s += dt
            self._idle_ok |= self._idle_s >= 30.0
        else:
            self._idle_s = 0.0
        if (not self._den_done and self._run_s >= 600.0 and self._fast_s >= 300.0 and self._idle_ok
                and e.ambient_est_c >= -7.0 and e.baro_kpa >= 75.0):
            self._den_done = True
            self.obd_cond = min(65535, self.obd_cond + 1)
            for g in GROUP_MONITOR:
                self.den[g] = min(65535, self.den[g] + 1)
        for g, key in GROUP_MONITOR.items():
            m = diag.monitor(key)
            if g not in self._num_done and m.complete_this_cycle:
                self._num_done.add(g)
                self.num[g] = min(65535, self.num[g] + 1)

    def values(self) -> list[int]:
        out = [self.obd_cond, self.ign_cntr]
        for g in GROUPS:
            out += [self.num[g], self.den[g]]
        return out

    def export(self) -> dict:
        return {"obd_cond": self.obd_cond, "ign_cntr": self.ign_cntr, "num": self.num, "den": self.den}

    def load(self, d: dict) -> None:
        self.obd_cond = int(d.get("obd_cond", 0))
        self.ign_cntr = int(d.get("ign_cntr", 0))
        self.num.update({k: int(v) for k, v in d.get("num", {}).items() if k in self.num})
        self.den.update({k: int(v) for k, v in d.get("den", {}).items() if k in self.den})
