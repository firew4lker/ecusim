#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run every fault (and every sensor failure mode) through a standard drive cycle in parallel and
print which DTCs the ECU set, next to what the catalogue expects. Takes ~2.5 minutes on 8 cores.

    python3 tools/fault_sweep.py
"""

from __future__ import annotations

import sys
import time
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ecusim.driving import standard_drive_cycle  # noqa: E402
from ecusim.faults.catalog import BY_KEY, FAULTS, SENSOR  # noqa: E402
from ecusim.faults.manager import FaultSetting  # noqa: E402
from ecusim.sim import Simulator  # noqa: E402

COLD = {"wb_heater", "nb_heater", "thermostat_open", "evap_leak", "gas_cap", "vent", "purge_closed", "purge_open",
        "s_ect"}


def run(key: str, mode: str):
    cold = key in COLD
    sim = Simulator(ambient_c=20.0, seed=3)
    if not cold:
        sim.plant.set_warm()
        sim.sensors = type(sim.sensors)(sim.rng)
    sim.faults.set(FaultSetting(key, mode=mode, cylinder=3))
    sim.run(0.1)
    started = sim.start_engine(max_crank_s=8.0) > 0
    if started:
        standard_drive_cycle(sim, cruise_s=400.0 if cold else 150.0)
    d = sim.ecu.diag
    return key, mode, started, sorted(set(d.dtc.pending) | set(d.dtc.confirmed)), sim.ecu.limp


def main() -> int:
    jobs = [(d.key, m) for d in FAULTS for m in (d.modes if d.kind == SENSOR else ("",))]
    t0 = time.time()
    with Pool(8) as pool:
        for key, mode, started, dtcs, limp in pool.starmap(run, jobs):
            extra = ("" if started else " NO START") + (" LIMP" if limp else "")
            print(f"{key:18s} {mode:10s} {' '.join(dtcs) or '-':40s}{extra:10s} expected: {BY_KEY[key].expected}")
    print(f"{len(jobs)} runs in {time.time() - t0:.0f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
