# SPDX-License-Identifier: GPL-3.0-or-later
"""Non-continuous monitors and fault -> DTC detection over realistic drive cycles (M5).

The drive cycles are independent, so they run in a process pool to keep the suite fast.
"""

from concurrent.futures import ProcessPoolExecutor

import pytest

from ecusim.driving import standard_drive_cycle
from ecusim.faults.manager import FaultSetting
from ecusim.sim import Simulator

# name: (fault setting dict or None, cold start, region, codes that must appear, codes that must not)
SCENARIOS = {
    "healthy_cold": (None, True, "eu", set(), None),
    "catalyst": ({"key": "catalyst"}, False, "eu", {"P0420"}, set()),
    "evap_1mm": ({"key": "evap_leak", "value": 1.0}, True, "eu", {"P0442"}, {"P0455"}),
    "evap_05mm_us": ({"key": "evap_leak", "value": 0.5}, True, "us", {"P0456"}, {"P0442"}),
    "evap_05mm_eu": ({"key": "evap_leak", "value": 0.5}, True, "eu", set(), {"P0442", "P0456"}),
    "gas_cap": ({"key": "gas_cap"}, True, "eu", {"P0455"}, set()),
    "vent": ({"key": "vent"}, True, "eu", {"P0446"}, {"P0441"}),
    "purge_closed": ({"key": "purge_closed"}, True, "eu", {"P0441"}, {"P0446"}),
    "purge_open": ({"key": "purge_open"}, True, "eu", {"P0496"}, set()),
    "wb_slow": ({"key": "s_wb", "mode": "slow", "amount": 1.5}, False, "eu", {"P0133"}, set()),
    "wb_heater": ({"key": "wb_heater"}, True, "eu", {"P0030", "P0135"}, set()),
    "nb_heater": ({"key": "nb_heater"}, True, "eu", {"P0036"}, set()),
    "nb_slow": ({"key": "s_nb", "mode": "slow", "amount": 2.0}, False, "eu", {"P0139"}, set()),
    "vvt_stuck": ({"key": "vvt_stuck"}, False, "eu", {"P0011"}, {"P0016"}),
    "chain": ({"key": "chain"}, False, "eu", {"P0016"}, {"P0011", "P0012"}),
    "exhaust_leak": ({"key": "exhaust_leak"}, False, "eu", {"P2195"}, set()),
    "injector_leak": ({"key": "injector_leak", "cylinder": 2}, False, "eu", {"P0172"}, set()),
    "vacuum_leak": ({"key": "vacuum_leak"}, False, "eu", {"P0171"}, {"P0106", "P0101"}),
    "dirty_maf": ({"key": "dirty_maf"}, False, "eu", {"P0101", "P0171"}, {"P0106"}),
    "map_stuck": ({"key": "s_map", "mode": "stuck"}, False, "eu", {"P0106"}, {"P0101"}),
    "thermostat": ({"key": "thermostat_open"}, True, "eu", {"P0128"}, set()),
    "plug": ({"key": "plug", "cylinder": 3, "value": 85}, False, "eu", {"P0303"}, set()),
}
CRUISE = {"healthy_cold": 420.0}


def run(name: str):
    setting, cold, region, _must, _mustnt = SCENARIOS[name]
    sim = Simulator(ambient_c=20.0, seed=4)
    sim.ecu.region = region
    if not cold:
        sim.plant.set_warm()
        sim.sensors = type(sim.sensors)(sim.rng)
    if setting:
        sim.faults.set(FaultSetting.from_dict(setting))
    sim.start_engine(max_crank_s=8.0)
    standard_drive_cycle(sim, cruise_s=CRUISE.get(name, 420.0 if cold else 150.0))
    d = sim.ecu.diag
    codes = set(d.dtc.pending) | set(d.dtc.confirmed)
    monitors = {m.key: m.complete_this_cycle for m in d.monitors if m.supported}
    iumpr = d.iumpr.export()
    return name, codes, monitors, iumpr


@pytest.fixture(scope="module")
def results():
    with ProcessPoolExecutor(max_workers=8) as pool:
        return {name: (codes, mons, iumpr) for name, codes, mons, iumpr in pool.map(run, SCENARIOS)}


def test_healthy_cold_cycle_completes_every_monitor(results):
    codes, monitors, iumpr = results["healthy_cold"]
    assert codes == set()
    assert all(monitors.values()), monitors
    assert iumpr["num"]["catalyst_b1"] == 1 and iumpr["num"]["evap"] == 1 and iumpr["num"]["o2_b1"] == 1


@pytest.mark.parametrize("name", [n for n in SCENARIOS if n != "healthy_cold"])
def test_fault_sets_expected_codes(results, name):
    codes, _monitors, _ = results[name]
    _setting, _cold, _region, must, mustnt = SCENARIOS[name]
    assert must <= codes, f"{name}: expected {sorted(must)}, got {sorted(codes)}"
    assert not (mustnt & codes), f"{name}: unexpected {sorted(mustnt & codes)}"
