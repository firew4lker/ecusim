# SPDX-License-Identifier: GPL-3.0-or-later
"""Vehicle and driveline: drive away, shift, cruise, stall, interlocks (DESIGN §4.9, M3)."""

import statistics

import pytest

from ecusim.driving import ScriptedDriver
from ecusim.ecu.fuel import FS_CL, FS_OL_DRIVE
from ecusim.sim import Simulator

MODE_RUN = 3


def warm_sim(seed=3) -> Simulator:
    sim = Simulator(ambient_c=20.0, seed=seed)
    sim.plant.set_warm()
    sim.sensors = type(sim.sensors)(sim.rng)
    assert sim.start_engine() > 0
    sim.run(20.0)
    return sim


@pytest.fixture(scope="module")
def trip():
    sim = warm_sim()
    d = ScriptedDriver(sim)
    marks = {}
    d.launch(1, 0.2, 2.0)
    d.accelerate_until(0.5, rpm=3000)
    for g in (2, 3, 4, 5):
        d.shift(g, pedal_after=0.4)
        d.accelerate_until(0.6, rpm=3000 if g < 5 else None, kmh=100 if g == 5 else None, timeout=40)
    marks["cruise"] = sim.t
    d.hold_speed(100.0, 30.0)
    marks["lift"] = sim.t
    d.ramp("pedal", 0.0, 0.2)
    d.wait(4.0)
    d.brake_to_stop()
    marks["stopped"] = sim.t
    d.wait(5.0)
    return sim, d.rec, marks


def test_never_stalls_on_trip(trip):
    sim, rec, _ = trip
    assert all(r["mode"] == MODE_RUN for r in rec.rows)


def test_cruise_100_in_fifth(trip):
    sim, rec, m = trip
    rows = rec.between(m["cruise"] + 10.0, m["lift"])
    avg = lambda k: statistics.fmean(r[k] for r in rows)  # noqa: E731
    assert avg("gear") == 5
    assert 98.0 <= avg("kmh") <= 102.0
    assert 2600 <= avg("rpm") <= 2800
    assert 45 <= avg("map_kpa") <= 65
    assert 15 <= avg("maf_gps") <= 26
    assert all(r["fuel_status"] == FS_CL for r in rows)
    assert all(r["clutch_locked"] == 1.0 for r in rows)


def test_decel_fuel_cut_in_gear(trip):
    sim, rec, m = trip
    rows = rec.between(m["lift"] + 0.5, m["lift"] + 4.0)
    assert any(r["dfco"] == 1.0 and r["fuel_status"] == FS_OL_DRIVE for r in rows)


def test_stops_and_idles(trip):
    sim, rec, m = trip
    end = rec.between(m["stopped"] + 3.0, m["stopped"] + 5.0)
    assert all(r["kmh"] == 0.0 and r["gear"] == 0 for r in end)
    assert all(650 <= r["rpm"] <= 900 for r in end)
    assert sim.plant.vehicle.odometer_m > 1000.0


def test_idle_only_pull_away():
    sim = warm_sim()
    d = ScriptedDriver(sim)
    d.launch(1, pedal=0.0, slip_s=3.0)
    d.wait(3.0)
    assert sim.ecu.mode == "run"
    assert 4.0 < sim.plant.vehicle.kmh < 8.0  # idle speed in first gear
    assert min(r["rpm"] for r in d.rec.rows) > 400


def test_clutch_dump_against_brakes_stalls():
    sim = warm_sim()
    d = ScriptedDriver(sim)
    sim.driver.brake = 0.6
    d.ramp("clutch", 1.0, 0.2)
    sim.driver.gear = 1
    d.wait(0.3)
    sim.driver.clutch = 0.0
    d.wait(1.5)
    assert sim.ecu.mode == "koeo"
    assert sim.plant.engine.rpm == 0.0
    assert sim.plant.vehicle.kmh == 0.0


def test_shift_needs_clutch():
    sim = warm_sim()
    sim.driver.gear = 1
    sim.run(0.05)
    assert sim.plant.vehicle.gear == 0 and sim.driver.gear == 0
    assert sim.plant.vehicle.last_shift_refused
    sim.driver.clutch = 1.0
    sim.driver.gear = 1
    sim.run(0.05)
    assert sim.plant.vehicle.gear == 1


def test_starter_interlock():
    sim = Simulator(ambient_c=20.0)
    sim.driver.clutch = 1.0
    sim.driver.gear = 1
    sim.run(0.05)
    sim.driver.clutch = 0.0
    assert sim.start_engine(max_crank_s=2.0) < 0  # in gear, clutch up: starter does not engage
    sim.driver.clutch = 1.0
    assert sim.start_engine() > 0


def test_wot_zero_to_hundred():
    sim = warm_sim()
    d = ScriptedDriver(sim)
    t0 = sim.t
    d.launch(1, pedal=0.5, slip_s=1.2)
    d.accelerate_until(1.0, rpm=6000, kmh=100)
    for g in (2, 3):
        if sim.plant.vehicle.kmh >= 100:
            break
        d.shift(g, clutch_release_s=0.3, pedal_after=1.0)
        d.accelerate_until(1.0, rpm=6000, kmh=100)
    elapsed = sim.t - t0
    assert sim.plant.vehicle.kmh >= 100
    assert 9.0 <= elapsed <= 14.0
    assert any(r["pe"] == 1.0 for r in d.rec.rows)  # power enrichment at WOT
