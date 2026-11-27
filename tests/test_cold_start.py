# SPDX-License-Identifier: GPL-3.0-or-later
"""Cold start at 15 °C and idle warm-up (DESIGN §12.1)."""

import statistics

import pytest

from ecusim.ecu.fuel import FS_CL, FS_OL_TEMP
from ecusim.scenarios import cold_start_idle

MODE_RUN = 3


@pytest.fixture(scope="module")
def cold():
    rec, crank, t0 = cold_start_idle(ambient_c=15.0, seconds=960.0)
    return rec, crank, t0


def test_cranks_and_starts_quickly(cold):
    _, crank, _ = cold
    assert 0.3 < crank < 2.0


def test_never_stalls(cold):
    rec, _, _ = cold
    assert all(r["mode"] == MODE_RUN for r in rec.rows)


def test_start_flare(cold):
    rec, _, t0 = cold
    peak = max(r["rpm"] for r in rec.between(t0, t0 + 3.0))
    assert 1350 <= peak <= 1700


def test_open_loop_rich_after_start(cold):
    rec, _, t0 = cold
    r = rec.at(t0 + 5.0)
    assert r["fuel_status"] == FS_OL_TEMP
    assert r["lambda_cmd"] < 0.95
    assert 1050 <= r["rpm"] <= 1350


def test_catalyst_heating(cold):
    rec, _, t0 = cold
    r = rec.at(t0 + 10.0)
    assert r["cat_heat"] == pytest.approx(1.0)
    assert r["spark_deg"] < 5.0  # retarded spark
    lit = rec.first(lambda r: r["cat_c"] > 300.0, after=t0)
    assert lit is not None and lit["t"] - t0 < 45.0


def test_enters_closed_loop(cold):
    rec, _, t0 = cold
    cl = rec.first(lambda r: r["fuel_status"] == FS_CL, after=t0)
    assert cl is not None
    assert 20.0 <= cl["t"] - t0 <= 60.0


def test_charging_voltage(cold):
    rec, _, t0 = cold
    volts = [r["vbatt"] for r in rec.between(t0 + 5.0, t0 + 960.0)]
    assert 13.8 <= min(volts) and max(volts) <= 14.6


def test_warmup_time(cold):
    rec, _, t0 = cold
    hot = rec.first(lambda r: r["ect_c"] >= 90.0, after=t0)
    assert hot is not None
    assert 600.0 <= hot["t"] - t0 <= 960.0


def test_warm_idle_values(cold):
    rec, _, t0 = cold
    tail = rec.between(t0 + 900.0, t0 + 960.0)
    avg = lambda k: statistics.fmean(r[k] for r in tail)  # noqa: E731
    assert 720 <= avg("rpm") <= 780
    assert 28 <= avg("map_kpa") <= 38
    assert 2.2 <= avg("maf_gps") <= 3.2
    assert 8 <= avg("spark_deg") <= 20
    assert 2.5 <= avg("pw_ms") <= 4.0
    assert abs(avg("stft_pct")) < 5
    assert abs(avg("ltft_pct")) < 5
    assert 0.55 <= avg("nb_volts") <= 0.80
    assert 350 <= avg("cat_c") <= 600
    assert all(r["fuel_status"] == FS_CL for r in tail)
    assert 0.97 <= avg("lambda_wb") <= 1.03
