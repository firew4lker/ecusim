"""Warm free-rev: rev limiter, decel fuel cut, return to idle (DESIGN §5.4)."""

import pytest

from ecusim.ecu.fuel import FS_OL_DRIVE
from ecusim.scenarios import warm_free_rev


@pytest.fixture(scope="module")
def rev():
    rec, sim = warm_free_rev()
    return rec


def _phase(rec, t0, t1):
    return rec.between(t0, t1)


def test_hits_rev_limiter(rev):
    t_blip = rev.rows[0]["t"] + 60.0
    peak = max(r["rpm"] for r in _phase(rev, t_blip, t_blip + 3.0))
    assert 6500 <= peak <= 6850


def test_wot_airflow(rev):
    t_blip = rev.rows[0]["t"] + 60.0
    assert max(r["maf_gps"] for r in _phase(rev, t_blip, t_blip + 3.0)) > 70.0


def test_decel_fuel_cut_and_recovery(rev):
    t_lift = rev.rows[0]["t"] + 63.0
    decel = _phase(rev, t_lift, t_lift + 10.0)
    cut = [r for r in decel if r["dfco"] == 1.0]
    assert cut, "expected decel fuel cut"
    assert all(r["fuel_status"] == FS_OL_DRIVE for r in cut)
    assert max(r["lambda_wb"] for r in cut) > 1.5  # sensor reads air
    assert min(r["map_kpa"] for r in decel) > 12.0
    assert min(r["rpm"] for r in decel) > 600.0  # no stall
    end = _phase(rev, t_lift + 8.0, t_lift + 10.0)
    assert all(650 <= r["rpm"] <= 900 for r in end)
