"""Fault injection mechanics, failsafes and immediate circuit diagnostics (M5)."""

import math

import pytest

from ecusim.faults.catalog import FAULTS, SENSOR
from ecusim.faults.manager import FaultSetting
from ecusim.faults.presets import BUILTIN, PresetStore
from ecusim.sim import Simulator


def warm(seed=3) -> Simulator:
    s = Simulator(ambient_c=20.0, seed=seed)
    s.plant.set_warm()
    s.sensors = type(s.sensors)(s.rng)
    return s


def test_every_fault_applies_and_restores():
    s = warm()
    s.start_engine()
    for d in FAULTS:
        s.faults.set(FaultSetting(d.key, mode=d.modes[0] if d.kind == SENSOR else "", cylinder=2))
        s.run(0.05)
        s.faults.clear(d.key)
        s.run(0.05)
    assert s.faults._applied == {} and s.sensors.faults == {}
    p = s.plant
    assert p.air.vacuum_leak_cda_m2 == 0.0 and p.exhaust.cat_health == 1.0 and p.thermal.thermostat_stuck is None
    assert p.fuel.injector_health == [1.0] * 4 and p.engine.coil_failed == [False] * 4
    assert p.fuel_pressure_factor == 1.0 and p.vvt_stuck_deg is None and s.env.fuel_octane == 95.0


def test_intermittent_fault_toggles():
    s = warm()
    s.faults.set(FaultSetting("s_ect", mode="open", intermittent=True, duty=0.5, period_s=2.0))
    seen = set()
    for _ in range(300):
        s.step()
        seen.add("ect" in s.sensors.faults)
    assert seen == {True, False}


def test_per_cylinder_fault_moves_cleanly():
    s = warm()
    s.faults.set(FaultSetting("coil", cylinder=1))
    s.run(0.05)
    s.faults.set(FaultSetting("coil", cylinder=4))
    s.run(0.05)
    assert s.plant.engine.coil_failed == [False, False, False, True]


def test_ect_open_failsafe_and_raw_reading():
    s = Simulator(ambient_c=15.0, seed=1)
    s.faults.set(FaultSetting("s_ect", mode="open"))
    assert s.start_engine() > 0
    s.run(30.0)
    e = s.ecu
    assert e.obd_values()["ect"] < -35.0  # scan tool sees the open circuit
    assert e.inputs.ect_failed and e.fan == 2  # fan forced on
    assert 15.0 < e.inputs.ect_model < 40.0  # modelled warm-up used for control
    assert e.mode == "run"
    assert "P0118" in e.diag.dtc.pending


def test_throttle_correlation_limp_home():
    s = warm()
    s.faults.set(FaultSetting("s_tps_b", mode="offset", amount=15.0))
    s.start_engine()
    s.run(5.0)
    assert s.ecu.limp and s.ecu.epc
    assert "P2135" in s.ecu.diag.dtc.pending
    assert s.ecu.act.throttle_enable is False
    s.driver.pedal = 1.0
    s.run(5.0)
    assert s.plant.engine.rpm < 2700.0  # pedal ignored, limp rev limit
    assert s.plant.air.throttle == pytest.approx(s.plant.limp_throttle, abs=0.01)


def test_coil_failure_cuts_injector_without_lean_code():
    s = warm()
    s.faults.set(FaultSetting("coil", cylinder=3))
    s.start_engine()
    s.run(120.0)
    assert s.ecu.act.pw_ms[2] == 0.0 and s.ecu.act.pw_ms[0] > 0.0
    pending = s.ecu.diag.dtc.pending
    assert "P0353" in pending and "P0303" in pending
    assert "P0171" not in pending
    assert abs(s.ecu.fuel.stft + s.ecu.fuel.ltft_active) < 0.1


def test_dead_crank_sensor_no_start():
    s = warm()
    s.faults.set(FaultSetting("s_ckp", mode="open"))
    assert s.start_engine(max_crank_s=4.0) < 0
    assert "P0335" in s.ecu.diag.dtc.pending


def test_dead_cam_sensor_slower_start():
    healthy = warm()
    t_ok = healthy.start_engine()
    s = warm()
    s.faults.set(FaultSetting("s_cmp", mode="open"))
    t_bad = s.start_engine()
    assert t_bad > t_ok
    s.run(3.0)
    assert "P0340" in s.ecu.diag.dtc.pending


def test_low_octane_knock_control():
    s = warm()
    s.env.fuel_octane = 87.0
    s.start_engine()
    s.run(3.0)
    s.driver.pedal = 1.0
    w = 2500 * 2 * math.pi / 60
    retard = []
    for _ in range(300):
        s.plant.engine.omega = w
        s.step()
        retard.append(max(s.ecu.spark.knock_retard))
    assert sum(retard[100:]) / 200 > 2.0
    assert s.ecu.diag.dtc.pending == []


def test_knock_sensor_failure_safety_retard():
    s = warm()
    s.faults.set(FaultSetting("s_knock", mode="open"))
    s.start_engine()
    s.driver.pedal = 0.4
    s.run(6.0)
    assert s.ecu.inputs.knock_failed
    assert max(s.ecu.spark.knock_retard) == pytest.approx(5.0)
    assert "P0327" in s.ecu.diag.dtc.pending


def test_presets(tmp_path):
    store = PresetStore(tmp_path / "presets.json")
    assert len(store.all()) == len(BUILTIN)
    store.save({"name": "Mine", "ambient": 3.0, "faults": [{"key": "fan"}]})
    again = PresetStore(tmp_path / "presets.json")
    assert again.get("Mine")["faults"] == [{"key": "fan"}]
    for p in BUILTIN:
        for f in p["faults"]:
            FaultSetting.from_dict(f)  # every built-in preset fault is valid
