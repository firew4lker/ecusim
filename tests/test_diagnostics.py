"""End-to-end diagnostics through the OBD services: two-trip logic, Mode 04, permanent DTCs,
freeze frame, Mode 06, readiness, IUMPR, persistence (M4)."""

import pytest

from ecusim.ecu.diag import MIL_FLASH, MIL_OFF, MIL_ON
from ecusim.obd.datasource import diag_snapshot
from ecusim.obd.pids import MODE01
from ecusim.obd.server import ObdServer
from ecusim.persistence import StateStore, capture
from ecusim.obd.vehicle_info import VehicleInfo
from ecusim.sim import Simulator


class Rig:
    def __init__(self, seed=5):
        self.sim = Simulator(ambient_c=20.0, seed=seed)
        self.sim.plant.set_warm()
        self.sim.sensors = type(self.sim.sensors)(self.sim.rng)
        self.server = ObdServer(lambda: diag_snapshot(self.sim),
                                on_clear=lambda: self.sim.ecu.diag.clear())

    def obd(self, req: bytes) -> bytes | None:
        return self.server.handle(req, functional=False)

    def dtcs(self, mode: int) -> list[str]:
        from ecusim.ecu.diag.codes import decode_dtc

        r = self.obd(bytes([mode]))
        n = r[1]
        return [decode_dtc(r[2 + 2 * i: 4 + 2 * i]) for i in range(n)]

    def trip(self, seconds=150.0, misfire=0.0, key_off=True):
        self.sim.plant.engine.misfire_fault[2] = misfire
        assert self.sim.start_engine() > 0
        self.sim.run(seconds)
        if key_off:
            self.key_off()

    def key_off(self):
        self.sim.driver.key = "off"
        self.sim.run(1.0)
        self.sim.driver.key = "on"  # key on, engine off: ECU answers, MIL bulb check
        self.sim.run(0.5)


@pytest.fixture(scope="module")
def story():
    """Misfire on cylinder 3 for two trips, then fixed; recorded step by step."""
    rig = Rig()
    log = {}
    rig.trip(misfire=0.25)
    log["t1"] = {m: rig.dtcs(m) for m in (3, 7, 10)}
    log["t1_mil"] = rig.sim.ecu.diag.mil
    rig.trip(misfire=0.25, key_off=False)
    log["t2_running"] = {m: rig.dtcs(m) for m in (3, 7, 10)}
    log["t2_pid01"] = rig.obd(b"\x01\x01")
    log["t2_mil"] = rig.sim.ecu.diag.mil
    log["ff_dtc"] = rig.obd(b"\x02\x02\x00")
    log["ff_rpm"] = rig.obd(b"\x02\x0c\x00")
    log["mode06_a4"] = rig.obd(b"\x06\xa4")
    log["mode04_running"] = rig.obd(b"\x04")
    rig.key_off()
    log["mode04_koeo"] = rig.obd(b"\x04")
    rig.sim.run(0.1)
    log["after_clear"] = {m: rig.dtcs(m) for m in (3, 7, 10)}
    log["after_clear_pid01"] = rig.obd(b"\x01\x01")
    log["after_clear_ff"] = rig.obd(b"\x02\x02\x00")
    rig.trip(misfire=0.0)
    log["after_good_trip"] = {m: rig.dtcs(m) for m in (3, 7, 10)}
    log["iumpr"] = rig.obd(b"\x09\x08")
    return rig, log


def test_trip1_pending_only(story):
    _, log = story
    assert log["t1"] == {3: [], 7: ["P0303"], 10: []}
    assert log["t1_mil"] == MIL_OFF


def test_trip2_confirmed_mil_permanent(story):
    _, log = story
    assert log["t2_running"] == {3: ["P0303"], 7: ["P0303"], 10: ["P0303"]}
    assert log["t2_mil"] == MIL_ON
    a, b, c, d = log["t2_pid01"][2:6]
    assert a == 0x81  # MIL on, one stored DTC
    assert b & 0x07 == 0x07 and c == 0xE5


def test_freeze_frame(story):
    _, log = story
    assert log["ff_dtc"] == b"\x42\x02\x00\x03\x03"
    rpm = MODE01[0x0C].decode(log["ff_rpm"][3:5])
    assert 600 < rpm < 1000


def test_mode06_misfire_counts(story):
    _, log = story
    r = log["mode06_a4"]  # cylinder 3
    assert r[:2] == b"\x46\xa4"
    # 46 then 9-byte rows: MID TID UASID value(2) min(2) max(2)
    rows = {r[1 + 9 * i + 1]: int.from_bytes(r[1 + 9 * i + 3: 1 + 9 * i + 5], "big") for i in range((len(r) - 1) // 9)}
    assert rows[0x0C] > 50  # misfires counted this cycle on cylinder 3


def test_mode04_rules(story):
    _, log = story
    assert log["mode04_running"] == b"\x7f\x04\x22"
    assert log["mode04_koeo"] == b"\x44"
    assert log["after_clear"] == {3: [], 7: [], 10: ["P0303"]}
    a, b, c, d = log["after_clear_pid01"][2:6]
    assert a == 0x00
    assert b == 0x77  # continuous monitors supported and incomplete again
    assert d == 0xE5  # every non-continuous monitor incomplete
    assert log["after_clear_ff"] == b"\x42\x02\x00\x00\x00"


def test_permanent_erased_after_passing_trip(story):
    _, log = story
    assert log["after_good_trip"] == {3: [], 7: [], 10: []}


def test_iumpr_response(story):
    rig, log = story
    r = log["iumpr"]
    assert r[:3] == b"\x49\x08\x14" and len(r) == 3 + 40
    ign = int.from_bytes(r[5:7], "big")
    assert ign == rig.sim.ecu.diag.iumpr.ign_cntr == 3


def test_catalyst_damaging_misfire_flashes():
    rig = Rig(seed=7)
    rig.trip(seconds=45.0, misfire=0.9, key_off=False)
    assert rig.sim.ecu.diag.mil == MIL_FLASH
    assert "P0303" in rig.dtcs(7)


def test_vacuum_leak_sets_p0171():
    rig = Rig(seed=11)
    rig.sim.plant.air.vacuum_leak_cda_m2 = 5.0e-6
    rig.trip(seconds=240.0, key_off=False)
    assert rig.sim.ecu.fuel.ltft[0] > 0.20
    assert "P0171" in rig.dtcs(7)


def test_forced_readiness():
    rig = Rig()
    rig.sim.driver.key = "on"
    rig.sim.run(0.2)
    for key in ("catalyst", "evap", "o2", "o2_heater", "egr_vvt"):
        rig.sim.ecu.diag.monitor(key).force = "complete"
    rig.sim.ecu.diag.monitor("misfire").force = "complete"
    rig.sim.run(0.05)
    a, b, c, d = rig.obd(b"\x01\x01")[2:6]
    assert d == 0x00 and b & 0x10 == 0


def test_persistence_roundtrip_and_battery_disconnect(tmp_path):
    rig = Rig()
    rig.trip(misfire=0.25)
    rig.trip(misfire=0.25)
    store = StateStore(tmp_path)
    kam, nvm = capture(rig.sim, VehicleInfo())
    store.save(kam, nvm)

    k2, n2 = store.load()
    restored = StateStore.build_sim(k2, n2, now=kam["saved_at"] + 600.0)
    dtc = restored.ecu.diag.dtc
    assert dtc.confirmed == ["P0303"] and dtc.permanent == ["P0303"] and dtc.freeze_frame.code == "P0303"
    assert 75.0 < restored.plant.thermal.coolant_c < 90.0  # cooled for 10 minutes
    assert restored.plant.vehicle.odometer_m == pytest.approx(rig.sim.plant.vehicle.odometer_m)

    later = StateStore.build_sim(k2, n2, now=kam["saved_at"] + 24 * 3600.0)
    assert later.plant.thermal.coolant_c == pytest.approx(20.0, abs=0.5)

    store.wipe_kam()
    k3, n3 = store.load()
    assert k3 is None
    after_batt = StateStore.build_sim(k3, n3)
    assert after_batt.ecu.diag.dtc.permanent == ["P0303"]
    assert after_batt.ecu.diag.dtc.confirmed == []
