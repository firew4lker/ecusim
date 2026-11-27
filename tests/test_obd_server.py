"""J1979 encodings and request rules (DESIGN §7.3-7.5)."""

import pytest

from ecusim.obd.pids import MODE01, active_support_pids, decode_bitmap, support_bitmap
from ecusim.obd.server import ObdServer
from ecusim.obd.vehicle_info import DEFAULT_VIN, VehicleInfo, vin_check_digit, vin_is_valid, with_check_digit
from ecusim.sim import Simulator
from ecusim.obd.datasource import diag_snapshot


def base_snapshot(**over):
    d = {
        "powered": True, "running": True, "rpm": 1726.0, "mon01": (0, 7, 0xE5, 0xE5), "mon41": (0, 7, 0xE5, 0xE5),
        "fuel_status": 2, "calc_load": 25.0, "abs_load": 22.0, "ect": 90.0, "stft": -3.125, "ltft": 1.5625,
        "map": 34.0, "vss": 0.0, "spark": 15.0, "iat": 25.0, "maf": 2.75, "tps_a": 18.0, "tps_b": 82.0,
        "tps_rel": 4.0, "app_d": 15.0, "app_e": 7.5, "throttle_cmd": 6.0, "nb_volts": 0.65, "nb_ready": True,
        "nb_trim_pct": 1.0, "wb_ready": True, "wb_lambda": 1.0, "wb_pump_ma": 0.0, "lambda_cmd": 1.0,
        "obd_standard": 6, "run_time": 300.0, "dist_mil": 0, "dist_clear": 0, "time_mil": 0, "time_clear": 0,
        "warmups": 0, "purge": 0.0, "fuel_level": 60.0, "evap_pa": -125.0, "baro": 101.0, "cat_temp": 450.0,
        "vbatt": 14.3, "ambient": 15.0, "oil": 95.0, "fuel_rate": 0.85,
    }
    d.update(over)
    return d


@pytest.fixture
def server():
    snap = {"d": base_snapshot()}
    s = ObdServer(lambda: snap["d"])
    s._snap = snap
    return s


# ---------------------------------------------------------------- encodings
@pytest.mark.parametrize(
    "pid,field,value,raw",
    [
        (0x0C, "rpm", 1726.0, b"\x1a\xf8"),
        (0x05, "ect", 90.0, b"\x82"),
        (0x0F, "iat", -40.0, b"\x00"),
        (0x0E, "spark", 15.0, b"\x9e"),
        (0x0E, "spark", -10.0, b"\x6c"),
        (0x10, "maf", 2.75, b"\x01\x13"),
        (0x06, "stft", -3.125, b"\x7c"),
        (0x07, "ltft", 1.5625, b"\x82"),
        (0x42, "vbatt", 14.3, b"\x37\xdc"),
        (0x3C, "cat_temp", 450.0, b"\x13\x24"),
        (0x44, "lambda_cmd", 1.0, b"\x80\x00"),
        (0x32, "evap_pa", -125.0, b"\xfe\x0c"),
        (0x5E, "fuel_rate", 0.85, b"\x00\x11"),
    ],
)
def test_pid_encoding(pid, field, value, raw):
    assert MODE01[pid].encode(base_snapshot(**{field: value})) == raw


@pytest.mark.parametrize("pid", sorted(MODE01))
def test_pid_lengths_and_roundtrip(pid):
    p = MODE01[pid]
    data = p.encode(base_snapshot())
    assert len(data) == p.length
    p.decode(data)  # must not raise


def test_roundtrip_values():
    d = base_snapshot()
    assert MODE01[0x0C].decode(MODE01[0x0C].encode(d)) == pytest.approx(1726.0)
    lam, ma = MODE01[0x34].decode(MODE01[0x34].encode(base_snapshot(wb_lambda=1.25, wb_pump_ma=0.8)))
    assert lam == pytest.approx(1.25, abs=1e-4) and ma == pytest.approx(0.8, abs=0.01)
    assert MODE01[0x24].decode(MODE01[0x24].encode(base_snapshot(wb_ready=False))) == (0.0, 0.0)


def test_support_bitmaps():
    sup = set(MODE01)
    bm = support_bitmap(sup, 0x00)
    assert decode_bitmap(bm, 0x00) == {p for p in sup if p <= 0x20} | {0x20}
    assert active_support_pids(sup) == {0x00, 0x20, 0x40}
    last = support_bitmap(sup, 0x40)
    assert not (int.from_bytes(last, "big") & 1)  # nothing beyond 0x60


# ---------------------------------------------------------------- request rules
def test_single_pid(server):
    assert server.handle(b"\x01\x0c", True) == b"\x41\x0c\x1a\xf8"


def test_multi_pid_skips_unsupported(server):
    resp = server.handle(b"\x01\x0c\x05\x99\x0b", True)
    assert resp == b"\x41\x0c\x1a\xf8\x05\x82\x0b\x22"


def test_unsupported_pid_functional_is_silent(server):
    assert server.handle(b"\x01\x99", True) is None


def test_unsupported_pid_physical_nrc(server):
    assert server.handle(b"\x01\x99", False) == b"\x7f\x01\x12"


def test_unsupported_service(server):
    assert server.handle(b"\x22\xf1\x90", True) is None
    assert server.handle(b"\x22\xf1\x90", False) == b"\x7f\x22\x11"


def test_too_many_pids(server):
    assert server.handle(b"\x01" + bytes([5] * 7), False) == b"\x7f\x01\x13"


def test_silent_when_unpowered(server):
    server._snap["d"] = base_snapshot(powered=False)
    assert server.handle(b"\x01\x00", False) is None


def test_obd_standard_from_vehicle_info():
    s = ObdServer(lambda: base_snapshot(), VehicleInfo(region="us"))
    assert s.handle(b"\x01\x1c", True) == b"\x41\x1c\x01"


def test_mode09_vin(server):
    resp = server.handle(b"\x09\x02", True)
    assert resp[:3] == b"\x49\x02\x01" and resp[3:].decode() == DEFAULT_VIN and len(resp) == 20


def test_mode09_support_and_names(server):
    resp = server.handle(b"\x09\x00", True)
    assert decode_bitmap(resp[2:6], 0) == {0x02, 0x04, 0x06, 0x08, 0x0A}
    name = server.handle(b"\x09\x0a", True)
    assert name[3:] == b"ECM\x00-EngineControl\x00\x00" and len(name[3:]) == 20
    assert len(server.handle(b"\x09\x04", True)[3:]) == 16
    assert len(server.handle(b"\x09\x06", True)[3:]) == 4


def test_mode09_rejects_mixed_request(server):
    assert server.handle(b"\x09\x02\x04", False) == b"\x7f\x09\x12"


def test_dtc_modes_empty_and_clear_rules(server):
    assert server.handle(b"\x03", True) == b"\x43\x00"
    assert server.handle(b"\x07", True) == b"\x47\x00"
    assert server.handle(b"\x0a", True) == b"\x4a\x00"
    assert server.handle(b"\x04", True) == b"\x7f\x04\x22"  # engine running
    server._snap["d"] = base_snapshot(running=False)
    assert server.handle(b"\x04", True) == b"\x44"


# ---------------------------------------------------------------- VIN
def test_vin_check_digit_known_example():
    assert vin_check_digit("1M8GDM9AXKP042788") == "X"
    assert vin_is_valid("1M8GDM9AXKP042788")
    assert vin_is_valid(DEFAULT_VIN)
    assert with_check_digit("1M8GDM9A0KP042788") == "1M8GDM9AXKP042788"


def test_vin_rejects_bad_characters():
    assert not vin_is_valid("1M8GDM9AXKP04278O")


# ---------------------------------------------------------------- live snapshot
def test_diag_snapshot_from_running_sim():
    sim = Simulator(ambient_c=20.0)
    sim.start_engine()
    sim.run(5.0)
    d = diag_snapshot(sim)
    s = ObdServer(lambda: d)
    rpm = MODE01[0x0C].decode(s.handle(b"\x01\x0c", True)[2:4])
    assert 900 < rpm < 1500
    for pid in sorted(MODE01):
        resp = s.handle(bytes([0x01, pid]), False)
        assert resp[:2] == bytes([0x41, pid]) and len(resp) == 2 + MODE01[pid].length
