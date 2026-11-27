"""DTC lifecycle rules in isolation (DESIGN §6.1)."""

import pytest

from ecusim.ecu.diag.codes import decode_dtc, encode_dtc
from ecusim.ecu.diag.dtc import DtcManager


def trip(m: DtcManager, code: str, result: bool | None) -> None:
    """One drive cycle in which ``code`` fails (True), passes (False) or is not tested (None)."""
    if result is not None:
        m.report(code, result)
    m.end_trip()


def test_two_trip_confirmation():
    m = DtcManager()
    m.report("P0171", True)
    assert m.pending == ["P0171"] and m.confirmed == [] and not m.mil_on
    m.end_trip()
    m.report("P0171", True)
    assert m.confirmed == ["P0171"] and m.permanent == ["P0171"] and m.mil_on


def test_pending_cleared_by_passing_trip():
    m = DtcManager()
    trip(m, "P0171", True)
    trip(m, "P0171", False)
    assert m.pending == [] and m.records == {}
    trip(m, "P0171", True)  # starts over: pending only
    assert m.pending == ["P0171"] and not m.mil_on


def test_untested_trip_keeps_pending_and_consecutive_count():
    m = DtcManager()
    trip(m, "P0420", True)
    trip(m, "P0420", None)
    assert m.pending == ["P0420"]
    m.report("P0420", True)
    assert m.confirmed == ["P0420"]


def test_mil_off_after_three_good_trips_permanent_erased():
    m = DtcManager()
    trip(m, "P0301", True)
    trip(m, "P0301", True)
    for i in range(2):
        trip(m, "P0301", False)
        assert m.mil_on, f"MIL went off after {i + 1} good trips"
    trip(m, "P0301", False)
    assert not m.mil_on
    assert m.confirmed == ["P0301"]  # history stays until aged out
    assert m.permanent == []


def test_failure_resets_good_trip_count():
    m = DtcManager()
    trip(m, "P0301", True)
    trip(m, "P0301", True)
    trip(m, "P0301", False)
    trip(m, "P0301", False)
    trip(m, "P0301", True)
    trip(m, "P0301", False)
    trip(m, "P0301", False)
    assert m.mil_on


def test_aged_out_after_40_warmups():
    m = DtcManager()
    trip(m, "P0301", True)
    trip(m, "P0301", True)
    for _ in range(3):
        trip(m, "P0301", False)
    for _ in range(39):
        m.warmup_cycle()
    assert m.confirmed == ["P0301"]
    m.warmup_cycle()
    assert m.confirmed == [] and m.records == {}


def test_clear_keeps_permanent_until_monitor_passes():
    m = DtcManager(capture=lambda: {"rpm": 800.0})
    trip(m, "P0303", True)
    m.report("P0303", True)
    assert m.freeze_frame.code == "P0303"
    m.clear()
    assert m.confirmed == [] and m.pending == [] and not m.mil_on
    assert m.permanent == ["P0303"]
    assert m.freeze_frame is None
    trip(m, "P0303", None)
    assert m.permanent == ["P0303"]
    trip(m, "P0303", False)
    assert m.permanent == []


def test_freeze_frame_priority():
    m = DtcManager(capture=lambda: {"src": "first"})
    trip(m, "P0128", True)
    m.report("P0128", True)
    assert m.freeze_frame.code == "P0128"
    m.capture = lambda: {"src": "misfire"}
    m.report("P0302", True)  # misfire stores at pending and overrides lower priority
    assert m.freeze_frame.code == "P0302" and m.freeze_frame.values["src"] == "misfire"
    m.capture = lambda: {"src": "other"}
    trip(m, "P0562", True)
    m.report("P0562", True)
    assert m.freeze_frame.code == "P0302"


def test_export_load_roundtrip():
    m = DtcManager(capture=lambda: {"rpm": 1.0})
    trip(m, "P0171", True)
    m.report("P0171", True)
    n = DtcManager()
    n.load(m.export())
    assert n.confirmed == ["P0171"] and n.permanent == ["P0171"] and n.freeze_frame.code == "P0171"


@pytest.mark.parametrize("code,raw", [("P0171", b"\x01\x71"), ("P0300", b"\x03\x00"), ("P2135", b"\x21\x35"),
                                      ("C0035", b"\x40\x35"), ("U0100", b"\xc1\x00"), ("B1A2F", b"\x9a\x2f")])
def test_dtc_encoding(code, raw):
    assert encode_dtc(code) == raw
    assert decode_dtc(raw) == code
