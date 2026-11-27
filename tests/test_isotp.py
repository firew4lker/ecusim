# SPDX-License-Identifier: GPL-3.0-or-later
"""ISO 15765-2 segmentation, flow control and reassembly."""

import pytest

from ecusim.obd.isotp import IsoTpConfig, IsoTpLink, stmin_to_seconds


def pump(a: IsoTpLink, b: IsoTpLink, now: float = 0.0, rounds: int = 50) -> None:
    """Shuttle frames between two links until quiet."""
    for _ in range(rounds):
        fa = a.poll(now)
        fb = b.poll(now)
        if not fa and not fb:
            return
        for f in fa:
            b.on_frame(f, now)
        for f in fb:
            a.on_frame(f, now)


def test_single_frame_padded():
    a = IsoTpLink(IsoTpConfig(padding=0xAA))
    a.send(b"\x41\x0c\x1a\xf8", 0.0)
    assert a.poll(0.0) == [bytes([0x04, 0x41, 0x0C, 0x1A, 0xF8, 0xAA, 0xAA, 0xAA])]


def test_single_frame_unpadded():
    a = IsoTpLink(IsoTpConfig(padding=None))
    a.send(b"\x41\x05\x82", 0.0)
    assert a.poll(0.0) == [bytes([0x03, 0x41, 0x05, 0x82])]


def test_vin_multiframe_layout_and_reassembly():
    vin = b"1SMEC16D0FS000001"
    msg = bytes([0x49, 0x02, 0x01]) + vin  # 20 bytes
    ecu, tester = IsoTpLink(), IsoTpLink()
    ecu.send(msg, 0.0)
    ff = ecu.poll(0.0)
    assert ff == [bytes([0x10, 0x14]) + msg[:6]]
    assert ecu.busy
    tester.on_frame(ff[0], 0.0)
    fc = tester.poll(0.0)
    assert fc == [bytes([0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA])]
    ecu.on_frame(fc[0], 0.0)
    cfs = ecu.poll(0.0)
    assert [f[0] for f in cfs] == [0x21, 0x22]
    assert cfs[1][1:] == msg[13:20]
    for f in cfs:
        tester.on_frame(f, 0.0)
    assert tester.rx_queue.popleft() == msg
    assert not ecu.busy


def test_sequence_number_wraps():
    data = bytes(range(256)) * 2  # 512 bytes -> 73 CFs
    a, b = IsoTpLink(), IsoTpLink()
    a.send(data, 0.0)
    pump(a, b)
    assert b.rx_queue.popleft() == data
    assert not a.errors and not b.errors


def test_block_size_and_stmin_respected():
    data = bytes(40)  # FF(6) + 5 CFs
    ecu = IsoTpLink()
    tester = IsoTpLink(IsoTpConfig(rx_block_size=2, rx_stmin=10))
    ecu.send(data, 0.0)
    tester.on_frame(ecu.poll(0.0)[0], 0.0)
    ecu.on_frame(tester.poll(0.0)[0], 0.0)
    first = ecu.poll(0.0)
    assert len(first) == 1  # STmin 10 ms: one CF now
    assert ecu.poll(0.005) == []
    second = ecu.poll(0.0101)
    assert len(second) == 1
    assert ecu.poll(0.5) == []  # block of 2 done: waits for the next FC
    for f in first + second:
        tester.on_frame(f, 0.01)
    fc = tester.poll(0.011)
    assert fc and fc[0][0] == 0x30
    ecu.on_frame(fc[0], 0.02)
    now = 0.02
    while ecu.busy or not tester.rx_queue:
        for f in ecu.poll(now):
            tester.on_frame(f, now)
        for f in tester.poll(now):
            ecu.on_frame(f, now)
        now += 0.011
        if now > 1.0:
            break
    assert tester.rx_queue.popleft() == data


def test_flow_control_timeout_aborts():
    ecu = IsoTpLink(IsoTpConfig(n_bs=1.0))
    ecu.send(bytes(20), 0.0)
    ecu.poll(0.0)
    assert ecu.poll(1.5) == []
    assert not ecu.busy
    assert "N_Bs" in ecu.errors[0]


def test_wait_then_overflow():
    ecu = IsoTpLink()
    ecu.send(bytes(20), 0.0)
    ecu.poll(0.0)
    ecu.on_frame(bytes([0x31, 0, 0]), 0.1)  # WAIT restarts N_Bs
    assert ecu.poll(1.05) == [] and ecu.busy
    ecu.on_frame(bytes([0x32, 0, 0]), 1.1)  # overflow
    assert not ecu.busy and "overflow" in ecu.errors[0]


def test_wrong_sequence_number_drops_message():
    rx = IsoTpLink()
    rx.on_frame(bytes([0x10, 0x10, 1, 2, 3, 4, 5, 6]), 0.0)
    rx.poll(0.0)
    rx.on_frame(bytes([0x22, 0, 0, 0, 0, 0, 0, 0]), 0.0)
    assert not rx.rx_queue and rx.errors


def test_consecutive_frame_timeout():
    rx = IsoTpLink(IsoTpConfig(n_cr=1.0))
    rx.on_frame(bytes([0x10, 0x10, 1, 2, 3, 4, 5, 6]), 0.0)
    rx.poll(0.0)
    rx.poll(1.2)
    rx.on_frame(bytes([0x21, 7, 8, 9, 10, 11, 12, 13]), 1.3)
    assert not rx.rx_queue
    assert any("N_Cr" in e for e in rx.errors)


@pytest.mark.parametrize("raw,expected", [(0, 0.0), (0x7F, 0.127), (0xF1, 0.0001), (0xF9, 0.0009), (0x80, 0.127)])
def test_stmin_decoding(raw, expected):
    assert stmin_to_seconds(raw) == pytest.approx(expected)
