# SPDX-License-Identifier: GPL-3.0-or-later
"""End to end over a python-can virtual bus: simulator thread + ECU CAN server + tester client."""

import uuid

import can
import pytest

from ecusim.obd.can_transport import CanConfig, EcuCanServer
from ecusim.obd.client import NegativeResponse, ObdClient, ObdTimeout
from ecusim.obd.server import ObdServer
from ecusim.obd.vehicle_info import DEFAULT_VIN
from ecusim.runtime import RealtimeRunner
from ecusim.sim import Simulator


def _setup(extended: bool, padding):
    channel = f"ecusim-test-{uuid.uuid4().hex}"
    cfg = CanConfig(interface="virtual", channel=channel, extended=extended, padding=padding)
    sim = Simulator(ambient_c=20.0)
    sim.start_engine()
    sim.run(3.0)
    runner = RealtimeRunner(sim, time_scale=1.0)
    server = EcuCanServer(cfg, ObdServer(runner.diag_source).handle, runner.powered)
    runner.start()
    server.start()
    sniffer = can.Bus(interface="virtual", channel=channel)
    client = ObdClient(cfg)
    return runner, server, client, sniffer


@pytest.fixture(params=[(False, 0xAA), (True, 0x00), (False, None)], ids=["11bit-padAA", "29bit-pad00", "11bit-nopad"])
def rig(request):
    runner, server, client, sniffer = _setup(*request.param)
    yield runner, server, client, sniffer, request.param
    client.close()
    sniffer.shutdown()
    server.stop()
    runner.stop()


def test_live_rpm_ect_vin(rig):
    runner, server, client, sniffer, (extended, padding) = rig
    rpm = client.read_pid(0x0C)
    ect = client.read_pid(0x05)
    assert 600 < rpm < 1600
    assert 15 <= ect <= 30
    assert client.vin() == DEFAULT_VIN
    assert client.ecu_name() == "ECM-EngineControl"
    vals = client.read_pids([0x0C, 0x05, 0x0B, 0x10, 0x0E, 0x06])
    assert set(vals) == {0x0C, 0x05, 0x0B, 0x10, 0x0E, 0x06}
    assert 0x0C in client.supported_pids()
    assert server.stats["errors"] == 0


def test_frames_on_the_wire(rig):
    runner, server, client, sniffer, (extended, padding) = rig
    client.vin()
    frames = []
    while (m := sniffer.recv(timeout=0.05)) is not None:
        frames.append(m)
    resp_id = 0x18DAF110 if extended else 0x7E8
    ecu = [m for m in frames if m.arbitration_id == resp_id]
    assert [m.data[0] >> 4 for m in ecu] == [1, 2, 2]  # FF + 2 CFs for a 20-byte VIN
    assert all(m.is_extended_id == extended for m in ecu)
    client.read_pid(0x05)  # single-frame reply: 03 41 05 xx
    sf = [m for m in iter(lambda: sniffer.recv(timeout=0.05), None) if m.arbitration_id == resp_id]
    assert len(sf) == 1
    if padding is None:
        assert len(sf[0].data) == 4
    else:
        assert all(len(m.data) == 8 for m in ecu + sf)
        assert bytes(sf[0].data[4:]) == bytes([padding]) * 4


def test_physical_negative_response_and_key_off(rig):
    runner, server, client, sniffer, _ = rig
    with pytest.raises(NegativeResponse) as e:
        client.request(b"\x01\x99", functional=False)
    assert e.value.code == 0x12
    runner.submit(lambda s: setattr(s.driver, "key", "off"))
    import time

    time.sleep(0.1)
    with pytest.raises(ObdTimeout):
        client.request(b"\x01\x00", timeout=0.3)
