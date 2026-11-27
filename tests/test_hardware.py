# SPDX-License-Identifier: GPL-3.0-or-later
"""Bench tests on real CAN hardware (skipped unless enabled).

Wire two adapters together (CAN-H to CAN-H, CAN-L to CAN-L, 120 Ω at each end), bring both up at
500 kbit/s and run:

    ECUSIM_HW=can0,can1 .venv/bin/python -m pytest -q tests/test_hardware.py
    ECUSIM_HW=slcan:/dev/ttyACM0,can1 .venv/bin/python -m pytest -q tests/test_hardware.py

Each side is ``interface:channel``; a bare name means SocketCAN.

The simulated ECU runs on the first interface, the tester on the second, so every frame crosses
the physical bus. Where can-utils' isotpsend/isotprecv are installed, the kernel's ISO-TP stack
is used as an independent tester as well.
"""

import os
import shutil
import subprocess
import time

import pytest

HW = os.environ.get("ECUSIM_HW", "")
pytestmark = pytest.mark.skipif(not HW, reason="set ECUSIM_HW=ecu_if,tester_if to run on real CAN hardware")

if HW:
    import can  # noqa: F401

    from ecusim.obd.can_transport import CanConfig, EcuCanServer
    from ecusim.obd.client import NegativeResponse, ObdClient, ObdTimeout
    from ecusim.obd.server import ObdServer
    from ecusim.obd.vehicle_info import DEFAULT_VIN
    from ecusim.runtime import RealtimeRunner
    from ecusim.sim import Simulator

def _split(spec: str) -> tuple[str, str]:
    iface, _, chan = spec.partition(":")
    return (iface, chan) if chan else ("socketcan", iface)


(ECU_KIND, ECU_IF), (TESTER_KIND, TESTER_IF) = [_split(s) for s in (HW.split(",") + ["", ""])[:2]]


@pytest.fixture(params=[False, True], ids=["11bit", "29bit"])
def bench(request):
    extended = request.param
    sim = Simulator(ambient_c=20.0)
    sim.start_engine()
    sim.run(3.0)
    runner = RealtimeRunner(sim)
    server = EcuCanServer(CanConfig(ECU_KIND, ECU_IF, extended=extended),
                          ObdServer(runner.diag_source).handle, runner.powered)
    runner.start()
    server.start()
    client = ObdClient(CanConfig(TESTER_KIND, TESTER_IF, extended=extended), p2_timeout=0.1)
    yield runner, server, client, extended
    client.close()
    server.stop()
    runner.stop()


def test_basic_services_over_the_wire(bench):
    runner, server, client, _ = bench
    assert 600 < client.read_pid(0x0C) < 1600
    assert client.vin() == DEFAULT_VIN
    assert client.ecu_name() == "ECM-EngineControl"
    assert client.dtcs(0x03) == []
    with pytest.raises(NegativeResponse):
        client.request(b"\x04", functional=False)  # engine running: conditions not correct
    assert server.stats["errors"] == 0


def test_sustained_polling_no_timeouts(bench):
    _, server, client, _ = bench
    timeouts = 0
    worst = 0.0
    for i in range(500):
        t = time.perf_counter()
        try:
            if i % 10 == 9:
                assert client.vin() == DEFAULT_VIN
            else:
                client.read_pids([0x0C, 0x05, 0x0B, 0x10, 0x0E, 0x06])
        except ObdTimeout:
            timeouts += 1
        worst = max(worst, time.perf_counter() - t)
    assert timeouts == 0
    assert worst < 0.05  # well inside P2


@pytest.mark.skipif(bool(not shutil.which("isotpsend") or (HW and _split(HW.split(",")[1])[0] != "socketcan")),
                    reason="needs can-utils ISO-TP tools and a SocketCAN tester interface")
def test_kernel_isotp_flow_control(bench):
    """Kernel ISO-TP as tester, demanding block size 1 and STmin 32 ms."""
    _, _, client, extended = bench
    src, dst = ("18DA10F1", "18DAF110") if extended else ("7E0", "7E8")
    rx = subprocess.Popen(["timeout", "3", "isotprecv", "-s", src, "-d", dst, "-p", "AA", "-b", "1", "-m", "20",
                           TESTER_IF], stdout=subprocess.PIPE, text=True)
    time.sleep(0.2)
    subprocess.run(["isotpsend", "-s", src, "-d", dst, "-p", "AA", TESTER_IF], input="09 02", text=True, check=True)
    out, _ = rx.communicate(timeout=5)
    data = bytes.fromhex(out.strip())
    assert data[:3] == b"\x49\x02\x01" and data[3:].decode() == DEFAULT_VIN
