# SPDX-License-Identifier: GPL-3.0-or-later
"""ELM327 adapter emulation (DESIGN §7.7)."""

import json
import socket
import sys
import threading
import time
from argparse import ArgumentParser

import pytest

from ecusim.elm import cli_args
from ecusim.elm.emulator import SEARCH_S, CarLink, Elm327, segment
from ecusim.elm.manager import ElmManager
from ecusim.elm.transports import Channel, serve
from ecusim.obd.server import ObdServer
from ecusim.obd.vehicle_info import VehicleInfo
from ecusim.runtime import diag_snapshot
from ecusim.sim import Simulator

VIN = VehicleInfo().vin


@pytest.fixture(scope="module")
def snapshot():
    sim = Simulator(ambient_c=20.0)
    sim.start_engine()
    for _ in range(100):
        sim.step()
    return diag_snapshot(sim)


class Car:
    """Switchable fake car around a real ObdServer."""

    def __init__(self, snap):
        self.on = True
        self.ext, self.bitrate, self.padding = False, 500_000, 0xAA
        self.server = ObdServer(lambda: snap)

    def link(self) -> CarLink:
        return CarLink(self.server.handle, lambda: self.on, lambda: 14.2, lambda: (self.ext, self.bitrate,
                                                                                  self.padding))


@pytest.fixture
def car(snapshot):
    return Car(snapshot)


def talk(elm: Elm327, cmd: str) -> str:
    """Send one command; return the text without echo and prompt, lines joined by '|'."""
    (r,) = elm.feed((cmd + "\r").encode())
    assert r.body.endswith(">")
    return "|".join(x for x in r.body[:-1].replace("\n", "").split("\r") if x)


def ready(car, **kw) -> Elm327:
    """An adapter with echo off and the protocol set (no search on the first request)."""
    e = Elm327(car.link(), **kw)
    talk(e, "ATE0")
    talk(e, "ATSP6")
    return e


def test_init_sequence(car):
    e = Elm327(car.link())
    (r,) = e.feed(b"ATZ\r")
    assert r.echo == "ATZ\r" and r.body == "\r\rELM327 v1.5\r\r>"
    (r,) = e.feed(b"ATE0\r")
    assert r.echo == "ATE0\r" and r.body == "OK\r\r>"  # echoed before it takes effect
    (r,) = e.feed(b"ATI\r")
    assert r.echo == "" and r.body == "ELM327 v1.5\r\r>"
    assert talk(e, "ATRV") == "14.2V"
    assert talk(e, "AT@1") == "OBDII to RS232 Interpreter"
    assert talk(e, "ATSP0") == "OK"
    assert talk(e, "ATDPN") == "A0"
    assert talk(e, "0100").startswith("SEARCHING...|41 00 ")
    assert talk(e, "ATDPN") == "A6"
    assert talk(e, "ATDP") == "AUTO, ISO 15765-4 (CAN 11/500)"
    assert talk(e, "0100").startswith("41 00 ")  # no second search


def test_formatting_options(car):
    e = ready(car)
    rpm = talk(e, "010C")
    assert rpm.startswith("41 0C ") and len(rpm.split()) == 4
    talk(e, "ATS0")
    assert talk(e, "0105").startswith("4105") and " " not in talk(e, "0105")
    talk(e, "ATS1")
    talk(e, "ATH1")
    assert talk(e, "0105").startswith("7E8 03 41 05 ")
    talk(e, "ATD1")
    assert talk(e, "0105").startswith("7E8 8 03 41 05 ")
    talk(e, "ATD0")
    talk(e, "ATL1")
    (r,) = e.feed(b"ATI\r")
    assert r.body == "ELM327 v1.5\r\n\r\n>"
    talk(e, "ATL0")
    talk(e, "ATR0")
    assert talk(e, "010C") == ""
    talk(e, "ATR1")
    talk(e, "ATCAF0")
    talk(e, "ATH0")
    assert talk(e, "0105").split()[:3] == ["03", "41", "05"] and len(talk(e, "0105").split()) == 8


def test_multiframe_vin_both_ways(car):
    e = ready(car)
    hexvin = " ".join(f"{b:02X}" for b in VIN.encode())
    lines = talk(e, "0902").split("|")
    assert lines[0] == "014"
    assert lines[1] == "0: 49 02 01 " + hexvin[:8]
    joined = " ".join(x.split(": ", 1)[1] for x in lines[1:])
    assert joined == "49 02 01 " + hexvin
    talk(e, "ATH1")
    frames = talk(e, "0902").split("|")
    assert [f[:6] for f in frames] == ["7E8 10", "7E8 21", "7E8 22"]
    assert all(len(f.split()) == 9 for f in frames)  # ID + 8 bytes (padding shown on CFs)


def test_29bit_car(car):
    car.ext = True
    e = Elm327(car.link())
    talk(e, "ATH1")
    assert talk(e, "0100").startswith("SEARCHING...|18 DA F1 10 06 41 00")
    assert talk(e, "ATDPN") == "A7"
    talk(e, "ATSHDA10F1")  # physical
    assert talk(e, "010D").startswith("18 DA F1 10 03 41 0D")
    talk(e, "ATSP6")  # wrong ID format, right bitrate
    assert talk(e, "010D") == "NO DATA"


def test_protocol_mismatch_and_non_can(car):
    car.bitrate = 250_000
    e = Elm327(car.link())
    assert talk(e, "ATSP6") == "OK"
    assert talk(e, "010C") == "CAN ERROR"
    assert talk(e, "ATSP8") == "OK"
    assert talk(e, "010C").startswith("41 0C")
    talk(e, "ATSP3")
    assert talk(e, "010C") == "BUS INIT: ...ERROR"
    talk(e, "ATSP1")
    assert talk(e, "010C") == "NO DATA"


def test_key_off(car):
    car.on = False
    e = Elm327(car.link())
    (r,) = e.feed(b"0100\r")
    assert "SEARCHING..." in r.body and "UNABLE TO CONNECT" in r.body and r.delay == SEARCH_S
    assert talk(e, "ATRV") == "14.2V"  # the adapter itself is powered from the OBD socket
    car.on = True
    assert talk(e, "0100").startswith("SEARCHING...|41 00")
    car.on = False
    assert talk(e, "010C") == "NO DATA"
    assert talk(e, "ATIGN") == "OFF"


def test_headers_and_filters(car):
    e = ready(car)
    assert talk(e, "0122") == "NO DATA"  # functional, unsupported PID: the ECU stays silent
    talk(e, "ATSH7E0")
    assert talk(e, "0122") == "7F 01 12"  # physical: negative response
    talk(e, "ATSH7E1")
    assert talk(e, "0100") == "NO DATA"  # no module there
    talk(e, "ATSH7DF")
    talk(e, "ATCRA7E9")
    assert talk(e, "0100") == "NO DATA"
    talk(e, "ATCRA")
    assert talk(e, "0100").startswith("41 00")


def test_odd_input(car):
    e = ready(car)
    assert talk(e, "010C1").startswith("41 0C")  # response-count digit
    assert talk(e, "01 0c").startswith("41 0C")  # spaces and lower case
    assert talk(e, "") .startswith("41 0C")  # bare CR repeats
    for bad in ("XYZ", "ATMA", "STDI", "ATPPS", "0"):
        assert talk(e, bad) == "?", bad
    assert talk(e, "ATZ") == "ELM327 v1.5"
    assert talk(e, "ATAT1") == "OK" and talk(e, "ATST64") == "OK" and talk(e, "ATCFC1") == "OK"
    (r,) = e.feed(b"AT I\r")  # echo is back on after ATZ
    assert r.echo == "AT I\r"
    assert e.feed(b"\n\x00") == []


def test_mode04_and_trace(car):
    frames = []
    e = ready(car, on_frame=lambda *a: frames.append(a))
    talk(e, "0902")
    dirs = [(d, hex(i), f[0]) for d, i, _, f in frames]
    assert dirs == [("rx", "0x7df", 0x02), ("tx", "0x7e8", 0x10), ("rx", "0x7e0", 0x30), ("tx", "0x7e8", 0x21),
                    ("tx", "0x7e8", 0x22)]
    assert talk(e, "04") == "44" or talk(e, "04") == "7F 04 22"


def test_segment():
    assert segment(b"\x41\x0c\x1a\xf8", 0xAA) == [bytes([4, 0x41, 0x0C, 0x1A, 0xF8, 0xAA, 0xAA, 0xAA])]
    fs = segment(bytes(range(20)), None)
    assert fs[0][:2] == b"\x10\x14" and [f[0] for f in fs[1:]] == [0x21, 0x22] and len(fs[-1]) == 8


class FakeChannel(Channel):
    def __init__(self, chunks, close_at=1.0):
        self.chunks = list(chunks)  # (time, bytes)
        self.close_at = close_at  # the app hangs up
        self.out = b""
        self.t0 = time.monotonic()

    def read(self, timeout):
        if not self.chunks:
            if time.monotonic() - self.t0 >= self.close_at:
                return None
            time.sleep(timeout)
            return b""
        due, data = self.chunks[0]
        now = time.monotonic() - self.t0
        if due <= now + timeout:
            time.sleep(max(0.0, due - now))
            self.chunks.pop(0)
            return data
        time.sleep(timeout)
        return b""

    def write(self, data):
        self.out += data


def test_serve_latency_and_stopped(car):
    e = Elm327(car.link(), latency_s=0.3)
    talk(e, "ATSP6")
    ch = FakeChannel([(0.0, b"ATE0\r"), (0.05, b"010C\r"), (0.1, b"x"), (0.5, b"ATI\r")])
    serve(ch, e, threading.Event())
    out = ch.out.decode()
    assert out == "ATE0\rOK\r\r>STOPPED\r\r>ELM327 v1.5\r\r>"


def test_serve_completes_after_latency(car):
    e = Elm327(car.link(), latency_s=0.1)
    talk(e, "ATSP6")
    ch = FakeChannel([(0.0, b"ATE0\r010C\r")], close_at=0.3)
    t0 = time.monotonic()
    serve(ch, e, threading.Event())
    assert time.monotonic() - t0 >= 0.1
    assert ch.out.decode().startswith("ATE0\rOK\r\r>41 0C ")


# ---------------------------------------------------------------------- manager and transports
def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def tcp_talk(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd + "\r").encode())
    buf = b""
    end = time.monotonic() + 5.0
    while not buf.endswith(b">") and time.monotonic() < end:
        buf += sock.recv(512)
    return buf.decode()


def test_tcp_transport_one_client_at_a_time(car, tmp_path):
    mgr = ElmManager(car.link(), tmp_path / "config.json")
    mgr.update(None, latency_ms=0.0)
    port = free_port()
    mgr.update("tcp", port=port)
    mgr.set_enabled("tcp", True)
    try:
        a = socket.create_connection(("127.0.0.1", port), timeout=5)
        assert "ELM327" in tcp_talk(a, "ATZ")
        assert mgr.status("tcp") == "connected: 127.0.0.1"
        b = socket.create_connection(("127.0.0.1", port), timeout=5)
        assert b.recv(16) == b""  # refused
        assert "already in use" in mgr.note("tcp")
        assert "41 0C" in tcp_talk(a, "010C")
        a.close()
        time.sleep(0.4)
        assert mgr.status("tcp") == "waiting for app"
        c = socket.create_connection(("127.0.0.1", port), timeout=5)
        assert "OK" in tcp_talk(c, "ATE0")  # the adapter keeps its settings across reconnects
        c.close()
        assert any(line.who == "app" and line.text == "010C" for line in mgr.log)
        assert len(mgr.trace) >= 2
    finally:
        mgr.stop_all()
    assert not mgr.is_running("tcp")


@pytest.mark.skipif(sys.platform == "win32", reason="no pseudo-terminals on Windows")
def test_pty_transport(car, tmp_path):
    import serial

    mgr = ElmManager(car.link())
    mgr.update(None, latency_ms=0.0)
    link = tmp_path / "ttyELM"
    mgr.update("pty", link=str(link))
    mgr.set_enabled("pty", True)
    try:
        assert link.is_symlink()
        p = serial.Serial(str(link), 38400, timeout=0.2)
        p.write(b"ATE0\rATSP6\r0105\r")
        buf, end = b"", time.monotonic() + 5
        while buf.count(b">") < 3 and time.monotonic() < end:
            buf += p.read(256)
        assert buf.startswith(b"ATE0\rOK\r\r>OK\r\r>41 05 ")
        p.close()
    finally:
        mgr.stop_all()
    assert not link.exists()


def test_failed_start_is_reported(car, tmp_path):
    mgr = ElmManager(car.link(), tmp_path / "config.json")
    with pytest.raises(Exception):
        mgr.set_enabled("serial", True)  # no port chosen
    assert "no serial port" in mgr.status("serial")
    assert not mgr.is_running("serial")


def test_settings_persist_but_command_line_does_not(car, tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"other": 1}))
    mgr = ElmManager(car.link(), cfg)
    port = free_port()
    mgr.update("tcp", port=port)
    mgr.set_enabled("tcp", True)
    mgr.update(None, identity="ELM327 v2.1")
    mgr.stop_all()
    mgr.save()
    data = json.loads(cfg.read_text())
    assert data["other"] == 1 and data["elm"]["tcp"] == {"enabled": True, "port": port, "network": False}

    mgr2 = ElmManager(car.link(), cfg)
    assert mgr2.settings["identity"] == "ELM327 v2.1" and mgr2.settings["tcp"]["enabled"]
    ap = ArgumentParser()
    cli_args.add_arguments(ap)
    pty = [] if sys.platform == "win32" else ["--elm-pty", str(tmp_path / "tty")]
    args = ap.parse_args(["--no-elm", "--elm-serial", "COM99@9600"] + pty)
    errors = cli_args.apply(mgr2, args)
    assert not mgr2.is_running("tcp")  # --no-elm
    assert any("serial" in e for e in errors)  # COM99 does not exist
    assert mgr2.settings["serial"] == {"enabled": True, "port": "COM99", "baud": 9600}
    mgr2.stop_all()
    mgr2.save()
    saved = json.loads(cfg.read_text())["elm"]
    assert saved["tcp"]["enabled"] and saved["serial"] == {"enabled": False, "port": "", "baud": 38400}

    args = ap.parse_args(["--elm-tcp", "0.0.0.0:" + str(port)])
    mgr3 = ElmManager(car.link(), cfg)
    cli_args.apply(mgr3, args)
    assert mgr3.settings["tcp"]["network"] and mgr3.is_running("tcp")
    mgr3.stop_all()


def test_python_obd_end_to_end(tmp_path):
    obd = pytest.importorskip("obd")
    from ecusim.ui.controller import SimController

    ctl = SimController(ambient_c=20.0)
    try:
        ctl.set_key("on")
        ctl.runner.submit(lambda s: s.request_start())
        time.sleep(2.0)
        port = free_port()
        ctl.elm.update("tcp", port=port)
        ctl.elm.update(None, latency_ms=5.0)
        ctl.elm.set_enabled("tcp", True)
        conn = obd.OBD(f"socket://127.0.0.1:{port}", fast=False, timeout=2)
        try:
            assert conn.status() == obd.OBDStatus.CAR_CONNECTED
            assert conn.protocol_id() == "6"
            assert 500 < conn.query(obd.commands.RPM).value.magnitude < 2000
            assert conn.query(obd.commands.GET_DTC).value == []
            assert 13.0 < conn.query(obd.commands.ELM_VOLTAGE).value.magnitude < 15.0
            assert len(conn.supported_commands) > 50
        finally:
            conn.close()
    finally:
        ctl.shutdown()


# ---------------------------------------------------------------------- permanent /dev name (Linux)
def test_devlink_state_and_commands(tmp_path):
    from ecusim.elm import devlink

    dev = tmp_path / "dev"
    dev.mkdir()
    target = tmp_path / "home" / "ttyELM327"
    assert devlink.state("ttyELM327", target, dev) == "missing"
    (dev / "ttyELM327").symlink_to(target)
    assert devlink.state("ttyELM327", target, dev) == "ok"
    assert devlink.state("ttyELM327", tmp_path / "elsewhere", dev) == "other"
    (dev / "ttyUSB0").write_text("")
    assert devlink.state("ttyUSB0", target, dev) == "taken"

    install, remove = devlink.setup_commands("ttyUSB-ELM327", target)
    assert "sudo" in install and "/dev/ttyUSB-ELM327" in install and str(target) in install
    assert "/dev/ttyUSB-ELM327" in remove
    if devlink.survives_reboot():
        assert "L+ /dev/ttyUSB-ELM327 - - - - " + str(target) in install
        assert "/etc/tmpfiles.d/ecusim-elm327-ttyUSB-ELM327.conf" in install and remove
    for bad in ("", "../etc/passwd", "a b", "x/y", "-rf"):
        assert not devlink.valid_name(bad)
        with pytest.raises(ValueError):
            devlink.setup_commands(bad, target)


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="the /dev name is Linux only")
def test_startup_hint_offers_setup(car, tmp_path, monkeypatch):
    from ecusim.elm import devlink

    mgr = ElmManager(car.link())
    mgr.update("pty", link=str(tmp_path / "tty"), dev_name="ttyTEST-ecusim-not-there")
    mgr.set_enabled("pty", True)
    try:
        lines = cli_args.hints(mgr)
        assert "/dev/ttyTEST-ecusim-not-there" in lines[1] and any("sudo" in x for x in lines)
        monkeypatch.setattr(devlink, "state", lambda *a, **k: "ok")
        assert cli_args.hints(mgr) == ["ELM327 virtual serial port: programs can open /dev/ttyTEST-ecusim-not-there"]
        assert mgr.transports["pty"].path_for_apps == "/dev/ttyTEST-ecusim-not-there"
        assert "programs can open /dev/ttyTEST-ecusim-not-there" in mgr.status("pty")
    finally:
        mgr.stop_all()
