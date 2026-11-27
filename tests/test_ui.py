# SPDX-License-Identifier: GPL-3.0-or-later
"""UI smoke tests (offscreen). Skipped when PySide6 is not installed."""

import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ecusim.ui.controller import SimController  # noqa: E402
from ecusim.ui.gauges import Cluster, coolant_fraction  # noqa: E402
from ecusim.ui.main_window import MainWindow  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def win(app):
    ctl = SimController(ambient_c=15.0)
    w = MainWindow(ctl)
    w.isActiveWindow = lambda: True  # offscreen windows are never "active"
    w.show()
    yield w
    w.close()


def pump(app, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.005)


def wait_for(app, pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        pump(app, 0.02)
    return False


def test_keyboard_start_and_drive_away(app, win):
    ctl = win.ctl
    QTest.keyPress(win, Qt.Key.Key_K)
    assert wait_for(app, lambda: ctl.latest["mode"] == 3, 5.0)
    QTest.keyRelease(win, Qt.Key.Key_K)
    pump(app, 2.0)
    assert ctl.latest["key"] == 2.0

    QTest.keyClick(win, Qt.Key.Key_1)
    assert wait_for(app, lambda: ctl.latest["shift_refused"] == 1.0, 1.0)  # clutch not pressed
    assert ctl.latest["gear"] == 0.0

    QTest.keyPress(win, Qt.Key.Key_Space)
    assert wait_for(app, lambda: ctl.latest["clutch"] > 90.0, 1.0)
    QTest.keyClick(win, Qt.Key.Key_1)
    assert wait_for(app, lambda: ctl.latest["gear"] == 1.0, 1.0)
    win.driver.gas.hold = True
    win.driver.gas.set_value(0.25)
    QTest.keyRelease(win, Qt.Key.Key_Space)  # slow keyboard release ramp
    assert wait_for(app, lambda: ctl.latest["kmh"] > 8.0, 6.0)
    assert ctl.latest["mode"] == 3  # did not stall


def test_tabs_refresh_without_errors(app, win):
    for tab in (win.live, win.charts, win.can, win.env, win.vinfo):
        win.tabs.setCurrentWidget(tab)
        pump(app, 0.3)
    assert win.live.obd.rowCount() > 40


def test_bulb_check_lamps(app):
    c = Cluster()
    s = {"key": 2.0, "rpm": 0.0, "kmh": 0.0, "ect_c": 15.0, "fuel_pct": 60.0, "gear": 0.0, "odometer_km": 0.0,
         "trip_km": 0.0, "ambient_c": 15.0, "shift_refused": 0.0, "starter": 0.0, "alt_a": 0.0, "handbrake": 1.0,
         "fan": 0.0, "lights": 0.0}
    c.update_from(s, 0, 1)
    lit = {k for k, lamp in c.lamps.items() if lamp.state}
    assert {"CHECK", "EPC", "BATT", "OIL", "BRAKE"} <= lit


def test_coolant_needle_parks_mid_scale():
    assert coolant_fraction(15.0) == 0.0
    assert coolant_fraction(85.0) == pytest.approx(0.5)
    assert coolant_fraction(100.0) == pytest.approx(0.5)
    assert coolant_fraction(125.0) == pytest.approx(1.0)


def test_controller_resets_keep_odometer_and_trims(app):
    ctl = SimController(ambient_c=20.0)
    try:
        ctl.runner.submit(lambda s: (setattr(s.plant.vehicle, "odometer_m", 12345.0),
                                     s.ecu.fuel.ltft.__setitem__(0, 0.04)))
        pump(app, 0.1)
        ctl.warm_restart()
        pump(app, 0.2)
        sim = ctl.current_sim
        assert sim.plant.vehicle.odometer_m == pytest.approx(12345.0, abs=1.0)
        assert sim.ecu.fuel.ltft[0] == pytest.approx(0.04)
        assert sim.plant.thermal.coolant_c > 80.0
        ctl.reset_cold(-5.0)
        pump(app, 0.2)
        assert ctl.current_sim.plant.thermal.coolant_c == pytest.approx(-5.0)
    finally:
        ctl.shutdown()


def test_trip_and_odometer_reset(app, win):
    ctl = win.ctl
    ctl.runner.submit(lambda s: (setattr(s.plant.vehicle, "odometer_m", 12345.0),
                                 setattr(s.plant.vehicle, "trip_m", 678.0)))
    pump(app, 0.1)
    ctl.warm_restart()  # the trip survives a key cycle, like the odometer
    pump(app, 0.2)
    assert ctl.current_sim.plant.vehicle.trip_m == pytest.approx(678.0, abs=1.0)
    info = win.cluster.info
    QTest.mouseClick(info, Qt.MouseButton.LeftButton, pos=info.rect().center())
    assert wait_for(app, lambda: ctl.current_sim.plant.vehicle.trip_m == 0.0, 1.0)
    assert ctl.current_sim.plant.vehicle.odometer_m == pytest.approx(12345.0, abs=1.0)
    ctl.reset_odometer()
    assert wait_for(app, lambda: ctl.current_sim.plant.vehicle.odometer_m == 0.0, 1.0)


def test_elm_tab_switches_tcp(app, win):
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    panel = win.elm
    win.tabs.setCurrentWidget(panel)
    panel.tcp_port.setValue(port)
    panel.switch["tcp"].setChecked(True)
    assert win.ctl.elm.is_running("tcp") and not panel.tcp_port.isEnabled()
    c = socket.create_connection(("127.0.0.1", port), timeout=5)
    c.sendall(b"ATI\r")
    buf = b""
    while not buf.endswith(b">"):
        buf += c.recv(256)
    assert b"ELM327" in buf
    pump(app, 0.3)
    assert "connected" in panel.state["tcp"].text()
    c.close()
    panel.switch["tcp"].setChecked(False)
    assert not win.ctl.elm.is_running("tcp") and panel.tcp_port.isEnabled()
    panel.switch["serial"].setChecked(True)  # no port chosen: refused, switch goes back off
    assert not panel.switch["serial"].isChecked() and "no serial port" in panel.state["serial"].text()


def test_hand_throttle_holds_gas(app, win):
    ctl = win.ctl
    sim_pedal = lambda: ctl.call(lambda s: s.driver.pedal)  # noqa: E731
    win.driver.hand.setValue(30)
    assert wait_for(app, lambda: abs(sim_pedal() - 0.30) < 1e-3, 1.0)
    assert win.driver.gas.value == pytest.approx(0.30)
    QTest.keyPress(win, Qt.Key.Key_W)  # pedal adds on top of the hand throttle
    assert wait_for(app, lambda: sim_pedal() > 0.5, 1.5)
    QTest.keyRelease(win, Qt.Key.Key_W)
    assert wait_for(app, lambda: abs(sim_pedal() - 0.30) < 1e-3, 2.0)  # back to the held level, not zero
    QTest.keyClick(win, Qt.Key.Key_BracketRight)
    assert win.driver.hand.value() == 35
    QTest.keyClick(win, Qt.Key.Key_G)
    assert win.driver.hand.value() == 0
    assert wait_for(app, lambda: sim_pedal() == 0.0, 1.0)  # exactly zero, not a float residue
