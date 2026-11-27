#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Regenerate the screenshots used by docs/USAGE.md (runs the real window offscreen).

    QT_QPA_PLATFORM=offscreen .venv/bin/python tools/make_doc_screenshots.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PySide6.QtWidgets import QApplication  # noqa: E402

from ecusim.driving import ScriptedDriver  # noqa: E402
from ecusim.faults.manager import FaultSetting  # noqa: E402
from ecusim.obd.can_transport import CanConfig  # noqa: E402
from ecusim.obd.client import ObdClient  # noqa: E402
from ecusim.ui.controller import SimController  # noqa: E402
from ecusim.ui.main_window import MainWindow  # noqa: E402

OUT = ROOT / "docs" / "images"
app = QApplication.instance() or QApplication([])


def pump(seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.005)


class Session:
    """A fresh controller + window for one scenario."""

    def __init__(self, ambient: float = 20.0):
        self.ctl = SimController(ambient_c=ambient)
        self.win = MainWindow(self.ctl)
        self.win.resize(1400, 900)
        self.win.isActiveWindow = lambda: True
        self.win.show()
        pump(0.3)

    def drive(self, script, record_every: float = 0.1, timeout: float = 900.0) -> None:
        """Run ``script(sim, driver)`` on the simulation thread and load its history into the charts."""

        def job(sim):
            d = ScriptedDriver(sim, every_s=record_every)
            script(sim, d)
            return d.rec.rows

        rows = self.ctl.call(job, timeout=timeout)
        buf = self.win.charts.buf
        buf.clear()
        buf.extend(rows[-buf.maxlen:])
        pump(0.5)

    def charts(self, *presets: str) -> None:
        for combo, name in zip(self.win.charts.combos, presets):
            combo.setCurrentText(name)
        self.win.tabs.setCurrentWidget(self.win.charts)
        self.win.charts.refresh()
        pump(0.4)

    def tab(self, widget) -> None:
        self.win.tabs.setCurrentWidget(widget)
        pump(1.2)

    def shot(self, name: str, widget=None) -> None:
        pump(0.6)
        (widget or self.win).grab().save(str(OUT / name))
        print("wrote", name)

    def close(self) -> None:
        self.win.close()
        pump(0.2)


def warm(sim) -> None:
    sim.plant.set_warm()
    sim.sensors = type(sim.sensors)(sim.rng)
    sim.readings = sim.sensors.update(sim.dt, sim.plant, sim.ecu.act, sim.driver)


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)

    # 1. Overview: warm engine idling, live data
    s = Session()
    s.drive(lambda sim, d: (warm(sim), sim.start_engine(), d.wait(40)))
    s.tab(s.win.live)
    s.shot("overview.png")
    s.shot("driver_panel.png", s.win.driver)
    s.shot("cluster.png", s.win.cluster)
    s.close()

    # 2. Cold start at 10 °C: 10 minutes of idle, open -> closed loop, warm-up
    s = Session(ambient=10.0)
    s.drive(lambda sim, d: (sim.start_engine(), d.wait(600)), record_every=0.5)
    s.charts("Engine speed", "Temperatures", "Lambda", "Fuel trims")
    s.shot("cold_start_charts.png")
    s.close()

    # 3. Fuel trims: vacuum leak, idle then cruise
    s = Session()

    def leak(sim, d):
        warm(sim)
        sim.start_engine()
        d.wait(30)
        sim.faults.set(FaultSetting("vacuum_leak"))
        d.wait(150)

    s.ctl.fault_settings["vacuum_leak"] = FaultSetting("vacuum_leak")
    s.drive(leak, record_every=0.2)
    s.charts("Fuel trims", "MAP / MAF", "Lambda", "Engine speed")
    s.shot("vacuum_leak_charts.png")
    s.tab(s.win.live)
    s.shot("vacuum_leak_live.png")
    s.tab(s.win.faults)
    s.win.faults.tree.setCurrentItem(s.win.faults._items["vacuum_leak"])
    s.shot("faults_tab.png")
    s.close()

    # 4. Misfire: worn plug on cylinder 3, two trips under load
    s = Session()
    s.ctl.fault_settings["plug"] = FaultSetting("plug", cylinder=3, value=85)

    def misfire(sim, d):
        warm(sim)
        sim.faults.set(FaultSetting("plug", cylinder=3, value=85))
        for _trip in range(2):
            sim.start_engine()
            d.wait(20)
            d.launch(1, 0.4, 1.2)
            d.accelerate_until(0.8, rpm=3500)
            d.shift(2, pedal_after=0.8)
            d.accelerate_until(0.8, rpm=4000)
            d.shift(3, pedal_after=0.7)
            d.hold_speed(70, 90)
            d.brake_to_stop()
            d.wait(5)
            sim.driver.key = "off"
            d.wait(1)
        sim.driver.key = "on"
        d.wait(1)

    s.drive(misfire, record_every=0.2)
    s.tab(s.win.diag)
    m06 = s.win.diag.m06
    rows = [r for r in range(m06.rowCount()) if m06.item(r, 0) and m06.item(r, 0).text() == "A4"]
    if rows:
        m06.scrollToItem(m06.item(rows[0], 0), m06.ScrollHint.PositionAtTop)
        m06.selectRow(rows[0])
    s.shot("misfire_diagnostics.png")
    s.close()

    # 5. Catalyst: healthy vs worn, steady 70 km/h
    for label, fault in (("healthy", None), ("worn", FaultSetting("catalyst"))):
        s = Session()

        def cat(sim, d, fault=fault):
            warm(sim)
            if fault:
                sim.faults.set(fault)
            sim.start_engine()
            d.wait(20)
            d.launch(1, 0.3, 1.5)
            d.accelerate_until(0.5, rpm=3000)
            for g in (2, 3, 4):
                d.shift(g, pedal_after=0.4)
                d.accelerate_until(0.5, rpm=3000 if g < 4 else None, kmh=70 if g == 4 else None, timeout=20)
            d.hold_speed(70, 60)

        s.drive(cat, record_every=0.05)
        s.charts("Lambda", "Downstream O2", "Vehicle speed", "Fuel trims")
        s.shot(f"catalyst_{label}.png", s.win.charts)
        s.close()

    # 6. Readiness after clearing codes
    s = Session()
    s.drive(lambda sim, d: (warm(sim), sim.start_engine(), d.wait(30)))
    s.ctl.clear_codes()
    s.tab(s.win.monitors)
    s.shot("monitors_tab.png")
    s.close()

    # 7. Broken coolant sensor: scan tool reads -40 °C, ECU uses its model, fan forced on
    s = Session(ambient=15.0)
    s.ctl.fault_settings["s_ect"] = FaultSetting("s_ect", mode="open")
    s.drive(lambda sim, d: (sim.faults.set(FaultSetting("s_ect", mode="open")), sim.start_engine(), d.wait(60)))
    s.tab(s.win.live)
    s.shot("ect_open_live.png")
    s.close()

    # 8. Limp home: throttle sensors disagree
    s = Session()
    s.ctl.fault_settings["s_tps_b"] = FaultSetting("s_tps_b", mode="offset", amount=15.0)
    s.drive(lambda sim, d: (warm(sim), sim.faults.set(FaultSetting("s_tps_b", mode="offset", amount=15.0)),
                            sim.start_engine(), d.wait(5), d.ramp("pedal", 1.0, 0.5), d.wait(5)))
    s.shot("limp_cluster.png", s.win.cluster)
    s.close()

    # 9. CAN tab with a scan tool talking to it (in-process virtual bus)
    s = Session()
    s.ctl.connect_can(CanConfig("virtual", "docs"))
    s.win.can.sync_from_controller()
    s.drive(lambda sim, d: (warm(sim), sim.start_engine(), d.wait(10)))
    with ObdClient(CanConfig("virtual", "docs")) as c:
        c.read_pids([0x0C, 0x05, 0x0B])
        c.vin()
        c.dtcs(0x03)
    s.tab(s.win.can)
    s.shot("can_tab.png")
    s.ctl.disconnect_can()
    s.close()

    # 10. ELM327 tab: a phone-style app connected over TCP
    s = Session()
    s.ctl.set_key("on")
    s.ctl.runner.submit(lambda sim: sim.request_start())
    pump(3.0)
    elm = s.win.elm
    elm.switch["tcp"].setChecked(True)
    import socket

    with socket.create_connection(("127.0.0.1", s.ctl.elm.settings["tcp"]["port"]), timeout=5) as c:
        for cmd in ("ATZ", "ATE0", "ATL0", "ATH0", "ATSP0", "0100", "010C", "0105", "0902", "03", "ATRV"):
            c.sendall((cmd + "\r").encode())
            buf = b""
            while not buf.endswith(b">"):
                buf += c.recv(512)
        s.tab(elm)
        elm.refresh()
        pump(0.5)
        s.shot("elm327_tab.png")
    s.ctl.elm.stop_all()
    s.close()

    # 11. Environment tab
    s = Session()
    s.tab(s.win.env)
    s.shot("environment_tab.png")
    s.close()
    return 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)  # skip PySide6 interpreter-exit teardown (see ecusim/ui/app.py)
