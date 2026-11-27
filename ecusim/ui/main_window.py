"""Main window: cluster on top, driver controls on the left, tabs on the right (DESIGN §9.1)."""

from __future__ import annotations

import time

from PySide6.QtCore import QEvent, QObject, Qt, QTimer
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QComboBox,
    QLabel,
    QLineEdit,
    QMainWindow,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .controller import SimController
from .driver import DriverPanel
from .gauges import Cluster
from .panels import (
    CanPanel,
    ChartsPanel,
    DiagnosticsPanel,
    EnvironmentPanel,
    FaultsPanel,
    LiveDataPanel,
    MonitorsPanel,
    VehicleInfoPanel,
)

UI_HZ = 30
INPUT_HZ = 50

# Keyboard pedal ramps (per second)
RATES = {
    "pedal": (1.0, 2.5),
    "brake": (3.0, 4.0),
    "clutch": (5.0, 0.8),  # quick to press, slow to release so a keyboard launch is possible
}
GEAR_KEYS = {
    Qt.Key.Key_1: 1, Qt.Key.Key_2: 2, Qt.Key.Key_3: 3, Qt.Key.Key_4: 4, Qt.Key.Key_5: 5,
    Qt.Key.Key_R: -1, Qt.Key.Key_N: 0, Qt.Key.Key_0: 0,
}
PEDAL_KEYS = {
    Qt.Key.Key_W: "pedal", Qt.Key.Key_Up: "pedal",
    Qt.Key.Key_S: "brake", Qt.Key.Key_Down: "brake",
    Qt.Key.Key_Space: "clutch", Qt.Key.Key_C: "clutch",
}


class MainWindow(QMainWindow):
    def __init__(self, ctl: SimController):
        super().__init__()
        self.ctl = ctl
        self.setWindowTitle("ECU Simulator — 1.6 DOHC")
        self.resize(1400, 900)

        self.cluster = Cluster()
        self.cluster.setMinimumHeight(300)
        self.driver = DriverPanel()
        self.driver.setMaximumWidth(340)
        self.live = LiveDataPanel(ctl)
        self.charts = ChartsPanel(ctl)
        self.can = CanPanel(ctl)
        self.env = EnvironmentPanel(ctl)
        self.vinfo = VehicleInfoPanel(ctl)
        self.diag = DiagnosticsPanel(ctl)
        self.monitors = MonitorsPanel(ctl)
        self.faults = FaultsPanel(ctl)
        self.tabs = QTabWidget()
        self.tabs.addTab(self.live, "Live data")
        self.tabs.addTab(self.charts, "Charts")
        self.tabs.addTab(self.faults, "Faults")
        self.tabs.addTab(self.diag, "Diagnostics")
        self.tabs.addTab(self.monitors, "Monitors")
        self.tabs.addTab(self.can, "CAN")
        self.tabs.addTab(self.env, "Environment")
        self.tabs.addTab(self.vinfo, "Vehicle info")

        lower = QSplitter()
        lower.addWidget(self.driver)
        lower.addWidget(self.tabs)
        lower.setStretchFactor(1, 1)
        vsplit = QSplitter(Qt.Orientation.Vertical)
        vsplit.addWidget(self.cluster)
        vsplit.addWidget(lower)
        vsplit.setSizes([330, 570])
        central = QWidget()
        lay = QVBoxLayout(central)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(vsplit)
        self.setCentralWidget(central)

        self.status = QLabel()
        self.statusBar().addPermanentWidget(self.status, 1)

        # driver wiring
        d = self.driver
        d.key.key_changed.connect(self._set_key)
        d.gas.value_changed.connect(lambda v: self._pedal_from_mouse("pedal", v))
        d.brake.value_changed.connect(lambda v: self._pedal_from_mouse("brake", v))
        d.clutch.value_changed.connect(lambda v: self._pedal_from_mouse("clutch", v))
        d.shifter.gear_requested.connect(self._shift)
        d.handbrake.toggled.connect(lambda on: ctl.set_driver(handbrake=on))
        d.ac.toggled.connect(lambda on: ctl.set_driver(ac=on))
        d.lights.toggled.connect(lambda on: ctl.set_driver(lights=on))
        self._pedal_widgets = {"pedal": d.gas, "brake": d.brake, "clutch": d.clutch}
        self._values = {"pedal": 0.0, "brake": 0.0, "clutch": 0.0}
        self._sent = dict(self._values)
        self._held: set[str] = set()
        self._key = "off"
        self._refused_until = 0.0
        self._frame = 0

        QApplication.instance().installEventFilter(self)
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh)
        self._ui_timer.start(1000 // UI_HZ)
        self._in_timer = QTimer(self)
        self._in_timer.timeout.connect(self._inputs)
        self._in_timer.start(1000 // INPUT_HZ)

    # ------------------------------------------------------------------ driver actions
    def _set_key(self, key: str) -> None:
        self._key = key
        self.ctl.set_key(key)
        self.driver.key.show_key(key)

    def _shift(self, gear: int) -> None:
        self.ctl.shift(gear)
        self._refused_until = 0.0
        QTimer.singleShot(80, self._check_refused)

    def _check_refused(self) -> None:
        if self.ctl.latest.get("shift_refused"):
            self._refused_until = time.monotonic() + 1.0

    def _pedal_from_mouse(self, name: str, v: float) -> None:
        self._values[name] = v

    # ------------------------------------------------------------------ keyboard
    def _typing(self) -> bool:
        w = QApplication.focusWidget()
        return isinstance(w, (QLineEdit, QAbstractSpinBox)) or (isinstance(w, QComboBox) and w.isEditable())

    def eventFilter(self, obj: QObject, ev: QEvent) -> bool:  # noqa: N802
        if ev.type() not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease) or self._typing():
            return False
        if not self.isActiveWindow():
            return False
        press = ev.type() == QEvent.Type.KeyPress
        if ev.isAutoRepeat():
            return ev.key() in PEDAL_KEYS or ev.key() == Qt.Key.Key_K
        k = ev.key()
        if k in PEDAL_KEYS:
            name = PEDAL_KEYS[k]
            (self._held.add if press else self._held.discard)(name)
            return True
        if k == Qt.Key.Key_K:
            self._set_key("start" if press else "on")
            return True
        if not press:
            return False
        if k in GEAR_KEYS:
            self._shift(GEAR_KEYS[k])
            return True
        if k == Qt.Key.Key_H:
            self.driver.handbrake.toggle()
            return True
        if k == Qt.Key.Key_L:
            self.driver.lights.toggle()
            return True
        if k in (Qt.Key.Key_BracketLeft, Qt.Key.Key_BracketRight):
            step = 5 if k == Qt.Key.Key_BracketRight else -5
            self.driver.hand.setValue(self.driver.hand.value() + step)
            return True
        if k == Qt.Key.Key_G:
            self.driver.hand.setValue(0)
            return True
        return False

    def _inputs(self) -> None:
        dt = 1.0 / INPUT_HZ
        for name, w in self._pedal_widgets.items():
            up, down = RATES[name]
            if name in self._held:
                self._values[name] = min(1.0, self._values[name] + up * dt)
                w.set_value(self._values[name], emit=False)
            elif not w._dragging and not w.hold and self._values[name] > 0.0:
                v = self._values[name] - down * dt
                self._values[name] = v if v > 1e-3 else 0.0  # land exactly on zero
                w.set_value(self._values[name], emit=False)
        # Hand throttle sets a floor under the gas pedal
        hand = self.driver.hand.value() / 100.0
        effective = dict(self._values, pedal=max(self._values["pedal"], hand))
        gas = self._pedal_widgets["pedal"]
        if not gas._dragging and abs(gas.value - effective["pedal"]) > 1e-4:
            gas.set_value(effective["pedal"], emit=False)
        changed = {k: v for k, v in effective.items()
                   if abs(v - self._sent[k]) > 1e-4 or (v == 0.0 and self._sent[k] != 0.0)}
        if changed:
            self.ctl.set_driver(**changed)
            self._sent.update(changed)

    # ------------------------------------------------------------------ display
    def _refresh(self) -> None:
        s = self.ctl.latest
        sim = self.ctl.current_sim
        ecu_mode = int(s["mode"])
        self.cluster.update_from(s, int(s["mil"]), ecu_mode)
        self.cluster.tick()
        sh = self.driver.shifter
        sh.gear = int(s["gear"])
        sh.refused = time.monotonic() < self._refused_until
        sh.update()
        if self._key != "start":
            key_now = {0: "off", 1: "acc", 2: "on", 3: "start"}[int(s["key"])]
            if key_now != self._key and key_now != "start":
                self._key = key_now
                self.driver.key.show_key(key_now)
        self._frame += 1
        if self._frame % 3 == 0:
            self.charts.record(s)
        current = self.tabs.currentWidget()
        if self._frame % 6 == 0:
            if current is self.live:
                self.live.refresh()
            elif current is self.charts:
                self.charts.refresh()
            elif current is self.can:
                self.can.refresh()
            elif current is self.env:
                self.env.refresh()
            elif current is self.diag:
                self.diag.refresh()
            elif current is self.monitors:
                self.monitors.refresh()
            elif current is self.faults:
                self.faults.refresh()
            mode = ["OFF", "KEY ON", "CRANKING", "RUNNING"][ecu_mode]
            can = self.ctl.can_cfg
            can_txt = f"CAN {can.interface}:{can.channel}" if self.ctl.can is not None and can else "CAN off"
            fs = {0: "—", 1: "open loop (cold)", 2: "closed loop", 4: "open loop (drive)", 8: "open loop (fault)",
                  16: "closed loop (fault)"}.get(int(s["fuel_status"]), "?")
            self.status.setText(
                f"ECU: {mode}  ·  fuel: {fs}  ·  sim t={s['t']:.1f} s  ×{self.ctl.runner.time_scale:g}  ·  "
                f"{can_txt}  ·  overruns {self.ctl.runner.overruns}  ·  ambient {sim.env.ambient_c:.0f} °C"
            )

    def closeEvent(self, ev) -> None:  # noqa: N802
        self._ui_timer.stop()
        self._in_timer.stop()
        self.ctl.shutdown()
        super().closeEvent(ev)
