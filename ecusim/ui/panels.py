"""Side panels: live data, charts, CAN, environment, vehicle info (DESIGN §9.4)."""

from __future__ import annotations

import sys
from collections import deque

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..obd.can_transport import CanConfig
from ..obd.pids import MODE01
from ..obd.vehicle_info import OBD_STANDARD, vin_is_valid, with_check_digit
from .controller import SimController

MONO = QFont("monospace")
MONO.setStyleHint(QFont.StyleHint.Monospace)


def _fmt(v: object) -> str:
    if isinstance(v, float):
        return f"{v:.2f}"
    if isinstance(v, tuple):
        return ", ".join("—" if x is None else (f"{x:.3f}" if isinstance(x, float) else str(x)) for x in v)
    return str(v)


def _item(text: str, mono: bool = False, align_right: bool = False) -> QTableWidgetItem:
    it = QTableWidgetItem(text)
    it.setFlags(it.flags() & ~Qt.ItemFlag.ItemIsEditable)
    if mono:
        it.setFont(MONO)
    if align_right:
        it.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    return it


# ====================================================================== live data
INTERNAL = [
    # key, label, unit, is_truth
    ("rpm_target", "Idle target", "rpm", False),
    ("pw_ms", "Injector pulse width", "ms", False),
    ("spark_deg", "Spark advance", "° BTDC", False),
    ("lambda_cmd", "Commanded lambda", "λ", False),
    ("lambda_wb", "Wideband lambda (sensor)", "λ", False),
    ("stft_pct", "STFT", "%", False),
    ("ltft_pct", "LTFT (active cell)", "%", False),
    ("dtrim", "Downstream trim", "Δλ", False),
    ("cat_heat", "Catalyst heating", "0-1", False),
    ("cat_est_c", "Catalyst temp (ECU model)", "°C", False),
    ("vvt_deg", "Intake cam advance", "°", False),
    ("throttle_pct", "Throttle opening", "%", False),
    ("calc_load_pct", "Calculated load", "%", False),
    ("clutch_torque", "Clutch torque", "Nm", False),
    ("gear", "Gear", "", False),
    ("kmh", "Vehicle speed", "km/h", False),
    ("fuel_l", "Fuel in tank", "L", False),
    ("maf_true_gps", "Air through throttle (true)", "g/s", True),
    ("cyl_flow_gps", "Air into cylinders (true)", "g/s", True),
    ("coolant_true_c", "Coolant (true)", "°C", True),
    ("lambda_true", "Exhaust lambda (true)", "λ", True),
    ("lambda_comb", "Combustion lambda (true)", "λ", True),
    ("mbt_deg", "MBT spark (true)", "°", True),
    ("cat_c", "Catalyst front temp (true)", "°C", True),
    ("egt_c", "Exhaust gas temp (true)", "°C", True),
    ("osc_state", "Catalyst O2 storage (true)", "0-1", True),
    ("torque_ind", "Indicated torque (true)", "Nm", True),
    ("torque_brake", "Brake torque (true)", "Nm", True),
]


class LiveDataPanel(QWidget):
    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        self.pids = sorted(MODE01)
        self.obd = QTableWidget(len(self.pids), 5)
        self.obd.setHorizontalHeaderLabels(["PID", "Name", "Value", "Unit", "Raw"])
        for row, pid in enumerate(self.pids):
            self.obd.setItem(row, 0, _item(f"{pid:02X}", mono=True))
            self.obd.setItem(row, 1, _item(MODE01[pid].name))
            self.obd.setItem(row, 3, _item(MODE01[pid].unit))
        self.obd.verticalHeader().setVisible(False)
        self.obd.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)

        self.truth = QCheckBox("Show plant truth (what the ECU can't see)")
        self.internal = QTableWidget(len(INTERNAL), 3)
        self.internal.setHorizontalHeaderLabels(["Signal", "Value", "Unit"])
        self.internal.verticalHeader().setVisible(False)
        self.internal.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for row, (_k, label, unit, is_truth) in enumerate(INTERNAL):
            it = _item(label)
            if is_truth:
                it.setForeground(QColor(230, 150, 40))
            self.internal.setItem(row, 0, it)
            self.internal.setItem(row, 2, _item(unit))
        self.truth.toggled.connect(self._apply_truth)
        self._apply_truth(False)

        left = QGroupBox("Generic OBD-II, Mode 01 (as a scan tool sees it)")
        ll = QVBoxLayout(left)
        ll.addWidget(self.obd)
        right = QGroupBox("ECU internals")
        rl = QVBoxLayout(right)
        rl.addWidget(self.truth)
        rl.addWidget(self.internal)
        split = QSplitter()
        split.addWidget(left)
        split.addWidget(right)
        split.setSizes([620, 420])
        lay = QVBoxLayout(self)
        lay.addWidget(split)

    def _apply_truth(self, on: bool) -> None:
        for row, (*_, is_truth) in enumerate(INTERNAL):
            self.internal.setRowHidden(row, is_truth and not on)

    def refresh(self) -> None:
        d = dict(self.ctl.latest_diag, obd_standard=self.ctl.info.obd_standard)
        powered = d.get("powered", False)
        for row, pid in enumerate(self.pids):
            if powered:
                raw = MODE01[pid].encode(d)
                self.obd.setItem(row, 2, _item(_fmt(MODE01[pid].decode(raw)), align_right=True))
                self.obd.setItem(row, 4, _item(raw.hex(" ").upper(), mono=True))
            else:
                self.obd.setItem(row, 2, _item("no response", align_right=True))
                self.obd.setItem(row, 4, _item(""))
        s = self.ctl.latest
        for row, (k, *_rest) in enumerate(INTERNAL):
            self.internal.setItem(row, 1, _item(_fmt(float(s.get(k, 0.0))), align_right=True))


# ====================================================================== charts
CHART_PRESETS = {
    "Engine speed": ["rpm", "rpm_target"],
    "Vehicle speed": ["kmh"],
    "Pedals": ["pedal", "clutch", "brake", "throttle_pct"],
    "Temperatures": ["ect_c", "oil_c", "iat_c"],
    "Catalyst": ["cat_c", "cat_est_c"],
    "MAP / MAF": ["map_kpa", "maf_gps"],
    "Lambda": ["lambda_cmd", "lambda_wb"],
    "Fuel trims": ["stft_pct", "ltft_pct"],
    "Spark": ["spark_deg", "mbt_deg"],
    "Injector PW": ["pw_ms"],
    "Downstream O2": ["nb_volts"],
    "Load": ["calc_load_pct", "abs_load_pct"],
    "Battery": ["vbatt"],
}
CHART_YRANGE = {"Downstream O2": (0.0, 1.0)}  # fixed scales where autoscaling would exaggerate ripple
COLORS = ["#ff8c28", "#4aa3ff", "#5ad17a", "#e05555", "#c08cff"]


class ChartsPanel(QWidget):
    WINDOW_S = 120.0

    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        self.buf: deque[dict] = deque(maxlen=int(self.WINDOW_S * 10))
        pg.setConfigOptions(antialias=True)
        self.glw = pg.GraphicsLayoutWidget()
        self.plots: list[pg.PlotItem] = []
        self.combos: list[QComboBox] = []
        bar = QHBoxLayout()
        defaults = ["Engine speed", "Vehicle speed", "MAP / MAF", "Lambda"]
        for i, name in enumerate(defaults):
            cb = QComboBox()
            cb.addItems(list(CHART_PRESETS))
            cb.setCurrentText(name)
            cb.currentTextChanged.connect(lambda _t, i=i: self._rebuild(i))
            bar.addWidget(QLabel(f"{i + 1}:"))
            bar.addWidget(cb)
            self.combos.append(cb)
            plot = self.glw.addPlot(row=i, col=0)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLegend(offset=(5, 5))
            if i:
                plot.setXLink(self.plots[0])
            self.plots.append(plot)
        self.pause = QCheckBox("Pause")
        bar.addStretch(1)
        bar.addWidget(self.pause)
        self.curves: list[list[tuple[str, pg.PlotDataItem]]] = [[] for _ in self.plots]
        for i in range(len(self.plots)):
            self._rebuild(i)
        lay = QVBoxLayout(self)
        lay.addLayout(bar)
        lay.addWidget(self.glw)

    def _rebuild(self, i: int) -> None:
        plot = self.plots[i]
        plot.clear()
        if plot.legend is not None:
            plot.legend.clear()
        name = self.combos[i].currentText()
        if name in CHART_YRANGE:
            plot.setYRange(*CHART_YRANGE[name], padding=0.02)
        else:
            plot.enableAutoRange(axis="y")
        keys = CHART_PRESETS[name]
        self.curves[i] = [(k, plot.plot(name=k, pen=pg.mkPen(COLORS[j % len(COLORS)], width=1.5)))
                          for j, k in enumerate(keys)]

    def record(self, s: dict) -> None:
        self.buf.append(s)

    def refresh(self) -> None:
        if self.pause.isChecked() or not self.buf:
            return
        t_end = self.buf[-1]["t"]
        t = [r["t"] - t_end for r in self.buf]
        for curves in self.curves:
            for k, curve in curves:
                curve.setData(t, [r.get(k, 0.0) for r in self.buf])


# ====================================================================== CAN
DEFAULT_CHANNELS = {
    "udp_multicast": "239.74.163.2",
    "virtual": "ecusim",
    "socketcan": "vcan0",
    "slcan": "COM3" if sys.platform == "win32" else "/dev/ttyACM0",
    "pcan": "PCAN_USBBUS1",
    "kvaser": "0",
    "gs_usb": "0",
}


def describe_frame(data: bytes) -> str:
    if not data:
        return ""
    kind = data[0] >> 4
    if kind == 0:
        n = data[0] & 0x0F
        payload = data[1 : 1 + n]
        return "SF " + payload.hex(" ").upper()
    if kind == 1:
        n = ((data[0] & 0x0F) << 8) | data[1]
        return f"FF len={n} " + data[2:].hex(" ").upper()
    if kind == 2:
        return f"CF sn={data[0] & 0x0F}"
    if kind == 3:
        return f"FC fs={data[0] & 0x0F} bs={data[1] if len(data) > 1 else 0} st={data[2] if len(data) > 2 else 0}"
    return "?"


class CanPanel(QWidget):
    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        self.iface = QComboBox()
        self.iface.setEditable(True)
        self.iface.addItems(list(DEFAULT_CHANNELS))
        self.channel = QLineEdit(DEFAULT_CHANNELS["udp_multicast"])
        self.iface.currentTextChanged.connect(lambda t: self.channel.setText(DEFAULT_CHANNELS.get(t, "")))
        self.bitrate = QComboBox()
        self.bitrate.addItems(["500000", "250000", "125000", "1000000"])
        self.tty = QComboBox()
        self.tty.setEditable(True)
        self.tty.addItems(["115200", "230400", "460800", "921600", "1000000", "2000000", "3000000"])
        self.tty.setToolTip("Only for slcan adapters on a serial port (ttyUSB / COM). "
                            "USB-CDC devices (ttyACM, CANable) ignore it.")
        self.tty_lbl = QLabel("Serial baud rate")
        self.iface.currentTextChanged.connect(self._tty_visible)
        self.ext = QCheckBox("29-bit identifiers")
        self.padding = QComboBox()
        self.padding.addItems(["AA", "00", "55", "CC", "none"])
        self.delay = QDoubleSpinBox()
        self.delay.setRange(0.0, 200.0)
        self.delay.setSuffix(" ms")
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle)
        self.status = QLabel("Disconnected")
        self.status.setWordWrap(True)
        self.stats = QLabel("")
        form = QFormLayout()
        form.addRow("Interface", self.iface)
        form.addRow("Channel", self.channel)
        form.addRow("Bitrate", self.bitrate)
        form.addRow(self.tty_lbl, self.tty)
        form.addRow("", self.ext)
        form.addRow("Padding byte", self.padding)
        form.addRow("Extra response delay", self.delay)
        form.addRow(self.connect_btn, self.status)
        form.addRow("Statistics", self.stats)
        settings = QGroupBox("CAN link")
        settings.setLayout(form)

        self.trace = QTableWidget(0, 5)
        self.trace.setHorizontalHeaderLabels(["Time", "Dir", "ID", "Data", "ISO-TP"])
        self.trace.verticalHeader().setVisible(False)
        hdr = self.trace.horizontalHeader()
        for col in range(4):
            hdr.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)
        hdr.setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.trace_pause = QCheckBox("Pause trace")
        tb = QHBoxLayout()
        tb.addWidget(self.trace_pause)
        tb.addStretch(1)
        tbox = QGroupBox("Bus trace (last 200 frames)")
        tl = QVBoxLayout(tbox)
        tl.addLayout(tb)
        tl.addWidget(self.trace)

        lay = QHBoxLayout(self)
        lay.addWidget(settings, 2)
        lay.addWidget(tbox, 3)
        self._tty_visible(self.iface.currentText())
        self.sync_from_controller()

    def _tty_visible(self, iface: str) -> None:
        on = iface.strip() in ("slcan", "serial", "robotell", "seeedstudio")
        self.tty.setVisible(on)
        self.tty_lbl.setVisible(on)

    def config(self) -> CanConfig:
        pad = self.padding.currentText()
        return CanConfig(
            interface=self.iface.currentText().strip(),
            channel=self.channel.text().strip(),
            bitrate=int(self.bitrate.currentText()),
            extended=self.ext.isChecked(),
            padding=None if pad == "none" else int(pad, 16),
            response_delay_ms=self.delay.value(),
            tty_baudrate=int(self.tty.currentText()) if self.tty.isVisible() and self.tty.currentText().isdigit()
            else None,
        )

    def sync_from_controller(self) -> None:
        c = self.ctl.can_cfg
        if self.ctl.can is not None and c is not None:
            self.iface.setCurrentText(c.interface)
            self.channel.setText(c.channel)
            a = c.addr
            self.status.setText(f"<b style='color:#3c3'>Connected</b> {c.interface}:{c.channel} — "
                                f"request 0x{a['request']:X}/0x{a['functional']:X}, response 0x{a['response']:X}")
            self.connect_btn.setText("Disconnect")
        else:
            msg = self.ctl.can_error or "Disconnected"
            self.status.setText(msg)
            self.connect_btn.setText("Connect")

    def _toggle(self) -> None:
        if self.ctl.can is not None:
            self.ctl.disconnect_can()
        else:
            try:
                self.ctl.connect_can(self.config())
            except Exception:  # error text kept on the controller
                pass
        self.sync_from_controller()

    def refresh(self) -> None:
        srv = self.ctl.can
        if srv is None:
            self.stats.setText("")
            return
        st = srv.stats
        self.stats.setText(f"rx {st['rx']}  tx {st['tx']}  requests {st['requests']}  "
                           f"responses {st['responses']}  errors {st['errors']}")
        if self.trace_pause.isChecked():
            return
        rows = list(srv.trace)[-200:]
        self.trace.setRowCount(len(rows))
        t0 = rows[0].t if rows else 0.0
        for i, e in enumerate(rows):
            self.trace.setItem(i, 0, _item(f"{e.t - t0:9.3f}", mono=True))
            self.trace.setItem(i, 1, _item(e.direction.upper()))
            self.trace.setItem(i, 2, _item(f"{e.arb_id:X}", mono=True))
            self.trace.setItem(i, 3, _item(e.data.hex(" ").upper(), mono=True))
            self.trace.setItem(i, 4, _item(describe_frame(e.data), mono=True))
        self.trace.scrollToBottom()


# ====================================================================== environment
class EnvironmentPanel(QWidget):
    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        sim = ctl.current_sim
        self.ambient = QDoubleSpinBox()
        self.ambient.setRange(-30.0, 45.0)
        self.ambient.setSuffix(" °C")
        self.ambient.setValue(sim.env.ambient_c)
        self.ambient.valueChanged.connect(ctl.set_ambient)
        self.baro = QDoubleSpinBox()
        self.baro.setRange(60.0, 105.0)
        self.baro.setSuffix(" kPa")
        self.baro.setValue(sim.env.baro_kpa)
        self.baro.valueChanged.connect(ctl.set_baro)
        self.grade = QSlider(Qt.Orientation.Horizontal)
        self.grade.setRange(-10, 10)
        self.grade_lbl = QLabel("0 %")
        self.grade.valueChanged.connect(lambda v: (ctl.set_grade(float(v)), self.grade_lbl.setText(f"{v:+d} %")))
        self.fuel = QSlider(Qt.Orientation.Horizontal)
        self.fuel.setRange(0, 100)
        self.fuel.setValue(60)
        self.fuel_lbl = QLabel("60 %")
        self.fuel.sliderReleased.connect(lambda: ctl.set_fuel_pct(float(self.fuel.value())))
        self.fuel.valueChanged.connect(lambda v: self.fuel_lbl.setText(f"{v} %"))
        self.scale = QComboBox()
        self.scale.addItems(["0.5", "1", "2", "5", "10", "20"])
        self.scale.setCurrentText("1")
        self.scale.currentTextChanged.connect(lambda t: ctl.set_time_scale(float(t)))
        self.pause = QCheckBox("Pause simulation")
        self.pause.toggled.connect(ctl.set_paused)
        cold = QPushButton("Cold soak (overnight at ambient)")
        cold.clicked.connect(lambda: ctl.reset_cold(self.ambient.value()))
        warm = QPushButton("Hot soak (stopped a few minutes ago)")
        warm.clicked.connect(ctl.warm_restart)

        g_row = QHBoxLayout()
        g_row.addWidget(self.grade, 1)
        g_row.addWidget(self.grade_lbl)
        f_row = QHBoxLayout()
        f_row.addWidget(self.fuel, 1)
        f_row.addWidget(self.fuel_lbl)
        form = QFormLayout()
        form.addRow("Ambient temperature", self.ambient)
        form.addRow("Barometric pressure", self.baro)
        form.addRow("Road grade", g_row)
        form.addRow("Fuel level (set on release)", f_row)
        form.addRow("Time acceleration", self.scale)
        form.addRow("", self.pause)
        box = QGroupBox("Environment")
        box.setLayout(form)
        resets = QGroupBox("Engine state (keeps odometer, fuel and learned trims)")
        rl = QVBoxLayout(resets)
        rl.addWidget(cold)
        rl.addWidget(warm)
        lay = QVBoxLayout(self)
        lay.addWidget(box)
        lay.addWidget(resets)
        lay.addStretch(1)

    def refresh(self) -> None:
        if not self.fuel.isSliderDown():
            v = int(round(self.ctl.latest.get("fuel_pct", 60.0)))
            if v != self.fuel.value():
                self.fuel.blockSignals(True)
                self.fuel.setValue(v)
                self.fuel_lbl.setText(f"{v} %")
                self.fuel.blockSignals(False)


# ====================================================================== vehicle info
class VehicleInfoPanel(QWidget):
    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        info = ctl.info
        self.vin = QLineEdit(info.vin)
        self.vin.setMaxLength(17)
        self.vin.setFont(MONO)
        self.vin_state = QLabel()
        fix = QPushButton("Fix check digit")
        fix.clicked.connect(self._fix)
        self.vin.textChanged.connect(self._validate)
        self.calid = QLineEdit(info.calid)
        self.calid.setMaxLength(16)
        self.ecu_name = QLineEdit(info.ecu_name)
        self.ecu_name.setMaxLength(15)
        self.region = QComboBox()
        self.region.addItems(list(OBD_STANDARD))
        self.region.setCurrentText(info.region)
        self.cvn = QLabel(info.cvn_bytes.hex().upper())
        self.cvn.setFont(MONO)
        apply = QPushButton("Apply")
        apply.clicked.connect(self._apply)
        self.applied = QLabel("")
        vin_row = QHBoxLayout()
        vin_row.addWidget(self.vin, 1)
        vin_row.addWidget(fix)
        form = QFormLayout()
        form.addRow("VIN (Mode 09 / 02)", vin_row)
        form.addRow("", self.vin_state)
        form.addRow("Calibration ID (09 / 04)", self.calid)
        form.addRow("CVN (09 / 06)", self.cvn)
        form.addRow("ECU name (09 / 0A)", self.ecu_name)
        form.addRow("OBD standard (01 / 1C)", self.region)
        form.addRow(apply, self.applied)
        box = QGroupBox("Vehicle information reported to scan tools")
        box.setLayout(form)
        lay = QVBoxLayout(self)
        lay.addWidget(box)
        lay.addStretch(1)
        self._validate(self.vin.text())

    def _validate(self, text: str) -> None:
        t = text.upper()
        if len(t) != 17:
            self.vin_state.setText(f"<span style='color:#d55'>{len(t)}/17 characters</span>")
        elif vin_is_valid(t):
            self.vin_state.setText("<span style='color:#3c3'>valid check digit</span>")
        else:
            self.vin_state.setText("<span style='color:#d90'>check digit does not match (some tools warn)</span>")

    def _fix(self) -> None:
        try:
            self.vin.setText(with_check_digit(self.vin.text().upper()))
        except ValueError as e:
            self.vin_state.setText(f"<span style='color:#d55'>{e}</span>")

    def _apply(self) -> None:
        vin = self.vin.text().upper()
        if len(vin) != 17 or any(c in "IOQ" for c in vin) or not vin.isalnum():
            self.applied.setText("<span style='color:#d55'>VIN must be 17 characters, no I/O/Q</span>")
            return
        info = self.ctl.info
        info.vin = vin
        info.calid = self.calid.text()
        info.ecu_name = self.ecu_name.text()
        info.region = self.region.currentText()
        self.ctl.runner.submit(lambda s, r=info.region: setattr(s.ecu, "region", r))
        self.applied.setText("<span style='color:#3c3'>applied</span>")


# ====================================================================== diagnostics
from ..ecu.diag.codes import describe  # noqa: E402
from ..obd.server import FREEZE_PIDS  # noqa: E402

UASID_NAMES = {0x01: ("raw", 1.0), 0x0B: ("V", 0.001), 0x10: ("ms", 1.0), 0x24: ("counts", 1.0),
               0x2F: ("%", 0.01), 0xAF: ("%", 0.01)}


def _mode06_rows(frozen: dict[int, bytes]) -> list[tuple]:
    rows = []
    for mid in sorted(frozen):
        b = frozen[mid]
        for i in range(0, len(b), 9):
            _m, tid, uas = b[i], b[i + 1], b[i + 2]
            signed = uas >= 0x80
            vals = [int.from_bytes(b[i + 3 + 2 * k: i + 5 + 2 * k], "big", signed=signed) for k in range(3)]
            unit, scale = UASID_NAMES.get(uas, ("?", 1.0))
            rows.append((mid, tid, uas, unit, *(v * scale for v in vals)))
    return rows


class DiagnosticsPanel(QWidget):
    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        self.mil = QLabel()
        self.counters = QLabel()
        self.counters.setWordWrap(True)
        clear = QPushButton("Clear codes (same as Mode 04)")
        clear.clicked.connect(ctl.clear_codes)
        batt = QPushButton("Battery disconnect…")
        batt.clicked.connect(self._battery)
        self.soak_h = QDoubleSpinBox()
        self.soak_h.setRange(0.1, 48.0)
        self.soak_h.setValue(8.0)
        self.soak_h.setSuffix(" h")
        soak = QPushButton("Key off and soak")
        soak.clicked.connect(lambda: ctl.soak(self.soak_h.value()))

        top = QHBoxLayout()
        top.addWidget(self.mil)
        top.addWidget(self.counters, 1)
        actions = QHBoxLayout()
        actions.addWidget(clear)
        actions.addWidget(batt)
        actions.addStretch(1)
        actions.addWidget(self.soak_h)
        actions.addWidget(soak)

        self.dtcs = QTableWidget(0, 6)
        self.dtcs.setHorizontalHeaderLabels(["Code", "Description", "Pending (07)", "Confirmed (03)",
                                             "Permanent (0A)", "Freeze frame"])
        self.dtcs.verticalHeader().setVisible(False)
        self.dtcs.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.ff = QTableWidget(0, 3)
        self.ff.setHorizontalHeaderLabels(["PID", "Name", "Value"])
        self.ff.verticalHeader().setVisible(False)
        self.ff.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.m06 = QTableWidget(0, 7)
        self.m06.setHorizontalHeaderLabels(["MID", "TID", "Unit", "Value", "Min", "Max", "Result"])
        self.m06.verticalHeader().setVisible(False)
        self.m06.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)

        dbox = QGroupBox("Diagnostic trouble codes")
        QVBoxLayout(dbox).addWidget(self.dtcs)
        fbox = QGroupBox("Freeze frame (Mode 02, frame 0)")
        QVBoxLayout(fbox).addWidget(self.ff)
        mbox = QGroupBox("On-board monitor test results (Mode 06)")
        QVBoxLayout(mbox).addWidget(self.m06)
        lower = QSplitter()
        lower.addWidget(fbox)
        lower.addWidget(mbox)
        split = QSplitter(Qt.Orientation.Vertical)
        split.addWidget(dbox)
        split.addWidget(lower)
        lay = QVBoxLayout(self)
        lay.addLayout(top)
        lay.addLayout(actions)
        lay.addWidget(split, 1)
        self._ff_key = None
        self._m06_key = None

    def _battery(self) -> None:
        from PySide6.QtWidgets import QMessageBox

        ok = QMessageBox.question(
            self, "Battery disconnect",
            "Turn the key off and disconnect the battery?\n\nThe ECU loses learned fuel trims, stored and "
            "pending codes, freeze frame, monitor status and counters. Permanent codes stay.",
        )
        if ok == QMessageBox.StandardButton.Yes:
            self.ctl.battery_disconnect()

    def refresh(self) -> None:
        d = self.ctl.latest_diag
        mil = d.get("mil", 0)
        txt, col = {0: ("MIL off", "#888"), 1: ("MIL ON", "#e8a000"), 2: ("MIL FLASHING", "#e33")}[mil]
        self.mil.setText(f"<b style='font-size:15px;color:{col}'>{txt}</b>")
        iumpr = d.get("iumpr", (0, 0))
        self.counters.setText(
            f"warm-ups since clear <b>{d.get('warmups', 0)}</b> · distance since clear "
            f"<b>{d.get('dist_clear', 0.0):.1f} km</b> · run time since clear <b>{d.get('time_clear', 0.0):.0f} min</b>"
            f" · distance with MIL <b>{d.get('dist_mil', 0.0):.1f} km</b> · trips <b>{d.get('trips', 0)}</b>"
            f" · ignition cycles <b>{iumpr[1]}</b> · general denominator <b>{iumpr[0]}</b>"
        )
        conf = set(d.get("dtc_confirmed", ()))
        pend = set(d.get("dtc_pending", ()))
        perm = set(d.get("dtc_permanent", ()))
        ff = d.get("freeze_frame")
        codes = sorted(conf | pend | perm)
        self.dtcs.setRowCount(len(codes))
        for row, c in enumerate(codes):
            self.dtcs.setItem(row, 0, _item(c, mono=True))
            self.dtcs.setItem(row, 1, _item(describe(c)))
            for col, s in ((2, pend), (3, conf), (4, perm)):
                self.dtcs.setItem(row, col, _item("●" if c in s else ""))
            self.dtcs.setItem(row, 5, _item("●" if ff and ff[0] == c else ""))
        key = (ff[0], id(ff[1])) if ff else None
        if key != self._ff_key:
            self._ff_key = key
            pids = sorted(p for p in FREEZE_PIDS if p in MODE01) if ff else []
            self.ff.setRowCount(len(pids) + (1 if ff else 0))
            if ff:
                self.ff.setItem(0, 0, _item("02", mono=True))
                self.ff.setItem(0, 1, _item("DTC that stored the freeze frame"))
                self.ff.setItem(0, 2, _item(ff[0], mono=True))
                vals = dict(ff[1], obd_standard=self.ctl.info.obd_standard)
                for row, pid in enumerate(pids, start=1):
                    p = MODE01[pid]
                    self.ff.setItem(row, 0, _item(f"{pid:02X}", mono=True))
                    self.ff.setItem(row, 1, _item(p.name))
                    self.ff.setItem(row, 2, _item(f"{_fmt(p.decode(p.encode(vals)))} {p.unit}"))
        m06 = d.get("mode06", {})
        mkey = tuple(sorted((k, v) for k, v in m06.items()))
        if mkey != self._m06_key:
            self._m06_key = mkey
            rows = _mode06_rows(m06)
            self.m06.setRowCount(len(rows))
            for row, (mid, tid, _uas, unit, v, lo, hi) in enumerate(rows):
                result = "—" if lo == hi == 0 else ("PASS" if lo <= v <= hi else "FAIL")
                for col, txt in enumerate((f"{mid:02X}", f"{tid:02X}", unit, f"{v:g}", f"{lo:g}", f"{hi:g}", result)):
                    self.m06.setItem(row, col, _item(txt, mono=col < 2))


class MonitorsPanel(QWidget):
    MODES = ["auto", "complete", "incomplete"]

    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["Monitor", "Type", "Since clear (PID 01)", "This cycle (PID 41)",
                                              "Enabled this cycle", "Readiness", "What it is waiting for"])
        self.table.verticalHeader().setVisible(False)
        hdr = self.table.horizontalHeader()
        for col in range(6):
            hdr.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)
        hdr.setSectionResizeMode(6, QHeaderView.ResizeMode.Stretch)
        self._combos: dict[str, QComboBox] = {}
        note = QLabel("Readiness can follow the real drive-cycle logic (<b>auto</b>) or be forced for testing "
                      "scan tools' readiness / emissions-inspection screens.")
        note.setWordWrap(True)
        lay = QVBoxLayout(self)
        lay.addWidget(note)
        lay.addWidget(self.table)

    def refresh(self) -> None:
        mons = [m for m in self.ctl.latest_diag.get("monitors", ()) if m[3]]
        if self.table.rowCount() != len(mons):
            self.table.setRowCount(len(mons))
            self._combos.clear()
            for row, m in enumerate(mons):
                cb = QComboBox()
                cb.addItems(self.MODES)
                cb.currentTextChanged.connect(lambda t, k=m[0]: self.ctl.set_monitor_force(k, t))
                self.table.setCellWidget(row, 5, cb)
                self._combos[m[0]] = cb
        ok = QColor(60, 180, 90)
        bad = QColor(220, 140, 30)
        for row, (key, name, cont, _sup, ready, enabled, ready_cycle, force, hint) in enumerate(mons):
            self.table.setItem(row, 0, _item(name))
            self.table.setItem(row, 1, _item("continuous" if cont else "non-continuous"))
            for col, flag in ((2, ready), (3, ready_cycle)):
                it = _item("complete" if flag else "incomplete")
                it.setForeground(ok if flag else bad)
                self.table.setItem(row, col, it)
            self.table.setItem(row, 4, _item("yes" if enabled else "no"))
            cb = self._combos[key]
            if cb.currentText() != force and not cb.hasFocus():
                cb.blockSignals(True)
                cb.setCurrentText(force)
                cb.blockSignals(False)
            hint_item = _item(hint if not ready_cycle else "")
            hint_item.setToolTip(hint)
            self.table.setItem(row, 6, hint_item)


# ====================================================================== faults
from PySide6.QtWidgets import (  # noqa: E402
    QInputDialog,
    QSpinBox,
    QTreeWidget,
    QTreeWidgetItem,
)

from ..faults.catalog import BY_KEY, FAULTS, GROUPS, SENSOR, SEVERITY, TOGGLE  # noqa: E402
from ..faults.manager import FaultSetting  # noqa: E402
from ..sensors.faults import MODE_LABELS  # noqa: E402


class FaultsPanel(QWidget):
    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        self._key: str | None = None
        self._loading = False

        # --- presets bar
        self.preset = QComboBox()
        self._fill_presets()
        apply_p = QPushButton("Apply preset")
        apply_p.clicked.connect(self._apply_preset)
        save_p = QPushButton("Save current as preset…")
        save_p.clicked.connect(self._save_preset)
        clear_all = QPushButton("Clear all faults")
        clear_all.clicked.connect(self._clear_all)
        bar = QHBoxLayout()
        bar.addWidget(QLabel("Scenario:"))
        bar.addWidget(self.preset, 1)
        bar.addWidget(apply_p)
        bar.addWidget(save_p)
        bar.addStretch(1)
        bar.addWidget(clear_all)

        # --- tree of faults
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Fault", "State"])
        self.tree.header().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self._items: dict[str, QTreeWidgetItem] = {}
        for g in GROUPS:
            gi = QTreeWidgetItem([g, ""])
            f = gi.font(0)
            f.setBold(True)
            gi.setFont(0, f)
            self.tree.addTopLevelItem(gi)
            for d in FAULTS:
                if d.group == g:
                    it = QTreeWidgetItem([d.name, ""])
                    it.setData(0, Qt.ItemDataRole.UserRole, d.key)
                    gi.addChild(it)
                    self._items[d.key] = it
            gi.setExpanded(True)
        self.tree.currentItemChanged.connect(self._select)

        # --- editor
        self.title = QLabel("<i>Select a fault</i>")
        self.title.setWordWrap(True)
        self.expected = QLabel()
        self.expected.setWordWrap(True)
        self.enable = QCheckBox("Fault active")
        self.enable.toggled.connect(self._changed)
        self.value = QDoubleSpinBox()
        self.value.setDecimals(2)
        self.value.valueChanged.connect(self._changed)
        self.value_lbl = QLabel("Severity")
        self.cyl = QSpinBox()
        self.cyl.setRange(1, 4)
        self.cyl.valueChanged.connect(self._changed)
        self.cyl_lbl = QLabel("Cylinder")
        self.mode = QComboBox()
        self.mode.currentIndexChanged.connect(self._mode_changed)
        self.mode_lbl = QLabel("Failure mode")
        self.amount = QDoubleSpinBox()
        self.amount.setDecimals(3)
        self.amount.valueChanged.connect(self._changed)
        self.amount_lbl = QLabel("Amount")
        self.inter = QCheckBox("Intermittent")
        self.inter.toggled.connect(self._changed)
        self.duty = QDoubleSpinBox()
        self.duty.setRange(1, 99)
        self.duty.setValue(50)
        self.duty.setSuffix(" % of the time")
        self.duty.valueChanged.connect(self._changed)
        self.period = QDoubleSpinBox()
        self.period.setRange(0.5, 600)
        self.period.setValue(10)
        self.period.setSuffix(" s period")
        self.period.valueChanged.connect(self._changed)
        form = QFormLayout()
        form.addRow(self.title)
        form.addRow(self.expected)
        form.addRow(self.enable)
        form.addRow(self.value_lbl, self.value)
        form.addRow(self.cyl_lbl, self.cyl)
        form.addRow(self.mode_lbl, self.mode)
        form.addRow(self.amount_lbl, self.amount)
        form.addRow(self.inter)
        inter_row = QHBoxLayout()
        inter_row.addWidget(self.duty)
        inter_row.addWidget(self.period)
        form.addRow("", inter_row)
        note = QLabel("Faults act on the engine or the sensor wiring only. The ECU has to detect them itself, "
                      "so codes appear when the relevant monitor runs (see the Monitors tab for what it needs).")
        note.setWordWrap(True)
        form.addRow(note)
        editor = QGroupBox("Fault settings")
        editor.setLayout(form)

        split = QSplitter()
        split.addWidget(self.tree)
        split.addWidget(editor)
        split.setSizes([520, 480])
        lay = QVBoxLayout(self)
        lay.addLayout(bar)
        lay.addWidget(split, 1)
        self._show_editor(None)

    # ------------------------------------------------------------------ presets
    def _fill_presets(self) -> None:
        self.preset.clear()
        self.preset.addItems([p["name"] for p in self.ctl.presets.all()])

    def _apply_preset(self) -> None:
        p = self.ctl.presets.get(self.preset.currentText())
        if p:
            self.ctl.apply_preset(p)
            self._select(self.tree.currentItem(), None)

    def _save_preset(self) -> None:
        name, ok = QInputDialog.getText(self, "Save preset", "Preset name:")
        if ok and name.strip():
            self.ctl.save_preset(name.strip())
            self._fill_presets()
            self.preset.setCurrentText(name.strip())

    def _clear_all(self) -> None:
        self.ctl.clear_all_faults()
        self._select(self.tree.currentItem(), None)

    # ------------------------------------------------------------------ editor
    def _show_editor(self, key: str | None) -> None:
        d = BY_KEY.get(key) if key else None
        for w in (self.enable, self.inter, self.duty, self.period):
            w.setEnabled(d is not None)
        sev = d is not None and d.kind == SEVERITY
        sensor = d is not None and d.kind == SENSOR
        for w in (self.value, self.value_lbl):
            w.setVisible(sev)
        for w in (self.cyl, self.cyl_lbl):
            w.setVisible(d is not None and d.per_cylinder)
        for w in (self.mode, self.mode_lbl):
            w.setVisible(sensor)
        if d is None:
            self.title.setText("<i>Select a fault</i>")
            self.expected.setText("")
            for w in (self.amount, self.amount_lbl):
                w.setVisible(False)
            return
        self.title.setText(f"<b>{d.name}</b> <span style='color:#888'>({d.group})</span>")
        self.expected.setText(f"Expected: {d.expected}")
        if sev:
            self.value.setRange(d.lo, d.hi)
            self.value.setSuffix(f" {d.unit}")
        if sensor:
            self.mode.blockSignals(True)
            self.mode.clear()
            for m in d.modes:
                self.mode.addItem(MODE_LABELS[m], m)
            self.mode.blockSignals(False)

    def _update_amount(self, d, mode: str) -> None:
        spec = d.amounts.get(mode) if d.kind == SENSOR else None
        has = spec is not None and spec[1] != spec[2]
        for w in (self.amount, self.amount_lbl):
            w.setVisible(has)
        if has:
            unit, lo, hi, default, _scale = spec
            self.amount.setRange(lo, hi)
            self.amount.setSuffix(f" {unit}")
            self.amount_lbl.setText({"slow": "Time constant", "noisy": "Noise (σ)", "gain": "Gain error"}.get(
                mode, "Offset"))
        return default if has else None

    def _select(self, item, _prev=None) -> None:
        key = item.data(0, Qt.ItemDataRole.UserRole) if item is not None else None
        self._key = key
        self._loading = True
        try:
            self._show_editor(key)
            if key is None:
                return
            d = BY_KEY[key]
            s = self.ctl.fault_settings.get(key)
            self.enable.setChecked(s is not None)
            if d.kind == SEVERITY:
                self.value.setValue(d.default if s is None or s.value is None else s.value)
            self.cyl.setValue(s.cylinder if s else 3)
            if d.kind == SENSOR:
                mode = s.mode if s and s.mode else d.modes[0]
                self.mode.setCurrentIndex(max(0, self.mode.findData(mode)))
                default = self._update_amount(d, mode)
                if default is not None:
                    self.amount.setValue(default if s is None or s.amount is None else s.amount)
            else:
                self._update_amount(d, "")
            self.inter.setChecked(bool(s and s.intermittent))
            self.duty.setValue((s.duty if s else 0.5) * 100.0)
            self.period.setValue(s.period_s if s else 10.0)
        finally:
            self._loading = False

    def _mode_changed(self) -> None:
        if self._key is None:
            return
        d = BY_KEY[self._key]
        default = self._update_amount(d, self.mode.currentData() or "")
        if default is not None and not self._loading:
            self._loading = True
            self.amount.setValue(default)
            self._loading = False
        self._changed()

    def _changed(self, *_a) -> None:
        if self._loading or self._key is None:
            return
        d = BY_KEY[self._key]
        if not self.enable.isChecked():
            self.ctl.clear_fault(self._key)
        else:
            self.ctl.set_fault(FaultSetting(
                key=self._key,
                value=self.value.value() if d.kind == SEVERITY else None,
                cylinder=self.cyl.value(),
                mode=(self.mode.currentData() or "") if d.kind == SENSOR else "",
                amount=self.amount.value() if self.amount.isVisible() else None,
                intermittent=self.inter.isChecked(),
                duty=self.duty.value() / 100.0,
                period_s=self.period.value(),
            ))
        self.refresh()

    def refresh(self) -> None:
        active = self.ctl.fault_settings
        on = QColor(230, 120, 30)
        for key, it in self._items.items():
            s = active.get(key)
            d = BY_KEY[key]
            if s is None:
                it.setText(1, "")
                continue
            if d.kind == TOGGLE:
                txt = "active"
            elif d.kind == SEVERITY:
                txt = f"{d.default if s.value is None else s.value:g} {d.unit}"
            else:
                txt = MODE_LABELS.get(s.mode, s.mode)
            if d.per_cylinder:
                txt += f", cyl {s.cylinder}"
            if s.intermittent:
                txt += " (intermittent)"
            it.setText(1, txt)
            it.setForeground(1, on)
