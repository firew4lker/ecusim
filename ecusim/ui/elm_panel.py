# SPDX-License-Identifier: GPL-3.0-or-later
"""ELM327 tab: switch each emulated adapter transport on or off (DESIGN §7.7.2)."""

from __future__ import annotations

from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from ..elm.emulator import IDENTITIES
from ..elm.manager import KINDS, unavailable
from ..elm.transports import default_pty_link, local_addresses
from .controller import SimController

TITLES = {
    "tcp": "Wi-Fi / TCP",
    "pty": "Virtual serial port",
    "serial": "Existing serial port",
    "bt": "Bluetooth",
}
HELP = {
    "tcp": "For apps with a Wi-Fi adapter option (Torque, Car Scanner, OBD Auto Doctor) and python-OBD.",
    "pty": "A serial port that exists only while this is on. Point the diagnostic program at the path.",
    "serial": "Windows: one end of a com0com pair (the app opens the other end). "
              "Or a real USB-serial null-modem cable to another computer.",
    "bt": "Android apps that pair with 'OBDII' Bluetooth adapters. Pair the phone with this computer first.",
}


class ElmPanel(QWidget):
    def __init__(self, ctl: SimController, parent: QWidget | None = None):
        super().__init__(parent)
        self.ctl = ctl
        self.mgr = ctl.elm
        s = self.mgr.settings
        self.switch: dict[str, QCheckBox] = {}
        self.state: dict[str, QLabel] = {}
        self.fields: dict[str, list[QWidget]] = {}
        self._log_seen = None

        # ---- shared options
        self.identity = QComboBox()
        self.identity.setEditable(True)
        self.identity.addItems(list(IDENTITIES))
        self.identity.setCurrentText(s["identity"])
        self.identity.currentTextChanged.connect(lambda t: self._option(identity=t.strip() or IDENTITIES[0]))
        self.latency = QDoubleSpinBox()
        self.latency.setRange(0.0, 500.0)
        self.latency.setSuffix(" ms")
        self.latency.setValue(float(s["latency_ms"]))
        self.latency.setToolTip("How long the adapter takes to answer a request. A real ELM327 takes 30–60 ms.")
        self.latency.valueChanged.connect(lambda v: self._option(latency_ms=v))
        opt = QFormLayout()
        opt.addRow("Reports itself as", self.identity)
        opt.addRow("Response time", self.latency)
        note = QLabel("Every connection is a separate adapter: switch on only the ones you need. "
                      "Each one serves one app at a time. The car's protocol follows the CAN tab settings "
                      "(11/29-bit, 500/250 kbit/s); the CAN adapter itself is not needed.")
        note.setWordWrap(True)
        opt.addRow(note)
        options = QGroupBox("Adapter")
        options.setLayout(opt)

        # ---- transports
        self.tcp_port = QSpinBox()
        self.tcp_port.setRange(1, 65535)
        self.tcp_port.setValue(int(s["tcp"]["port"]))
        self.tcp_net = QCheckBox("Allow phones and other computers to connect")
        self.tcp_net.setChecked(bool(s["tcp"]["network"]))
        self.tcp_net.setToolTip("Off: only programs on this computer can connect (127.0.0.1).")
        self.pty_link = QLineEdit(s["pty"]["link"])
        self.pty_link.setPlaceholderText(str(default_pty_link()))
        self.ser_port = QComboBox()
        self.ser_port.setEditable(True)
        self.ser_refresh = QPushButton("Rescan")
        self.ser_refresh.clicked.connect(self._scan_ports)
        self._scan_ports()
        self.ser_port.setCurrentText(s["serial"]["port"])
        self.ser_baud = QComboBox()
        self.ser_baud.setEditable(True)
        self.ser_baud.addItems(["9600", "38400", "57600", "115200", "230400", "500000"])
        self.ser_baud.setCurrentText(str(s["serial"]["baud"]))
        self.bt_name = QLineEdit(s["bt"]["name"])
        self.bt_rename = QCheckBox("Rename this computer's Bluetooth to it while on")
        self.bt_rename.setChecked(bool(s["bt"]["rename"]))
        self.bt_rename.setToolTip("Many apps list only devices called OBDII / OBD2 / ELM327. "
                                  "The old name comes back when this is switched off.")

        ser_row = QHBoxLayout()
        ser_row.addWidget(self.ser_port, 1)
        ser_row.addWidget(self.ser_refresh)
        rows = {
            "tcp": [("Port", self.tcp_port), ("", self.tcp_net)],
            "pty": [("Path", self.pty_link)],
            "serial": [("Port", ser_row), ("Baud rate", self.ser_baud)],
            "bt": [("Name", self.bt_name), ("", self.bt_rename)],
        }
        self.fields = {"tcp": [self.tcp_port, self.tcp_net], "pty": [self.pty_link],
                       "serial": [self.ser_port, self.ser_refresh, self.ser_baud],
                       "bt": [self.bt_name, self.bt_rename]}

        col = QVBoxLayout()
        col.addWidget(options)
        for kind in KINDS:
            box = QGroupBox(TITLES[kind])
            form = QFormLayout(box)
            sw = QCheckBox("On")
            sw.toggled.connect(lambda on, k=kind: self._toggle(k, on))
            st = QLabel("off")
            st.setWordWrap(True)
            head = QHBoxLayout()
            head.addWidget(sw)
            head.addWidget(st, 1)
            form.addRow(head)
            helptext = QLabel(HELP[kind])
            helptext.setWordWrap(True)
            helptext.setStyleSheet("color: palette(placeholder-text);")
            form.addRow(helptext)
            for label, w in rows[kind]:
                form.addRow(label, w)
            reason = unavailable(kind)
            if reason:
                sw.setEnabled(False)
                for w in self.fields[kind]:
                    w.setEnabled(False)
                st.setText(reason)
            self.switch[kind], self.state[kind] = sw, st
            col.addWidget(box)
        col.addStretch(1)
        left = QWidget()
        left.setLayout(col)
        scroll = QScrollArea()
        scroll.setWidget(left)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        # ---- conversation
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(400)
        self.log.setFont(_mono())
        self.log_pause = QCheckBox("Pause")
        lbox = QGroupBox("What the app sends (→) and the adapter answers (←)")
        ll = QVBoxLayout(lbox)
        ll.addWidget(self.log_pause)
        ll.addWidget(self.log, 1)

        split = QSplitter()
        split.addWidget(scroll)
        split.addWidget(lbox)
        split.setStretchFactor(0, 2)
        split.setStretchFactor(1, 3)
        lay = QVBoxLayout(self)
        lay.addWidget(split)
        self.sync()

    # ------------------------------------------------------------------ actions
    def _scan_ports(self) -> None:
        current = self.ser_port.currentText()
        self.ser_port.clear()
        try:
            from serial.tools import list_ports

            self.ser_port.addItems(sorted(p.device for p in list_ports.comports()))
        except Exception:
            pass
        self.ser_port.setCurrentText(current)

    def _option(self, **values) -> None:
        self.mgr.update(None, **values)
        self._save()

    def _settings_from_widgets(self, kind: str) -> dict:
        if kind == "tcp":
            return {"port": self.tcp_port.value(), "network": self.tcp_net.isChecked()}
        if kind == "pty":
            return {"link": self.pty_link.text().strip()}
        if kind == "serial":
            baud = self.ser_baud.currentText().strip()
            return {"port": self.ser_port.currentText().strip(), "baud": int(baud) if baud.isdigit() else 38400}
        return {"name": self.bt_name.text().strip() or "OBDII", "rename": self.bt_rename.isChecked()}

    def _toggle(self, kind: str, on: bool) -> None:
        if on:
            self.mgr.update(kind, **self._settings_from_widgets(kind))
        try:
            self.mgr.set_enabled(kind, on)
        except Exception:
            self.mgr.settings[kind]["enabled"] = False  # the error stays visible in the status line
        self._save()
        self.sync()

    def _save(self) -> None:
        try:
            self.mgr.save()
        except OSError:
            pass

    # ------------------------------------------------------------------ display
    def sync(self) -> None:
        for kind in KINDS:
            reason = unavailable(kind)
            if reason:
                continue
            running = self.mgr.is_running(kind)
            sw = self.switch[kind]
            sw.blockSignals(True)
            sw.setChecked(running)
            sw.blockSignals(False)
            for w in self.fields[kind]:
                w.setEnabled(not running)
        self.refresh_status()

    def refresh_status(self) -> None:
        for kind in KINDS:
            if unavailable(kind):
                continue
            text = self.mgr.status(kind)
            t = self.mgr.transports.get(kind)
            failed = t is not None and bool(t.error)
            if self.mgr.is_running(kind):
                c = self.mgr.counters(kind)
                text += f" · {c['requests']} requests, {c['responses']} answered"
                if kind == "tcp" and self.mgr.settings["tcp"]["network"]:
                    ips = ", ".join(local_addresses()) or "this computer's IP address"
                    text += f"<br>In the phone app: Wi-Fi adapter, IP <b>{ips}</b>, port " \
                            f"<b>{self.mgr.settings['tcp']['port']}</b>"
                note = self.mgr.note(kind)
                if note:
                    text += f"<br><i>{note}</i>"
            if failed:
                text = f"<span style='color:#d33'>{text}</span>"
            elif self.mgr.is_running(kind):
                text = f"<b style='color:#3a3'>On</b> — {text}"
            self.state[kind].setText(text)
            if failed and self.switch[kind].isChecked():
                self.switch[kind].blockSignals(True)
                self.switch[kind].setChecked(False)
                self.switch[kind].blockSignals(False)
                for w in self.fields[kind]:
                    w.setEnabled(True)

    def refresh(self) -> None:
        self.refresh_status()
        if self.log_pause.isChecked():
            return
        lines = list(self.mgr.log)
        key = (len(lines), lines[-1].t if lines else 0.0)
        if key == self._log_seen:
            return
        self._log_seen = key
        arrows = {"app": "→", "elm": "←", "info": "·"}
        self.log.setPlainText("\n".join(f"[{TITLES[ln.kind]}] {arrows.get(ln.who, ' ')} {ln.text}"
                                        for ln in lines[-300:]))
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())


def _mono():
    from PySide6.QtGui import QFont

    f = QFont("monospace")
    f.setStyleHint(QFont.StyleHint.Monospace)
    return f
