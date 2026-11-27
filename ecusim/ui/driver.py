"""Driver controls: key switch, pedals, H-pattern shifter, switches (DESIGN §9.2)."""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSlider,
    QVBoxLayout,
    QWidget,
)

KEY_POSITIONS = ["off", "acc", "on", "start"]


class KeySwitch(QWidget):
    """OFF / ACC / ON plus a momentary START button (spring-returns to ON)."""

    key_changed = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.buttons: dict[str, QPushButton] = {}
        for k in KEY_POSITIONS:
            b = QPushButton(k.upper())
            b.setCheckable(k != "start")
            b.setMinimumHeight(34)
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            lay.addWidget(b)
            self.buttons[k] = b
        for k in ("off", "acc", "on"):
            self.buttons[k].clicked.connect(lambda _=False, k=k: self.key_changed.emit(k))
        st = self.buttons["start"]
        st.setToolTip("Hold to crank (keyboard: K)")
        st.pressed.connect(lambda: self.key_changed.emit("start"))
        st.released.connect(lambda: self.key_changed.emit("on"))
        self.show_key("off")

    def show_key(self, key: str) -> None:
        for k, b in self.buttons.items():
            if b.isCheckable():
                b.setChecked(k == key or (key == "start" and k == "on"))
        self.buttons["start"].setDown(key == "start")


class Pedal(QWidget):
    """Vertical pedal: drag with the mouse, springs back on release unless held."""

    value_changed = Signal(float)

    def __init__(self, name: str, color: QColor, parent: QWidget | None = None):
        super().__init__(parent)
        self.name = name
        self.color = color
        self.value = 0.0
        self.hold = False
        self._dragging = False
        self.setMinimumSize(56, 170)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        self.setToolTip(f"{name}: drag down to press. Right-click toggles hold.")

    def set_value(self, v: float, emit: bool = True) -> None:
        v = min(max(v, 0.0), 1.0)
        if abs(v - self.value) > 1e-4:
            self.value = v
            self.update()
            if emit:
                self.value_changed.emit(v)

    def _bar(self) -> QRectF:
        return QRectF(8, 22, self.width() - 16, self.height() - 44)

    def _from_y(self, y: float) -> float:
        b = self._bar()
        return (y - b.top()) / b.height()

    def mousePressEvent(self, ev) -> None:  # noqa: N802
        if ev.button() == Qt.MouseButton.RightButton:
            self.hold = not self.hold
            self.update()
            return
        self._dragging = True
        self.set_value(self._from_y(ev.position().y()))

    def mouseMoveEvent(self, ev) -> None:  # noqa: N802
        if self._dragging:
            self.set_value(self._from_y(ev.position().y()))

    def mouseReleaseEvent(self, ev) -> None:  # noqa: N802
        self._dragging = False
        if ev.button() == Qt.MouseButton.LeftButton and not self.hold:
            self.set_value(0.0)

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        b = self._bar()
        p.setPen(QPen(QColor(90, 96, 108), 1.5))
        p.setBrush(QColor(30, 33, 40))
        p.drawRoundedRect(b, 6, 6)
        fill = QRectF(b.left(), b.top(), b.width(), b.height() * self.value)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(self.color)
        p.drawRoundedRect(fill, 6, 6)
        f = QFont(self.font())
        f.setPixelSize(11)
        f.setBold(True)
        p.setFont(f)
        p.setPen(self.palette().text().color())
        p.drawText(QRectF(0, 0, self.width(), 20), Qt.AlignmentFlag.AlignCenter, self.name)
        p.drawText(QRectF(0, self.height() - 20, self.width(), 20), Qt.AlignmentFlag.AlignCenter,
                   f"{self.value * 100:.0f}%" + (" ⏸" if self.hold else ""))
        p.end()


class Shifter(QWidget):
    """Clickable H-pattern: R 1 3 5 on top, 2 4 below, N in the middle."""

    gear_requested = Signal(int)

    LAYOUT = {-1: (0, 0), 1: (1, 0), 3: (2, 0), 5: (3, 0), 2: (1, 2), 4: (2, 2), 0: (1.5, 1)}

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.gear = 0
        self.refused = False
        self.setMinimumSize(200, 140)
        self.setToolTip("Click a gear (keyboard: 1-5, R, N). Needs the clutch pressed.")

    def _pos(self, gear: int) -> QPointF:
        col, row = self.LAYOUT[gear]
        w, h = self.width(), self.height()
        return QPointF(w * (0.14 + 0.24 * col), h * (0.17 + 0.33 * row))

    def mousePressEvent(self, ev) -> None:  # noqa: N802
        pos = ev.position()
        best = min(self.LAYOUT, key=lambda g: (self._pos(g) - pos).manhattanLength())
        if (self._pos(best) - pos).manhattanLength() < 60:
            self.gear_requested.emit(best)

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(QPen(QColor(110, 116, 128), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        mid_y = self._pos(0).y()
        p.drawLine(QPointF(self._pos(-1).x(), mid_y), QPointF(self._pos(5).x(), mid_y))
        for g in (-1, 1, 3, 5):
            p.drawLine(self._pos(g), QPointF(self._pos(g).x(), mid_y))
        for g in (2, 4):
            p.drawLine(self._pos(g), QPointF(self._pos(g).x(), mid_y))
        f = QFont(self.font())
        f.setPixelSize(14)
        f.setBold(True)
        p.setFont(f)
        for g in self.LAYOUT:
            c = self._pos(g)
            active = g == self.gear
            color = QColor(220, 60, 60) if (active and self.refused) else QColor(255, 140, 40) if active else QColor(
                52, 56, 66)
            r = 16 if g != 0 else 12
            p.setPen(QPen(QColor(150, 156, 168), 1.5))
            p.setBrush(color)
            p.drawEllipse(c, r, r)
            p.setPen(QColor(255, 255, 255))
            label = {-1: "R", 0: "N"}.get(g, str(g))
            p.drawText(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r), Qt.AlignmentFlag.AlignCenter, label)
        p.end()


class DriverPanel(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.key = KeySwitch()
        self.clutch = Pedal("CLUTCH", QColor(80, 140, 255))
        self.brake = Pedal("BRAKE", QColor(230, 70, 60))
        self.gas = Pedal("GAS", QColor(70, 200, 100))
        self.shifter = Shifter()
        self.handbrake = QCheckBox("Handbrake (H)")
        self.ac = QCheckBox("A/C")
        self.lights = QCheckBox("Lights (L)")
        for w in (self.handbrake, self.ac, self.lights):
            w.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        key_box = QGroupBox("Ignition")
        kl = QVBoxLayout(key_box)
        kl.addWidget(self.key)

        # Hand throttle: holds the gas at a set level; the pedal can still add more on top
        self.hand = QSlider(Qt.Orientation.Horizontal)
        self.hand.setRange(0, 100)
        self.hand.setPageStep(5)
        self.hand.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.hand.setToolTip("Hand throttle: keeps the gas at this level ( [ / ] adjust, G releases)")
        self.hand_lbl = QLabel("0 %")
        self.hand_lbl.setMinimumWidth(40)
        self.hand_zero = QPushButton("0")
        self.hand_zero.setFixedWidth(30)
        self.hand_zero.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.hand_zero.setToolTip("Release the hand throttle (G)")
        self.hand_zero.clicked.connect(lambda: self.hand.setValue(0))
        self.hand.valueChanged.connect(lambda v: self.hand_lbl.setText(f"{v} %"))

        pedal_box = QGroupBox("Pedals")
        pv = QVBoxLayout(pedal_box)
        pl = QHBoxLayout()
        for w in (self.clutch, self.brake, self.gas):
            pl.addWidget(w)
        pv.addLayout(pl, 1)
        hl = QHBoxLayout()
        hl.addWidget(QLabel("Hand throttle"))
        hl.addWidget(self.hand, 1)
        hl.addWidget(self.hand_lbl)
        hl.addWidget(self.hand_zero)
        pv.addLayout(hl)

        gear_box = QGroupBox("Gearbox")
        gl = QVBoxLayout(gear_box)
        gl.addWidget(self.shifter)

        sw_box = QGroupBox("Switches")
        sl = QGridLayout(sw_box)
        sl.addWidget(self.handbrake, 0, 0)
        sl.addWidget(self.ac, 0, 1)
        sl.addWidget(self.lights, 1, 0)

        help_lbl = QLabel(
            "<small><b>Keys:</b> W/↑ gas · S/↓ brake · Space clutch · 1-5 R N gears · "
            "K hold to crank · H handbrake · L lights · [ ] hand throttle ∓5% · G release it</small>"
        )
        help_lbl.setWordWrap(True)

        lay = QVBoxLayout(self)
        lay.addWidget(key_box)
        lay.addWidget(pedal_box, 1)
        lay.addWidget(gear_box)
        lay.addWidget(sw_box)
        lay.addWidget(help_lbl)
