# SPDX-License-Identifier: GPL-3.0-or-later
"""Instrument cluster widgets, painted with QPainter (DESIGN §9.3)."""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen, QRadialGradient
from PySide6.QtWidgets import QHBoxLayout, QSizePolicy, QVBoxLayout, QWidget

BG = QColor(14, 16, 20)
FACE = QColor(24, 27, 33)
TICK = QColor(225, 228, 235)
DIM = QColor(120, 126, 138)
NEEDLE = QColor(255, 90, 40)
RED = QColor(220, 40, 40)
AMBER = QColor(255, 176, 0)
GREEN = QColor(60, 200, 90)
BLUE = QColor(70, 140, 255)
OFF = QColor(48, 52, 60)


def _pt(cx: float, cy: float, r: float, deg: float) -> QPointF:
    a = math.radians(deg)
    return QPointF(cx + r * math.cos(a), cy - r * math.sin(a))


class DialGauge(QWidget):
    """Round analog gauge with a damped needle."""

    START = 225.0
    SPAN = 270.0

    def __init__(
        self,
        vmin: float,
        vmax: float,
        major: float,
        minor: int = 5,
        label_div: float = 1.0,
        title: str = "",
        unit: str = "",
        red_from: float | None = None,
        digital: bool = True,
        label_every: int = 1,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self.vmin, self.vmax, self.major, self.minor = vmin, vmax, major, minor
        self.label_every = label_every
        self.label_div = label_div
        self.title, self.unit = title, unit
        self.red_from = red_from
        self.digital = digital
        self.value = vmin
        self._shown = vmin
        self.setMinimumSize(150, 150)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_value(self, v: float) -> None:
        self.value = v

    def tick(self) -> None:
        """Advance needle damping by one UI frame."""
        self._shown += (self.value - self._shown) * 0.35
        self.update()

    def _angle(self, v: float) -> float:
        f = (min(max(v, self.vmin), self.vmax) - self.vmin) / (self.vmax - self.vmin)
        return self.START - self.SPAN * f

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        side = min(w, h)
        cx, cy = w / 2.0, h / 2.0
        r = side / 2.0 - 4

        grad = QRadialGradient(cx, cy, r)
        grad.setColorAt(0.0, QColor(34, 38, 46))
        grad.setColorAt(1.0, FACE)
        p.setPen(QPen(QColor(70, 76, 88), 2))
        p.setBrush(QBrush(grad))
        p.drawEllipse(QPointF(cx, cy), r, r)

        if self.red_from is not None:
            a0 = self._angle(self.red_from)
            a1 = self._angle(self.vmax)
            rr = r * 0.86
            p.setPen(QPen(RED, max(3.0, r * 0.06), Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawArc(QRectF(cx - rr, cy - rr, 2 * rr, 2 * rr), int(a1 * 16), int((a0 - a1) * 16))

        n_major = int(round((self.vmax - self.vmin) / self.major))
        font = QFont(self.font())
        font.setPixelSize(max(9, int(r * 0.13)))
        font.setBold(True)
        p.setFont(font)
        for i in range(n_major + 1):
            v = self.vmin + i * self.major
            a = self._angle(v)
            red = self.red_from is not None and v >= self.red_from
            p.setPen(QPen(RED if red else TICK, max(2.0, r * 0.025)))
            p.drawLine(_pt(cx, cy, r * 0.92, a), _pt(cx, cy, r * 0.78, a))
            if i % self.label_every == 0:
                lbl = f"{v / self.label_div:g}"
                c = _pt(cx, cy, r * 0.63, a)
                p.setPen(RED if red else TICK)
                p.drawText(QRectF(c.x() - r * 0.2, c.y() - r * 0.1, r * 0.4, r * 0.2), Qt.AlignmentFlag.AlignCenter,
                           lbl)
            if i < n_major:
                for j in range(1, self.minor):
                    am = self._angle(v + self.major * j / self.minor)
                    p.setPen(QPen(DIM, max(1.0, r * 0.012)))
                    p.drawLine(_pt(cx, cy, r * 0.92, am), _pt(cx, cy, r * 0.85, am))

        small = QFont(self.font())
        small.setPixelSize(max(8, int(r * 0.09)))
        p.setFont(small)
        p.setPen(DIM)
        p.drawText(QRectF(cx - r * 0.5, cy - r * 0.42, r, r * 0.16), Qt.AlignmentFlag.AlignCenter, self.title)
        if self.digital:
            big = QFont(self.font())
            big.setPixelSize(max(10, int(r * 0.17)))
            big.setBold(True)
            p.setFont(big)
            p.setPen(TICK)
            p.drawText(QRectF(cx - r * 0.5, cy + r * 0.32, r, r * 0.22), Qt.AlignmentFlag.AlignCenter,
                       f"{self.value:.0f}")
            p.setFont(small)
            p.setPen(DIM)
            p.drawText(QRectF(cx - r * 0.5, cy + r * 0.53, r, r * 0.14), Qt.AlignmentFlag.AlignCenter, self.unit)

        a = self._angle(self._shown)
        tip = _pt(cx, cy, r * 0.88, a)
        left = _pt(cx, cy, r * 0.035, a + 90)
        right = _pt(cx, cy, r * 0.035, a - 90)
        tail = _pt(cx, cy, r * 0.15, a + 180)
        path = QPainterPath(tip)
        path.lineTo(left)
        path.lineTo(tail)
        path.lineTo(right)
        path.closeSubpath()
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(NEEDLE)
        p.drawPath(path)
        p.setBrush(QColor(60, 64, 72))
        p.drawEllipse(QPointF(cx, cy), r * 0.07, r * 0.07)
        p.end()


class ArcGauge(QWidget):
    """Small quarter-circle gauge (coolant temperature, fuel)."""

    def __init__(self, low_label: str, high_label: str, title: str, warn_high: bool = True,
                 parent: QWidget | None = None):
        super().__init__(parent)
        self.low_label, self.high_label, self.title = low_label, high_label, title
        self.warn_high = warn_high
        self.fraction = 0.0
        self._shown = 0.0
        self.setMinimumSize(110, 80)

    def set_fraction(self, f: float) -> None:
        self.fraction = min(max(f, 0.0), 1.0)

    def tick(self) -> None:
        self._shown += (self.fraction - self._shown) * 0.08
        self.update()

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w * 0.42, h * 0.62)
        cx, cy = w / 2.0, h * 0.74
        start, span = 150.0, 120.0
        rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)
        p.setPen(QPen(OFF, 6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
        p.drawArc(rect, int((start - span) * 16), int(span * 16))
        warn_from = 0.85 if self.warn_high else 0.0
        warn_to = 1.0 if self.warn_high else 0.12
        p.setPen(QPen(RED, 6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
        a_lo = start - span * warn_to
        p.drawArc(rect, int(a_lo * 16), int(span * (warn_to - warn_from) * 16))
        for f in (0.0, 0.5, 1.0):
            a = start - span * f
            p.setPen(QPen(TICK, 2))
            p.drawLine(_pt(cx, cy, r * 1.08, a), _pt(cx, cy, r * 0.88, a))
        font = QFont(self.font())
        font.setPixelSize(max(9, int(r * 0.2)))
        font.setBold(True)
        p.setFont(font)
        p.setPen(TICK)
        lo = _pt(cx, cy, r, start)
        hi = _pt(cx, cy, r, start - span)
        p.drawText(QRectF(lo.x() - 15, lo.y() + 4, 30, 18), Qt.AlignmentFlag.AlignCenter, self.low_label)
        p.drawText(QRectF(hi.x() - 15, hi.y() + 4, 30, 18), Qt.AlignmentFlag.AlignCenter, self.high_label)
        p.setPen(DIM)
        p.drawText(QRectF(cx - r, cy + 6, 2 * r, h - cy - 6), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop,
                   self.title)
        a = start - span * self._shown
        p.setPen(QPen(NEEDLE, 3))
        p.drawLine(QPointF(cx, cy), _pt(cx, cy, r * 0.92, a))
        p.setBrush(QColor(60, 64, 72))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(cx, cy), 5, 5)
        p.end()


class Lamp(QWidget):
    def __init__(self, text: str, color: QColor, parent: QWidget | None = None):
        super().__init__(parent)
        self.text = text
        self.color = color
        self.state = 0  # 0 off, 1 on, 2 flashing
        self._phase = 0
        self.setMinimumSize(52, 26)
        self.setToolTip(text)

    def set_state(self, s: int) -> None:
        self.state = s

    def tick(self) -> None:
        self._phase = (self._phase + 1) % 16
        self.update()

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        lit = self.state == 1 or (self.state == 2 and self._phase < 8)
        rect = QRectF(2, 2, self.width() - 4, self.height() - 4)
        p.setPen(QPen(self.color if lit else OFF, 1.5))
        p.setBrush(self.color.darker(260) if lit else QColor(22, 24, 28))
        p.drawRoundedRect(rect, 5, 5)
        f = QFont(self.font())
        f.setPixelSize(11)
        f.setBold(True)
        p.setFont(f)
        p.setPen(self.color if lit else OFF.lighter(130))
        p.drawText(rect, Qt.AlignmentFlag.AlignCenter, self.text)
        p.end()


class InfoDisplay(QWidget):
    """Centre display: gear, odometer, trip, outside temperature, messages."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.gear = "N"
        self.odometer = 0.0
        self.trip = 0.0
        self.ambient = 15.0
        self.message = ""
        self.message_color = AMBER
        self.setMinimumSize(150, 100)

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = QRectF(4, 4, self.width() - 8, self.height() - 8)
        p.setPen(QPen(QColor(60, 66, 78), 1.5))
        p.setBrush(QColor(10, 12, 16))
        p.drawRoundedRect(rect, 8, 8)
        big = QFont(self.font())
        big.setPixelSize(int(rect.height() * 0.42))
        big.setBold(True)
        p.setFont(big)
        p.setPen(TICK)
        p.drawText(QRectF(rect.x(), rect.y() + 2, rect.width(), rect.height() * 0.5), Qt.AlignmentFlag.AlignCenter,
                   self.gear)
        small = QFont(self.font())
        small.setPixelSize(max(10, int(rect.height() * 0.11)))
        p.setFont(small)
        p.setPen(DIM)
        y = rect.y() + rect.height() * 0.52
        line_h = rect.height() * 0.15
        p.drawText(QRectF(rect.x() + 8, y, rect.width() - 16, line_h), Qt.AlignmentFlag.AlignLeft,
                   f"ODO {self.odometer:8.1f} km")
        p.drawText(QRectF(rect.x() + 8, y + line_h, rect.width() - 16, line_h), Qt.AlignmentFlag.AlignLeft,
                   f"TRIP {self.trip:6.1f} km")
        p.drawText(QRectF(rect.x() + 8, y, rect.width() - 16, line_h), Qt.AlignmentFlag.AlignRight,
                   f"{self.ambient:+.0f}°C")
        if self.message:
            p.setPen(self.message_color)
            p.drawText(QRectF(rect.x() + 8, y + 2 * line_h, rect.width() - 16, line_h),
                       Qt.AlignmentFlag.AlignCenter, self.message)
        p.end()


def coolant_fraction(ect: float) -> float:
    """Real clusters park the needle mid-scale across the normal band."""
    pts = [(40.0, 0.0), (75.0, 0.45), (80.0, 0.5), (105.0, 0.5), (112.0, 0.7), (125.0, 1.0)]
    if ect <= pts[0][0]:
        return 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if ect <= x1:
            return y0 + (y1 - y0) * (ect - x0) / (x1 - x0)
    return 1.0


class Cluster(QWidget):
    LAMPS = [("CHECK", AMBER), ("EPC", AMBER), ("BATT", RED), ("OIL", RED), ("TEMP", RED), ("BRAKE", RED),
             ("FAN", BLUE), ("LIGHTS", GREEN)]

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(self.backgroundRole(), BG)
        self.setPalette(pal)
        self.tach = DialGauge(0, 8000, 1000, 5, 1000, "x1000 rpm", "rpm", red_from=6500)
        self.speedo = DialGauge(0, 240, 20, 4, 1, "km/h", "km/h", label_every=2)
        self.temp = ArcGauge("C", "H", "TEMP")
        self.fuel = ArcGauge("E", "F", "FUEL", warn_high=False)
        self.info = InfoDisplay()
        self.lamps = {name: Lamp(name, color) for name, color in self.LAMPS}

        centre = QVBoxLayout()
        centre.addWidget(self.info, 3)
        small = QHBoxLayout()
        small.addWidget(self.temp)
        small.addWidget(self.fuel)
        centre.addLayout(small, 2)
        lamp_row = QHBoxLayout()
        lamp_row.setSpacing(4)
        for lamp in self.lamps.values():
            lamp_row.addWidget(lamp)

        top = QHBoxLayout()
        top.addWidget(self.tach, 4)
        top.addLayout(centre, 2)
        top.addWidget(self.speedo, 4)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 8, 10, 8)
        outer.addLayout(top, 1)
        outer.addLayout(lamp_row)

    def update_from(self, s: dict, mil: int, ecu_mode: int) -> None:
        powered = s["key"] >= 2
        self.tach.set_value(s["rpm"] if powered or s["rpm"] > 0 else 0.0)
        self.speedo.set_value(abs(s["kmh"]) if powered else 0.0)
        self.temp.set_fraction(coolant_fraction(s["ect_c"]) if powered else 0.0)
        self.fuel.set_fraction(s["fuel_pct"] / 100.0 if powered else 0.0)
        gear = {-1: "R", 0: "N"}.get(int(s["gear"]), str(int(s["gear"])))
        self.info.gear = gear
        self.info.odometer = s["odometer_km"]
        self.info.trip = s["trip_km"]
        self.info.ambient = s["ambient_c"]
        if s["shift_refused"]:
            self.info.message, self.info.message_color = "PRESS CLUTCH TO SHIFT", AMBER
        elif s["key"] == 3 and not s["starter"]:
            self.info.message, self.info.message_color = "PRESS CLUTCH TO START", AMBER
        else:
            self.info.message = ""
        engine_off = s["rpm"] < 400
        bulb_check = powered and engine_off
        lamps = {
            "CHECK": mil if powered and not engine_off else (1 if bulb_check else 0),
            "EPC": 1 if bulb_check or (powered and s.get("epc", 0.0)) else 0,
            "BATT": 1 if powered and s["alt_a"] <= 0.0 else 0,
            "OIL": 1 if powered and engine_off else 0,
            "TEMP": 1 if powered and s["ect_c"] > 115.0 else 0,
            "BRAKE": 1 if powered and s["handbrake"] else 0,
            "FAN": 1 if powered and s["fan"] > 0 else 0,
            "LIGHTS": 1 if s["lights"] else 0,
        }
        for k, v in lamps.items():
            self.lamps[k].set_state(v)

    def tick(self) -> None:
        for w in (self.tach, self.speedo, self.temp, self.fuel):
            w.tick()
        for lamp in self.lamps.values():
            lamp.tick()
        self.info.update()
