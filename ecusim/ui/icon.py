"""Application icon (single source for the window icon, the Linux .svg, the Windows .ico and macOS .icns).

Regenerate the packaging files with:  python tools/make_icons.py
"""

from __future__ import annotations

ICON_SVG = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 128 128" width="128" height="128">
  <rect x="4" y="4" width="120" height="120" rx="24" fill="#181b21"/>
  <circle cx="64" cy="68" r="44" fill="#232831" stroke="#4a5263" stroke-width="3"/>
  <path d="M 31.6 99.4 A 44 44 0 1 1 96.4 99.4" fill="none" stroke="#e1e4eb" stroke-width="5" stroke-linecap="round" stroke-dasharray="2 10.6"/>
  <path d="M 103.5 81 A 44 44 0 0 1 96.4 99.4" fill="none" stroke="#dc2828" stroke-width="7" stroke-linecap="round"/>
  <line x1="64" y1="68" x2="92" y2="40" stroke="#ff5a28" stroke-width="6" stroke-linecap="round"/>
  <circle cx="64" cy="68" r="8" fill="#3c4048"/>
  <rect x="38" y="92" width="52" height="18" rx="4" fill="#0a0c10" stroke="#3c4250"/>
  <text x="64" y="106" text-anchor="middle" font-family="DejaVu Sans Mono, monospace" font-size="13" font-weight="bold" fill="#ffb000">ECU</text>
</svg>
"""

# Simplified drawing for 16-48 px: no lettering or tick marks, heavier strokes.
ICON_SVG_SMALL = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 128 128" width="128" height="128">
  <rect x="2" y="2" width="124" height="124" rx="26" fill="#181b21"/>
  <path d="M 24 100 A 50 50 0 1 1 104 100" fill="none" stroke="#e1e4eb" stroke-width="12" stroke-linecap="round"/>
  <path d="M 112 76 A 50 50 0 0 1 104 100" fill="none" stroke="#dc2828" stroke-width="16" stroke-linecap="round"/>
  <line x1="64" y1="72" x2="98" y2="38" stroke="#ff5a28" stroke-width="14" stroke-linecap="round"/>
  <circle cx="64" cy="72" r="13" fill="#ffb000"/>
</svg>
"""

SMALL_MAX = 48  # sizes up to this use the simplified drawing (lettering unreadable below 64 px)
SIZES = (16, 20, 24, 32, 40, 48, 64, 96, 128, 256)


def render(size: int):
    """Render the icon at ``size`` px as a QImage (needs a QGuiApplication)."""
    from PySide6.QtCore import QByteArray, Qt
    from PySide6.QtGui import QImage, QPainter
    from PySide6.QtSvg import QSvgRenderer

    svg = ICON_SVG_SMALL if size <= SMALL_MAX else ICON_SVG
    renderer = QSvgRenderer(QByteArray(svg.encode()))
    img = QImage(size, size, QImage.Format.Format_ARGB32_Premultiplied)
    img.fill(Qt.GlobalColor.transparent)
    p = QPainter(img)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
    renderer.render(p)
    p.end()
    return img


def app_icon():
    """QIcon with every size, so window managers and taskbars pick a crisp one."""
    from PySide6.QtGui import QIcon, QPixmap

    icon = QIcon()
    for s in SIZES:
        icon.addPixmap(QPixmap.fromImage(render(s)))
    return icon
