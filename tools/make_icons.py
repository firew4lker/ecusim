#!/usr/bin/env python3
"""Generate the icon files used for packaging from ecusim/ui/icon.py:

    packaging/linux/ecusim.svg      Linux desktop entry (Arch package)
    packaging/windows/ecusim.ico    Windows .exe (16-256 px, PNG-compressed entries)
    packaging/macos/ecusim.icns     macOS .app bundle

    python tools/make_icons.py
"""

from __future__ import annotations

import os
import struct
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PySide6.QtCore import QBuffer, QByteArray, QIODevice  # noqa: E402
from PySide6.QtGui import QGuiApplication  # noqa: E402

from ecusim.ui.icon import ICON_SVG, render  # noqa: E402

ICO_SIZES = (16, 20, 24, 32, 40, 48, 64, 96, 128, 256)


def png_bytes(size: int) -> bytes:
    ba = QByteArray()
    buf = QBuffer(ba)
    buf.open(QIODevice.OpenModeFlag.WriteOnly)
    render(size).save(buf, "PNG")
    buf.close()
    return bytes(ba)


def write_ico(path: Path, sizes=ICO_SIZES) -> None:
    """ICO container with PNG-compressed images (supported since Windows Vista)."""
    images = [(s, png_bytes(s)) for s in sizes]
    header = struct.pack("<HHH", 0, 1, len(images))
    offset = 6 + 16 * len(images)
    entries = b""
    data = b""
    for s, png in images:
        dim = 0 if s >= 256 else s  # 0 means 256 in the directory entry
        entries += struct.pack("<BBBBHHII", dim, dim, 0, 0, 1, 32, len(png), offset)
        offset += len(png)
        data += png
    path.write_bytes(header + entries + data)


def main() -> int:
    app = QGuiApplication.instance() or QGuiApplication([])  # noqa: F841 (fonts for the SVG text)
    (ROOT / "packaging" / "linux").mkdir(parents=True, exist_ok=True)
    (ROOT / "packaging" / "windows").mkdir(parents=True, exist_ok=True)
    (ROOT / "packaging" / "macos").mkdir(parents=True, exist_ok=True)

    svg = ROOT / "packaging" / "linux" / "ecusim.svg"
    svg.write_text(ICON_SVG)
    ico = ROOT / "packaging" / "windows" / "ecusim.ico"
    write_ico(ico)
    icns = ROOT / "packaging" / "macos" / "ecusim.icns"
    if not render(512).save(str(icns), "ICNS"):
        raise SystemExit("Qt could not write the .icns file")
    for f in (svg, ico, icns):
        print(f"wrote {f.relative_to(ROOT)} ({f.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
