# SPDX-License-Identifier: GPL-3.0-or-later
# PyInstaller spec for the ECU simulator: one self-contained executable.
# Build from the repository root:   pyinstaller packaging/ecusim.spec
import os
import sys

from PyInstaller.utils.hooks import collect_submodules

# Refuse to build an executable that would be missing its libraries (PyInstaller itself only warns)
_missing = []
for _mod in ("PySide6.QtWidgets", "PySide6.QtSvg", "can", "pyqtgraph", "numpy", "serial", "msgpack"):
    try:
        __import__(_mod)
    except Exception as _e:  # noqa: BLE001
        _missing.append(f"{_mod}  ({type(_e).__name__}: {_e})")
if _missing:
    raise SystemExit(
        "ecusim.spec: these packages are missing from the Python running PyInstaller:\n    "
        + "\n    ".join(_missing)
        + "\nInstall them into that environment (see packaging/build_*.sh / build_windows.ps1) and build again."
    )

ROOT = os.path.abspath(os.path.join(SPECPATH, ".."))

# python-can loads its interface backends by name at run time; ship all of them
hidden = collect_submodules("can.interfaces") + collect_submodules("ecusim") + ["msgpack"]
# candleLight / CANable on Windows and macOS goes through libusb (gs_usb + pyusb)
# jeepney: Bluetooth ELM327 emulation through BlueZ (Linux)
for optional in ("gs_usb", "usb", "jeepney"):
    try:
        __import__(optional)
        hidden += collect_submodules(optional)
    except ImportError:
        pass

excludes = [
    "tkinter", "matplotlib", "pytest", "IPython", "PyInstaller", "scipy", "sphinx", "babel", "docutils",
    "jinja2", "pygments", "PIL", "pyqtgraph.jupyter", "pyqtgraph.opengl", "pyqtgraph.examples",
    "PySide6.QtQuickWidgets", "PySide6.QtVirtualKeyboard",  # (sqlite3 stays: python-can imports it)
    "PySide6.QtWebEngineCore", "PySide6.QtWebEngineWidgets", "PySide6.QtWebEngineQuick",
    "PySide6.Qt3DCore", "PySide6.Qt3DRender", "PySide6.QtQuick", "PySide6.QtQml", "PySide6.QtQuick3D",
    "PySide6.QtMultimedia", "PySide6.QtCharts", "PySide6.QtDataVisualization", "PySide6.QtPdf",
    "PySide6.QtBluetooth", "PySide6.QtPositioning", "PySide6.QtLocation", "PySide6.QtSql",
    "PySide6.QtDesigner", "PySide6.QtHelp", "PySide6.QtNetworkAuth", "PySide6.QtRemoteObjects",
    "PySide6.QtSensors", "PySide6.QtSerialBus", "PySide6.QtTextToSpeech", "PySide6.QtWebSockets",
    "PySide6.QtSpatialAudio", "PySide6.QtGraphs",
]

a = Analysis(
    [os.path.join(SPECPATH, "launcher.py")],
    pathex=[ROOT],
    hiddenimports=hidden,
    excludes=excludes,
    noarchive=False,
)
pyz = PYZ(a.pure)
# Icons (regenerate with tools/make_icons.py): .ico embedded in the Windows .exe, .icns for the macOS app
ICON = {
    "win32": os.path.join(SPECPATH, "windows", "ecusim.ico"),
    "darwin": os.path.join(SPECPATH, "macos", "ecusim.icns"),
}.get(sys.platform)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    name="ecusim",
    console=True,  # keeps the headless/tester subcommands usable from a terminal
    upx=False,
    strip=sys.platform.startswith("linux"),
    runtime_tmpdir=None,
    icon=ICON,
)
if sys.platform == "darwin":
    app = BUNDLE(exe, name="ECU Simulator.app", icon=ICON, bundle_identifier="org.ecusim.app")
