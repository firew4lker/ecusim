#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Build dist/ecusim (command line) and "dist/ECU Simulator.app" on macOS.
set -euo pipefail
cd "$(dirname "$0")/.."
PY=${PYTHON:-python3}
[ -x .venv-build/bin/python ] || "$PY" -m venv .venv-build
.venv-build/bin/pip install -q --upgrade "python-can>=4.3" msgpack pyserial gs_usb pyusb PySide6 pyqtgraph pyinstaller
.venv-build/bin/pyinstaller --noconfirm --clean --log-level WARN --workpath build --distpath dist packaging/ecusim.spec
ls -la dist
