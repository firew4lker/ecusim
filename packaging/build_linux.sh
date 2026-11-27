#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Build the self-contained Linux binary dist/ecusim.
# Uses a dedicated virtual environment (no system site-packages) so only runtime dependencies are bundled.
set -euo pipefail
cd "$(dirname "$0")/.."

PY=${PYTHON:-python3}
VENV=.venv-build
if [ ! -x "$VENV/bin/python" ]; then
    "$PY" -m venv "$VENV"
fi
"$VENV/bin/pip" install -q --upgrade "python-can>=4.3" msgpack pyserial gs_usb PySide6 pyqtgraph pyinstaller

# Qt plugins the app never uses but which drag in large dependency trees:
#  - GTK3 platform theme: libgtk-3, a second ICU, cairo, ... (the app uses its own Fusion style)
#  - virtual keyboard input context: QtQuick / Qml
PLUGINS=$("$VENV/bin/python" -c "import PySide6, os; print(os.path.join(os.path.dirname(PySide6.__file__), 'Qt', 'plugins'))")
rm -f "$PLUGINS/platformthemes/libqgtk3.so" "$PLUGINS/platforminputcontexts/libqtvirtualkeyboardplugin.so"

"$VENV/bin/pyinstaller" --noconfirm --clean --log-level WARN --workpath build --distpath dist packaging/ecusim.spec
ls -la dist/ecusim
