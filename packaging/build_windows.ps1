# Build dist\ecusim.exe on Windows (PowerShell). Requires Python 3.11+ on PATH.
#   powershell -ExecutionPolicy Bypass -File packaging\build_windows.ps1
$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")
if (-not (Test-Path ".venv-build\Scripts\python.exe")) { python -m venv .venv-build }
.venv-build\Scripts\pip install -q --upgrade "python-can>=4.3" msgpack pyserial gs_usb pyusb libusb-package PySide6 pyqtgraph pyinstaller
.venv-build\Scripts\pyinstaller --noconfirm --clean --log-level WARN --workpath build --distpath dist packaging\ecusim.spec
Get-Item dist\ecusim.exe
