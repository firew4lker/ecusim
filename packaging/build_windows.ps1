# SPDX-License-Identifier: GPL-3.0-or-later
# Build dist\ecusim.exe on Windows (PowerShell). Needs 64-bit Python 3.11 or newer.
#   powershell -ExecutionPolicy Bypass -File packaging\build_windows.ps1
# Another Python can be chosen with:  $env:PYTHON = "C:\Path\To\python.exe"
#
# Every step is checked: the script stops with a clear message instead of producing an .exe without its
# libraries, and finishes by starting the built .exe once (offscreen) to prove it works.

$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")

$onWindows = $env:OS -eq "Windows_NT"
$bin = if ($onWindows) { "Scripts" } else { "bin" }  # also runs under PowerShell on Linux, for testing
$exe = if ($onWindows) { ".exe" } else { "" }

function Invoke-Checked {
    param([string]$File, [string[]]$Arguments, [string]$What)
    & $File @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$What failed (exit code $LASTEXITCODE)." }
}

$checkPython = "import sys, struct; v = sys.version_info; b = struct.calcsize('P') * 8; " +
               "print('Python %d.%d.%d, %d-bit (%s)' % (v[0], v[1], v[2], b, sys.executable)); " +
               "sys.exit(0 if v >= (3, 11) and b == 64 else 3)"

# 1. The Python used to create the build environment
$py = if ($env:PYTHON) { $env:PYTHON } elseif ($onWindows) { "python" } else { "python3" }
if (-not (Get-Command $py -ErrorAction SilentlyContinue)) {
    throw "Python was not found ('$py'). Install 64-bit Python 3.11 or newer from python.org (tick 'Add python.exe to PATH'), " +
          "or point `$env:PYTHON at its python.exe."
}
& $py -c $checkPython
if ($LASTEXITCODE -ne 0) {
    throw "64-bit Python 3.11 or newer is required (PySide6 has no 32-bit Windows build). " +
          "Install it from python.org, or point `$env:PYTHON at a suitable python.exe."
}

# 2. Build environment
$venvPy = Join-Path ".venv-build" (Join-Path $bin "python$exe")
if (-not (Test-Path $venvPy)) {
    Invoke-Checked $py @("-m", "venv", ".venv-build") "Creating .venv-build"
}
& $venvPy -c $checkPython
if ($LASTEXITCODE -ne 0) {
    throw ".venv-build was made with an unsuitable Python. Delete the .venv-build folder and run this script again."
}

# 3. Dependencies (required ones in one step, so a failure stops the build)
Invoke-Checked $venvPy @("-m", "pip", "install", "--upgrade", "pip") "Upgrading pip"
Invoke-Checked $venvPy @("-m", "pip", "install", "--upgrade",
    "python-can>=4.3", "msgpack", "pyserial", "PySide6", "pyqtgraph", "numpy", "pyinstaller") "Installing dependencies"

# Optional: CANable/candleLight through libusb (the gs_usb backend). PCAN, Kvaser and slcan work without it.
& $venvPy -m pip install --upgrade gs_usb pyusb libusb-package
if ($LASTEXITCODE -ne 0) {
    Write-Warning "Optional USB packages could not be installed: --interface gs_usb will not work; everything else will."
}

Invoke-Checked $venvPy @("-c",
    "import PySide6, can, pyqtgraph, numpy, serial, msgpack; print('Dependencies OK: PySide6', PySide6.__version__, '/ python-can', can.__version__)") `
    "Checking dependencies"

# 4. Build
Invoke-Checked $venvPy @("-m", "PyInstaller", "--noconfirm", "--clean", "--log-level", "WARN",
    "--workpath", "build", "--distpath", "dist", "packaging\ecusim.spec".Replace("\", [IO.Path]::DirectorySeparatorChar)) "PyInstaller"

# 5. Smoke test: start the built program once without a window
$built = Join-Path "dist" "ecusim$exe"
$env:QT_QPA_PLATFORM = "offscreen"
try {
    Invoke-Checked $built @("--version") "Running $built --version"
    Invoke-Checked $built @("--no-can", "--no-state", "--quit-after", "1") "Starting the built app"
} finally {
    Remove-Item Env:QT_QPA_PLATFORM
}
Write-Host "Build OK:" -ForegroundColor Green
Get-Item $built
