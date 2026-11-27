# SPDX-License-Identifier: GPL-3.0-or-later
"""Desktop application entry point.

    python -m ecusim.ui                                   # UDP multicast CAN (no hardware)
    python -m ecusim.ui --interface socketcan --channel vcan0
    python -m ecusim.ui --interface slcan --channel /dev/ttyACM0
    python -m ecusim.ui --no-can
    python -m ecusim.ui --no-can --elm-tcp --elm-pty      # ELM327 adapter emulation only
"""

from __future__ import annotations

import argparse
import os
import sys


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m ecusim.ui", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interface", default="udp_multicast")
    ap.add_argument("--channel", default=None)
    ap.add_argument("--bitrate", type=int, default=500_000)
    ap.add_argument("--tty-baudrate", type=int, default=None, help="serial speed for slcan on ttyUSB/COM ports")
    ap.add_argument("--29bit", dest="extended", action="store_true")
    ap.add_argument("--no-can", action="store_true", help="start without connecting to a CAN bus")
    ap.add_argument("--ambient", type=float, default=None,
                    help="ambient °C (default: last session's, or 15)")
    ap.add_argument("--state-dir", default=None, help="where ECU memory is kept (default: per-user data dir)")
    ap.add_argument("--no-state", action="store_true", help="start fresh and do not save ECU memory")
    ap.add_argument("--time-scale", type=float, default=1.0)
    from ..elm.cli_args import add_arguments as add_elm_arguments

    add_elm_arguments(ap)
    ap.add_argument("--quit-after", type=float, default=None, help=argparse.SUPPRESS)  # smoke tests
    ap.add_argument("--screenshot", default=None, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    from PySide6.QtWidgets import QApplication

    from ..obd.can_transport import CanConfig
    from ..persistence import default_state_dir
    from .controller import SimController
    from .main_window import MainWindow
    from .panels import DEFAULT_CHANNELS

    app = QApplication(sys.argv[:1])
    app.setApplicationName("ECU Simulator")
    app.setDesktopFileName("ecusim")  # matches ecusim.desktop: icon in the taskbar / Wayland
    from .icon import app_icon

    app.setWindowIcon(app_icon())  # title bar and taskbar on every platform
    app.setStyle("Fusion")
    state_dir = None if args.no_state else (args.state_dir or default_state_dir())
    ctl = SimController(ambient_c=args.ambient, time_scale=args.time_scale, state_dir=state_dir)
    if not args.no_can:
        channel = args.channel or DEFAULT_CHANNELS.get(args.interface, "")
        try:
            ctl.connect_can(CanConfig(args.interface, channel, args.bitrate, args.extended,
                                      tty_baudrate=args.tty_baudrate))
        except Exception as e:
            print(f"CAN not connected: {e}", file=sys.stderr)
    from ..elm.cli_args import apply as apply_elm

    for err in apply_elm(ctl.elm, args):
        print(err, file=sys.stderr)
    win = MainWindow(ctl)
    win.can.sync_from_controller()
    win.elm.sync()
    win.show()
    if args.quit_after is not None:
        from PySide6.QtCore import QTimer

        def finish() -> None:
            if args.screenshot:
                win.grab().save(args.screenshot)
            win.close()
            app.quit()

        QTimer.singleShot(int(args.quit_after * 1000), finish)
    ret = app.exec()
    # Everything that matters already happened in MainWindow.closeEvent (simulation and CAN threads
    # stopped, ECU memory saved). Skip PySide6's interpreter-exit teardown, which can double-delete
    # widgets and crash (seen with PySide6 6.12).
    ctl.shutdown()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(ret)


if __name__ == "__main__":
    raise SystemExit(main())
