# SPDX-License-Identifier: GPL-3.0-or-later
"""Command-line switches for the emulated ELM327 (DESIGN §7.7.2), shared by the desktop app and
``ecusim headless``. A switch turns that transport on for this session only; saved settings stay."""

from __future__ import annotations

import argparse

from .manager import KINDS, ElmManager


def add_arguments(ap: argparse.ArgumentParser) -> None:
    g = ap.add_argument_group("ELM327 adapter emulation (diagnostic apps without CAN hardware)")
    g.add_argument("--elm-tcp", metavar="[HOST:]PORT", nargs="?", const="35000",
                   help="Wi-Fi style adapter on a TCP port (default 35000, this computer only; "
                        "HOST 0.0.0.0 lets phones on the network connect)")
    g.add_argument("--elm-pty", metavar="PATH", nargs="?", const="",
                   help="virtual serial port (Linux/macOS), reachable at PATH (default /dev/ttyELM327 if "
                        "writable, otherwise in the ecusim data folder)")
    g.add_argument("--elm-serial", metavar="PORT[@BAUD]",
                   help="answer on an existing serial port, e.g. COM11 or /dev/ttyUSB0@38400")
    g.add_argument("--elm-bt", metavar="NAME", nargs="?", const="OBDII",
                   help="Bluetooth serial adapter (Linux, BlueZ); pair from the phone first")
    g.add_argument("--no-elm", action="store_true", help="start with every ELM327 transport off")


def apply(mgr: ElmManager, args: argparse.Namespace) -> list[str]:
    """Apply the switches and start the enabled transports; returns error messages."""
    if args.no_elm:
        for kind in KINDS:
            mgr.settings[kind]["enabled"] = False
            mgr.session_only.add(kind)
    if args.elm_tcp is not None:
        host, _, port = args.elm_tcp.rpartition(":")
        mgr.settings["tcp"].update(port=int(port), network=host not in ("", "127.0.0.1", "localhost"))
    if args.elm_pty is not None:
        mgr.settings["pty"]["link"] = args.elm_pty
    if args.elm_serial is not None:
        port, _, baud = args.elm_serial.partition("@")
        mgr.settings["serial"].update(port=port, baud=int(baud) if baud else 38400)
    if args.elm_bt is not None:
        mgr.settings["bt"]["name"] = args.elm_bt
    for kind, value in (("tcp", args.elm_tcp), ("pty", args.elm_pty), ("serial", args.elm_serial),
                        ("bt", args.elm_bt)):
        if value is not None:
            mgr.settings[kind]["enabled"] = True
            mgr.session_only.add(kind)
    return mgr.start_enabled()
