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
                   help="virtual serial port (Linux/macOS) with its link at PATH (default: in the ecusim data "
                        "folder; on Linux also /dev/ttyELM327 after a one-time setup that ecusim prints)")
    g.add_argument("--elm-serial", metavar="PORT[@BAUD]",
                   help="answer on an existing serial port, e.g. COM11 or /dev/ttyUSB0@38400")
    g.add_argument("--elm-bt", metavar="NAME", nargs="?", const="OBDII",
                   help="Bluetooth serial adapter (Linux, BlueZ); pair from the phone first")
    g.add_argument("--no-elm", action="store_true", help="start with every ELM327 transport off")


def hints(mgr: ElmManager) -> list[str]:
    """Advice to print at start-up: how to give the virtual serial port its permanent /dev name."""
    from . import devlink

    t = mgr.transports.get("pty")
    if t is None or not t.running or devlink.supported():
        return []
    name = mgr.settings["pty"]["dev_name"] or devlink.DEFAULT_NAME
    st = devlink.state(name, t.link)
    if st == "ok":
        return [f"ELM327 virtual serial port: programs can open /dev/{name}"]
    if st == "taken":
        return [f"ELM327 virtual serial port: /dev/{name} is another device; choose another name in the ELM327 tab"]
    install, _ = devlink.setup_commands(name, t.link)
    return [f"ELM327 virtual serial port: {t.link}",
            f"Most Linux programs only look in /dev. To make it /dev/{name}, run once"
            + (" (it stays after reboots):" if devlink.survives_reboot() else " (until the next reboot):"),
            *("    " + line for line in install.splitlines())]


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
