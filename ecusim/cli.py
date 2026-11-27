"""The ``ecusim`` command.

    ecusim                      desktop app (default)
    ecusim headless [options]   ECU on a CAN bus with a text console, no window
    ecusim tester [options]     generic OBD-II tester
    ecusim --version
"""

from __future__ import annotations

import sys


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if args and args[0] in ("--version", "-V"):
        from . import __version__

        print(f"ecusim {__version__}")
        return 0
    if args and args[0] in ("help", "--commands"):
        print(__doc__)
        return 0
    if args and args[0] == "headless":
        from .__main__ import main as headless

        return headless(args[1:])
    if args and args[0] == "tester":
        from .tester_cli import main as tester

        return tester(args[1:])
    from .ui.app import main as ui

    return ui(args)


if __name__ == "__main__":
    sys.exit(main())
