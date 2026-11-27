#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Minimal generic OBD-II tester. Thin wrapper around ``ecusim.tester_cli``; see ``--help``."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ecusim.tester_cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
