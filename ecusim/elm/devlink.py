# SPDX-License-Identifier: GPL-3.0-or-later
"""A permanent name in /dev for the virtual serial port (Linux, DESIGN §7.7.1).

Linux programs look for serial ports in /dev, which only root can write to. A one-time systemd-tmpfiles rule
creates ``/dev/<name>`` at every boot, pointing at the per-user link that ecusim re-points to the current
pseudo-terminal each time the port is switched on:

    /dev/ttyELM327  →  ~/.local/share/ecusim/ttyELM327  →  /dev/pts/N
    (root, once)       (ecusim, every start)

ecusim never runs anything as root: it only shows the commands for the user to run.
"""

from __future__ import annotations

import os
import re
import shlex
import shutil
import sys
from pathlib import Path

DEFAULT_NAME = "ttyELM327"
RULE_FILE = "/etc/tmpfiles.d/ecusim-elm327-{name}.conf"
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")


def supported() -> str:
    """Why a /dev name can't be set up on this system, or "" if it can."""
    if not sys.platform.startswith("linux"):
        return "only needed on Linux"
    return ""


def valid_name(name: str) -> bool:
    return bool(_NAME_RE.match(name))


def dev_path(name: str, dev_dir: Path | str = "/dev") -> Path:
    return Path(dev_dir) / name


def state(name: str, target: Path | str, dev_dir: Path | str = "/dev") -> str:
    """"ok" (the /dev name leads to ``target``), "missing", "other" (points somewhere else) or "taken"
    (a real device or file of that name exists)."""
    p = dev_path(name, dev_dir)
    if p.is_symlink():
        return "ok" if os.readlink(p) == str(target) else "other"
    if p.exists():
        return "taken"
    return "missing"


def setup_commands(name: str, target: Path | str) -> tuple[str, str]:
    """(commands that create the permanent /dev name, commands that remove it)."""
    if not valid_name(name):
        raise ValueError(f"not a valid device name: {name!r}")
    rule_file = RULE_FILE.format(name=name)
    dev = f"/dev/{name}"
    if shutil.which("systemd-tmpfiles"):
        rule = f"L+ {dev} - - - - {target}"
        install = (f"echo {shlex.quote(rule)} | sudo tee {rule_file}\n"
                   f"sudo systemd-tmpfiles --create {rule_file}")
        remove = f"sudo rm -f {rule_file} {dev}"
    else:  # no systemd: the link lasts until the next reboot
        install = f"sudo ln -sfn {shlex.quote(str(target))} {dev}"
        remove = f"sudo rm -f {dev}"
    return install, remove


def survives_reboot() -> bool:
    return shutil.which("systemd-tmpfiles") is not None
