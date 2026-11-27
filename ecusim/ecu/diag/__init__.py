"""On-board diagnostics: DTCs, monitors, freeze frame, Mode 06, IUMPR."""

from .diagnostics import MIL_FLASH, MIL_OFF, MIL_ON, Diagnostics

__all__ = ["Diagnostics", "MIL_OFF", "MIL_ON", "MIL_FLASH"]
