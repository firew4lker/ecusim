# SPDX-License-Identifier: GPL-3.0-or-later
"""Active-fault bookkeeping: enable/disable, severity, sensor mode, intermittency (DESIGN §8)."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING

from ..sensors.faults import SensorFault
from .catalog import BY_KEY, SENSOR, SEVERITY, TOGGLE

if TYPE_CHECKING:
    from ..sim import Simulator


@dataclass
class FaultSetting:
    key: str
    value: float | None = None  # severity faults (None = catalogue default)
    cylinder: int = 1  # 1..4 for per-cylinder faults
    mode: str = ""  # sensor faults
    amount: float | None = None  # sensor faults (None = catalogue default for the mode)
    intermittent: bool = False
    duty: float = 0.5  # fraction of each period the fault is present
    period_s: float = 10.0

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "FaultSetting":
        return FaultSetting(**{k: v for k, v in d.items() if k in FaultSetting.__dataclass_fields__})


class FaultManager:
    def __init__(self) -> None:
        self.active: dict[str, FaultSetting] = {}
        self._applied: dict[tuple[str, int], float] = {}
        self.version = 0

    # ------------------------------------------------------------------ editing
    def set(self, setting: FaultSetting) -> None:
        d = BY_KEY[setting.key]
        if d.kind == SENSOR and not setting.mode:
            setting.mode = d.modes[0]
        self.active[setting.key] = setting
        self.version += 1

    def clear(self, key: str) -> None:
        if key in self.active:
            del self.active[key]
            self.version += 1

    def clear_all(self) -> None:
        self.active.clear()
        self.version += 1

    def describe(self) -> list[dict]:
        return [s.to_dict() for s in self.active.values()]

    # ------------------------------------------------------------------ application
    def _value(self, s: FaultSetting) -> float:
        d = BY_KEY[s.key]
        if d.kind == TOGGLE:
            return 1.0
        return d.default if s.value is None else s.value

    def apply(self, sim: "Simulator") -> None:
        """Push the effective fault state into the plant and sensor hooks (called every step)."""
        t = sim.t
        wanted: dict[tuple[str, int], float] = {}
        sensor_faults: dict[str, SensorFault] = {}
        for s in self.active.values():
            if s.intermittent and (t % max(s.period_s, 0.1)) / max(s.period_s, 0.1) >= s.duty:
                continue
            d = BY_KEY[s.key]
            if d.kind == SENSOR:
                unit_scale = d.amounts.get(s.mode, ("", 0, 0, 0, 1.0))
                amount = unit_scale[3] if s.amount is None else s.amount
                sensor_faults[d.channel] = SensorFault(s.mode, amount * unit_scale[4])
            else:
                cyl = (s.cylinder - 1) if d.per_cylinder else 0
                wanted[(s.key, cyl)] = self._value(s)
        # restore anything no longer wanted
        for k in list(self._applied):
            if k not in wanted:
                d = BY_KEY[k[0]]
                d.apply(sim, 0.0 if d.kind == TOGGLE else d.nominal, k[1])
                del self._applied[k]
        for k, v in wanted.items():
            if self._applied.get(k) != v:
                BY_KEY[k[0]].apply(sim, v, k[1])
                self._applied[k] = v
        sim.sensors.faults = sensor_faults

    def reset_hooks(self, sim: "Simulator") -> None:
        """Put every hook this manager touched back to nominal."""
        for k in list(self._applied):
            d = BY_KEY[k[0]]
            d.apply(sim, 0.0 if d.kind == TOGGLE else d.nominal, k[1])
        self._applied.clear()
        sim.sensors.faults = {}


__all__ = ["FaultManager", "FaultSetting", "SEVERITY", "TOGGLE", "SENSOR"]
