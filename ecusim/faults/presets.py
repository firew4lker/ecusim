# SPDX-License-Identifier: GPL-3.0-or-later
"""Scenario presets (DESIGN §8.2): environment + engine state + faults in one click."""

from __future__ import annotations

import json
from pathlib import Path

BUILTIN: list[dict] = [
    {"name": "Healthy car, cold morning (5 °C)", "ambient": 5.0, "soak": "cold", "faults": []},
    {"name": "Freshly cleared codes (readiness incomplete)", "faults": [], "clear_codes": True},
    {"name": "Vacuum leak at idle", "soak": "warm", "faults": [{"key": "vacuum_leak"}]},
    {"name": "Tired catalyst", "soak": "warm", "faults": [{"key": "catalyst"}]},
    {"name": "Misfire cylinder 3 under load", "soak": "warm", "faults": [{"key": "plug", "cylinder": 3, "value": 85}]},
    {"name": "Stuck-open thermostat in winter (0 °C)", "ambient": 0.0, "soak": "cold",
     "faults": [{"key": "thermostat_open"}]},
    {"name": "Fuel cap left loose", "soak": "cold", "faults": [{"key": "gas_cap"}]},
    {"name": "Dead coolant temperature sensor", "faults": [{"key": "s_ect", "mode": "open"}]},
    {"name": "Throttle sensors disagree (limp home)", "soak": "warm",
     "faults": [{"key": "s_tps_b", "mode": "offset", "amount": 15}]},
    {"name": "Intermittent crank sensor", "soak": "warm",
     "faults": [{"key": "s_ckp", "mode": "open", "intermittent": True, "duty": 0.1, "period_s": 30}]},
    {"name": "Lazy upstream O2 sensor", "soak": "warm",
     "faults": [{"key": "s_wb", "mode": "slow", "amount": 1.5}]},
]


class PresetStore:
    def __init__(self, path: Path | None):
        self.path = path
        self.user: list[dict] = []
        if path is not None and path.exists():
            try:
                data = json.loads(path.read_text())
                self.user = [p for p in data if isinstance(p, dict) and "name" in p]
            except (OSError, json.JSONDecodeError):
                self.user = []

    def all(self) -> list[dict]:
        return BUILTIN + self.user

    def get(self, name: str) -> dict | None:
        return next((p for p in self.all() if p["name"] == name), None)

    def save(self, preset: dict) -> None:
        self.user = [p for p in self.user if p["name"] != preset["name"]] + [preset]
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(self.user, indent=1))

    def delete(self, name: str) -> None:
        self.user = [p for p in self.user if p["name"] != name]
        if self.path is not None:
            self.path.write_text(json.dumps(self.user, indent=1))
