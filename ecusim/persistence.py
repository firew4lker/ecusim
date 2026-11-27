# SPDX-License-Identifier: GPL-3.0-or-later
"""Keep-alive memory (KAM) and non-volatile memory (NVM) on disk (DESIGN §10).

kam.json: adaptives, DTCs, freeze frame, monitors, Mode 06, counters, odometer, fuel, temperatures.
          Survives key-off and app restarts; wiped by "battery disconnect".
nvm.json: permanent DTCs and vehicle information. Survives a battery disconnect.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

from .obd.vehicle_info import VehicleInfo
from .sim import Simulator

VERSION = 1
COOLANT_SOAK_TAU_S = 6000.0
CAT_SOAK_TAU_S = 1200.0


def default_state_dir() -> Path:
    if sys.platform == "win32":
        base = Path(os.environ.get("APPDATA", Path.home() / "AppData" / "Roaming"))
    elif sys.platform == "darwin":
        base = Path.home() / "Library" / "Application Support"
    else:
        base = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share"))
    return base / "ecusim"


def _write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=1, sort_keys=True))
    os.replace(tmp, path)


def _read_json(path: Path) -> dict | None:
    try:
        data = json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None
    return data if isinstance(data, dict) and data.get("version") == VERSION else None


def capture(sim: Simulator, info: VehicleInfo) -> tuple[dict, dict]:
    """Snapshot KAM and NVM contents. Call on the simulation thread."""
    p = sim.plant
    kam = {
        "version": VERSION,
        "saved_at": time.time(),
        "ecu": sim.ecu.export_kam(),
        "vehicle": {"odometer_m": p.vehicle.odometer_m, "trip_m": p.vehicle.trip_m, "fuel_l": p.fuel_l},
        "thermal": {
            "ambient_c": sim.env.ambient_c,
            "baro_kpa": sim.env.baro_kpa,
            "coolant_c": p.thermal.coolant_c,
            "oil_c": p.thermal.oil_c,
            "cat_c": p.exhaust.cat_c,
        },
    }
    nvm = {
        "version": VERSION,
        "permanent": sim.ecu.diag.dtc.permanent,
        "vehicle_info": {"vin": info.vin, "calid": info.calid, "ecu_name": info.ecu_name, "region": info.region},
    }
    return kam, nvm


def soaked(temp_c: float, ambient_c: float, elapsed_s: float, tau_s: float) -> float:
    return ambient_c + (temp_c - ambient_c) * math.exp(-max(0.0, elapsed_s) / tau_s)


class StateStore:
    def __init__(self, directory: Path | str):
        self.dir = Path(directory)
        self.kam_path = self.dir / "kam.json"
        self.nvm_path = self.dir / "nvm.json"

    def save(self, kam: dict, nvm: dict) -> None:
        _write_json(self.kam_path, kam)
        _write_json(self.nvm_path, nvm)

    def load(self) -> tuple[dict | None, dict | None]:
        return _read_json(self.kam_path), _read_json(self.nvm_path)

    def wipe_kam(self) -> None:
        try:
            self.kam_path.unlink()
        except FileNotFoundError:
            pass

    @staticmethod
    def vehicle_info(nvm: dict | None) -> VehicleInfo:
        if not nvm or "vehicle_info" not in nvm:
            return VehicleInfo()
        try:
            return VehicleInfo(**nvm["vehicle_info"])
        except (TypeError, ValueError):
            return VehicleInfo()

    @staticmethod
    def build_sim(kam: dict | None, nvm: dict | None, ambient_c: float | None = None,
                  now: float | None = None) -> Simulator:
        """New simulator restored from KAM/NVM, with the engine soaked for the real time elapsed."""
        th = (kam or {}).get("thermal", {})
        amb = ambient_c if ambient_c is not None else float(th.get("ambient_c", 15.0))
        sim = Simulator(ambient_c=amb, baro_kpa=float(th.get("baro_kpa", 101.3)))
        if kam:
            elapsed = (now or time.time()) - float(kam.get("saved_at", 0.0))
            coolant = soaked(float(th.get("coolant_c", amb)), amb, elapsed, COOLANT_SOAK_TAU_S)
            if coolant > amb + 1.0:
                sim.plant.set_warm(
                    coolant_c=coolant,
                    oil_c=soaked(float(th.get("oil_c", amb)), amb, elapsed, COOLANT_SOAK_TAU_S),
                    cat_c=soaked(float(th.get("cat_c", amb)), amb, elapsed, CAT_SOAK_TAU_S),
                )
                sim.sensors = type(sim.sensors)(sim.rng)
                sim.readings = sim.sensors.update(sim.dt, sim.plant, sim.ecu.act, sim.driver)
            veh = kam.get("vehicle", {})
            sim.plant.vehicle.odometer_m = float(veh.get("odometer_m", 0.0))
            sim.plant.vehicle.trip_m = float(veh.get("trip_m", 0.0))
            sim.plant.fuel_l = float(veh.get("fuel_l", sim.plant.fuel_l))
            sim.ecu.import_kam(kam.get("ecu", {}))
        if nvm:
            dtc = sim.ecu.diag.dtc
            for code in nvm.get("permanent", []):
                if code in dtc.records:
                    dtc.records[code].permanent = True
                else:
                    from .ecu.diag.dtc import DtcRecord

                    dtc.records[code] = DtcRecord(code, permanent=True)
            dtc.version += 1
        return sim
