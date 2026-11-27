"""Glue between the UI and the simulation: owns the simulator, real-time runner, OBD server, CAN link
and the persisted ECU memory.

Qt-free so it can be scripted and tested without a display.
"""

from __future__ import annotations

import math
import threading
from pathlib import Path

from ..ecu.ecm import ECM
from ..faults.manager import FaultSetting
from ..faults.presets import PresetStore
from ..obd.can_transport import CanConfig, EcuCanServer
from ..obd.server import ObdServer
from ..obd.vehicle_info import VehicleInfo
from ..persistence import CAT_SOAK_TAU_S, COOLANT_SOAK_TAU_S, StateStore, capture, soaked
from ..runtime import RealtimeRunner
from ..sim import Simulator


class SimController:
    def __init__(self, ambient_c: float | None = None, baro_kpa: float = 101.3, info: VehicleInfo | None = None,
                 time_scale: float = 1.0, state_dir: Path | str | None = None):
        self.store = StateStore(state_dir) if state_dir is not None else None
        kam = nvm = None
        if self.store is not None:
            kam, nvm = self.store.load()
        self.info = info or StateStore.vehicle_info(nvm)
        if kam or nvm:
            self.sim = StateStore.build_sim(kam, nvm, ambient_c)
        else:
            self.sim = Simulator(ambient_c=15.0 if ambient_c is None else ambient_c, baro_kpa=baro_kpa)
        self.sim.ecu.region = self.info.region
        self.runner = RealtimeRunner(self.sim, time_scale)
        self.obd = ObdServer(lambda: self.runner.diag_source(), self.info, on_clear=self._mode04)
        self.can: EcuCanServer | None = None
        self.can_cfg: CanConfig | None = None
        self.can_error = ""
        self.fault_settings: dict[str, FaultSetting] = {}  # UI-side mirror of the active faults
        self.presets = PresetStore(Path(state_dir) / "presets.json" if state_dir is not None else None)
        self.runner.start()

    # ------------------------------------------------------------------ state
    @property
    def latest(self) -> dict:
        return self.runner.latest

    @property
    def latest_diag(self) -> dict:
        return self.runner.latest_diag

    @property
    def current_sim(self) -> Simulator:
        return self.runner.sim

    def call(self, fn, timeout: float = 2.0):
        """Run ``fn(sim)`` on the simulation thread and return its result."""
        done = threading.Event()
        box: dict = {}

        def wrapper(sim: Simulator) -> None:
            try:
                box["v"] = fn(sim)
            finally:
                done.set()

        self.runner.submit(wrapper)
        if not done.wait(timeout):
            raise TimeoutError("simulation thread did not respond")
        return box.get("v")

    # ------------------------------------------------------------------ CAN
    def connect_can(self, cfg: CanConfig) -> None:
        self.disconnect_can()
        srv = EcuCanServer(cfg, self.obd.handle, self.runner.powered)
        try:
            srv.start()
        except Exception as e:  # python-can raises many types depending on the backend
            self.can_error = f"{type(e).__name__}: {e}"
            raise
        self.can, self.can_cfg, self.can_error = srv, cfg, ""

    def disconnect_can(self) -> None:
        if self.can is not None:
            self.can.stop()
        self.can = None

    # ------------------------------------------------------------------ driver
    def set_driver(self, **fields) -> None:
        def apply(s: Simulator) -> None:
            for k, v in fields.items():
                setattr(s.driver, k, v)

        self.runner.submit(apply)

    def set_key(self, key: str) -> None:
        self.set_driver(key=key)

    def shift(self, gear: int) -> None:
        self.set_driver(gear=gear)

    # ------------------------------------------------------------------ environment
    def set_ambient(self, c: float) -> None:
        self.runner.submit(lambda s: setattr(s.env, "ambient_c", c))

    def set_baro(self, kpa: float) -> None:
        self.runner.submit(lambda s: setattr(s.env, "baro_kpa", kpa))

    def set_grade(self, pct: float) -> None:
        self.runner.submit(lambda s: setattr(s.plant.vehicle, "grade_pct", pct))

    def set_fuel_pct(self, pct: float) -> None:
        self.runner.submit(lambda s: setattr(s.plant, "fuel_l", s.plant.tank_l * pct / 100.0))

    def set_time_scale(self, x: float) -> None:
        self.runner.time_scale = x

    def set_paused(self, paused: bool) -> None:
        self.runner.paused = paused

    # ------------------------------------------------------------------ diagnostics
    def _mode04(self) -> None:
        """Mode 04 from a scan tool (already validated by the server)."""
        self.runner.submit(lambda s: s.ecu.diag.clear())

    def clear_codes(self) -> None:
        self.runner.submit(lambda s: s.ecu.diag.clear())

    def set_monitor_force(self, key: str, mode: str) -> None:
        self.runner.submit(lambda s: setattr(s.ecu.diag.monitor(key), "force", mode))

    def battery_disconnect(self) -> None:
        """Key off and lose keep-alive memory; permanent DTCs (non-volatile) survive."""

        def apply(s: Simulator) -> None:
            perm = s.ecu.diag.dtc.permanent
            s.driver.key = "off"
            s.ecu = ECM(region=s.ecu.region)
            s.ecu.diag.dtc.load_permanent_only(perm)

        self.runner.submit(apply)
        if self.store is not None:
            self.store.wipe_kam()

    # ------------------------------------------------------------------ faults
    def set_fault(self, setting: FaultSetting) -> None:
        copy = FaultSetting.from_dict(setting.to_dict())
        self.fault_settings[setting.key] = copy
        self.runner.submit(lambda s: s.faults.set(FaultSetting.from_dict(copy.to_dict())))

    def clear_fault(self, key: str) -> None:
        self.fault_settings.pop(key, None)
        self.runner.submit(lambda s: s.faults.clear(key))

    def clear_all_faults(self) -> None:
        self.fault_settings.clear()
        self.runner.submit(lambda s: s.faults.clear_all())

    def apply_preset(self, preset: dict) -> None:
        """Environment, engine state, codes and faults from a preset."""
        self.clear_all_faults()
        soak = preset.get("soak")
        if "ambient" in preset:
            self.set_ambient(float(preset["ambient"]))
        if soak == "cold":
            self.reset_cold(preset.get("ambient"))
        elif soak == "warm":
            self.warm_restart()
        if preset.get("clear_codes"):
            self.clear_codes()
        for f in preset.get("faults", []):
            self.set_fault(FaultSetting.from_dict(f))

    def save_preset(self, name: str) -> None:
        self.presets.save({
            "name": name,
            "ambient": self.current_sim.env.ambient_c,
            "faults": [s.to_dict() for s in self.fault_settings.values()],
        })

    # ------------------------------------------------------------------ resets / soak
    def _carry_over(self, old: Simulator, new: Simulator) -> None:
        """Keep what survives a key cycle: odometer, fuel, the ECU's keep-alive memory, active faults."""
        for s in old.faults.active.values():
            new.faults.set(FaultSetting.from_dict(s.to_dict()))
        new.ecu.region = old.ecu.region
        new.plant.vehicle.odometer_m = old.plant.vehicle.odometer_m
        new.plant.fuel_l = old.plant.fuel_l
        old.driver.key = "off"
        old.ecu.step(old.dt, old.readings, "off")  # end the drive cycle properly
        new.ecu.import_kam(old.ecu.export_kam())

    def _replace(self, build) -> None:
        def apply(old: Simulator) -> None:
            new = build(old)
            self._carry_over(old, new)
            self.runner.sim = new

        self.runner.submit(apply)

    def reset_cold(self, ambient_c: float | None = None) -> None:
        """Engine left standing overnight: everything at ambient, key off."""

        def build(old: Simulator) -> Simulator:
            amb = old.env.ambient_c if ambient_c is None else ambient_c
            return Simulator(ambient_c=amb, baro_kpa=old.env.baro_kpa)

        self._replace(build)

    def warm_restart(self) -> None:
        """Engine stopped a few minutes ago: hot coolant and catalyst, key off."""

        def build(old: Simulator) -> Simulator:
            new = Simulator(ambient_c=old.env.ambient_c, baro_kpa=old.env.baro_kpa)
            new.plant.set_warm()
            new.sensors = type(new.sensors)(new.rng)
            new.readings = new.sensors.update(new.dt, new.plant, new.ecu.act, new.driver)
            return new

        self._replace(build)

    def soak(self, hours: float) -> None:
        """Key off and let the engine cool for ``hours`` (analytic cool-down)."""

        def build(old: Simulator) -> Simulator:
            amb = old.env.ambient_c
            s = hours * 3600.0
            p = old.plant
            new = Simulator(ambient_c=amb, baro_kpa=old.env.baro_kpa)
            coolant = soaked(p.thermal.coolant_c, amb, s, COOLANT_SOAK_TAU_S)
            if coolant > amb + 1.0:
                new.plant.set_warm(coolant, soaked(p.thermal.oil_c, amb, s, COOLANT_SOAK_TAU_S),
                                   soaked(p.exhaust.cat_c, amb, s, CAT_SOAK_TAU_S))
                new.sensors = type(new.sensors)(new.rng)
                new.readings = new.sensors.update(new.dt, new.plant, new.ecu.act, new.driver)
            new.ecu.cat_est_c = amb + (old.ecu.cat_est_c - amb) * math.exp(-s / 600.0)
            return new

        self._replace(build)

    # ------------------------------------------------------------------ persistence
    def save(self) -> None:
        if self.store is None:
            return
        kam, nvm = self.call(lambda s: capture(s, self.info))
        self.store.save(kam, nvm)

    def shutdown(self) -> None:
        if getattr(self, "_shut_down", False):
            return
        self._shut_down = True
        self.disconnect_can()
        try:
            self.save()
        except TimeoutError:
            pass
        self.runner.stop()
