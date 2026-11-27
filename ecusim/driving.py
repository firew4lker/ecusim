"""Scripted driver for headless tests and demos: smooth pedal ramps, gear changes, speed hold."""

from __future__ import annotations

from .scenarios import Recording
from .sim import Simulator


class ScriptedDriver:
    def __init__(self, sim: Simulator, rec: Recording | None = None, every_s: float = 0.1):
        self.sim = sim
        self.rec = rec if rec is not None else Recording()
        self._stride = max(1, int(round(every_s / sim.dt)))
        self._n = 0

    @property
    def d(self):
        return self.sim.driver

    def _step(self) -> None:
        self.sim.step()
        self._n += 1
        if self._n % self._stride == 0:
            self.rec.rows.append(self.sim.snapshot())

    def wait(self, seconds: float) -> None:
        for _ in range(int(round(seconds / self.sim.dt))):
            self._step()

    def ramp(self, field: str, target: float, seconds: float) -> None:
        """Move a pedal (``pedal``, ``clutch``, ``brake``) linearly to ``target``."""
        start = getattr(self.d, field)
        n = max(1, int(round(seconds / self.sim.dt)))
        for i in range(1, n + 1):
            setattr(self.d, field, start + (target - start) * i / n)
            self._step()

    def shift(self, gear: int, clutch_release_s: float = 0.6, pedal_after: float | None = None) -> None:
        """Lift, clutch in, select gear, clutch out (with optional throttle)."""
        self.ramp("pedal", 0.0, 0.15)
        self.ramp("clutch", 1.0, 0.2)
        self.d.gear = gear
        self.wait(0.25)
        if pedal_after is not None:
            self.ramp("pedal", pedal_after, 0.15)
        self.ramp("clutch", 0.0, clutch_release_s)

    def launch(self, gear: int = 1, pedal: float = 0.2, slip_s: float = 2.0) -> None:
        """Pull away from rest: clutch in, select gear, find the bite point, release gently."""
        self.ramp("clutch", 1.0, 0.2)
        self.d.gear = gear
        self.wait(0.3)
        self.ramp("clutch", 0.62, 0.3)
        if pedal > 0.0:
            self.ramp("pedal", pedal, 0.3)
        self.ramp("clutch", 0.25, slip_s)
        self.ramp("clutch", 0.0, 0.3)

    def accelerate_until(self, pedal: float, rpm: float | None = None, kmh: float | None = None,
                         timeout: float = 30.0) -> None:
        self.ramp("pedal", pedal, 0.2)
        t0 = self.sim.t
        while self.sim.t - t0 < timeout:
            if rpm is not None and self.sim.plant.engine.rpm >= rpm:
                return
            if kmh is not None and self.sim.plant.vehicle.kmh >= kmh:
                return
            self._step()

    def hold_speed(self, kmh: float, seconds: float, kp: float = 0.08, ki: float = 0.02) -> None:
        """Simple cruise control on the accelerator."""
        integ = self.d.pedal
        for _ in range(int(round(seconds / self.sim.dt))):
            err = kmh - self.sim.plant.vehicle.kmh
            integ = min(1.0, max(0.0, integ + ki * err * self.sim.dt))
            self.d.pedal = min(1.0, max(0.0, integ + kp * err))
            self._step()

    def brake_to_stop(self, brake: float = 0.4, timeout: float = 30.0) -> None:
        self.ramp("pedal", 0.0, 0.15)
        t0 = self.sim.t
        while self.sim.plant.vehicle.kmh > 15.0 and self.sim.t - t0 < timeout:
            self.d.brake = brake
            self._step()
        self.ramp("clutch", 1.0, 0.2)
        while abs(self.sim.plant.vehicle.v) > 0.0 and self.sim.t - t0 < timeout:
            self.d.brake = brake
            self._step()
        self.d.gear = 0
        self.d.brake = 0.0
        self.wait(0.3)


def standard_drive_cycle(sim: Simulator, cruise_s: float = 150.0, rec_every_s: float = 1.0) -> ScriptedDriver:
    """Idle, a free-rev with fuel cut, pull away through the gears to a steady 60 km/h cruise,
    coast down in gear, stop and idle. Covers every monitor's enable conditions (EVAP and the
    O2 heaters additionally need a cold start and ``cruise_s`` >= 400). Stops early if the engine dies."""
    d = ScriptedDriver(sim, every_s=rec_every_s)

    def alive() -> bool:
        return sim.ecu.mode == "run"

    d.wait(70.0)
    if alive():
        d.ramp("pedal", 0.6, 0.3)
        d.wait(1.5)
        d.ramp("pedal", 0.0, 0.1)
        d.wait(4.0)
    if alive():
        d.launch(1, 0.3, 1.5)
        d.accelerate_until(0.5, rpm=3000, timeout=15)
        for g in (2, 3, 4):
            if not alive():
                break
            d.shift(g, pedal_after=0.4)
            d.accelerate_until(0.5, rpm=3000 if g < 4 else None, kmh=60 if g == 4 else None, timeout=20)
    if alive():
        d.hold_speed(60.0, cruise_s)
    if alive():
        d.ramp("pedal", 0.0, 0.2)
        d.wait(6.0)
        d.brake_to_stop()
        d.wait(60.0)
    return d
