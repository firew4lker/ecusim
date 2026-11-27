"""Reusable headless driving scenarios and a simple recorder."""

from __future__ import annotations

from dataclasses import dataclass, field

from .sim import Simulator


@dataclass
class Recording:
    rows: list[dict[str, float]] = field(default_factory=list)

    def column(self, key: str) -> list[float]:
        return [r[key] for r in self.rows]

    def at(self, t: float) -> dict[str, float]:
        """Row closest to sim time ``t``."""
        return min(self.rows, key=lambda r: abs(r["t"] - t))

    def between(self, t0: float, t1: float) -> list[dict[str, float]]:
        return [r for r in self.rows if t0 <= r["t"] <= t1]

    def first(self, predicate, after: float = 0.0) -> dict[str, float] | None:
        for r in self.rows:
            if r["t"] >= after and predicate(r):
                return r
        return None


def record(sim: Simulator, seconds: float, rec: Recording, every_s: float = 0.1) -> Recording:
    stride = max(1, int(round(every_s / sim.dt)))
    n = int(round(seconds / sim.dt))
    for i in range(n):
        sim.step()
        if i % stride == 0:
            rec.rows.append(sim.snapshot())
    return rec


def cold_start_idle(ambient_c: float = 15.0, seconds: float = 960.0, seed: int = 1) -> tuple[Recording, float, float]:
    """Soaked engine at ``ambient_c``: key on, crank, then idle in neutral.

    Returns (recording, crank_time_s, start_time_s) where start_time_s is the sim time the
    ECU entered run mode.
    """
    sim = Simulator(ambient_c=ambient_c, seed=seed)
    rec = Recording()
    crank = sim.start_engine()
    t_start = sim.t
    rec.rows.append(sim.snapshot())
    record(sim, seconds, rec)
    return rec, crank, t_start


def warm_free_rev(seed: int = 2) -> tuple[Recording, Simulator]:
    """Warm engine idling, then a WOT blip to the limiter, lift off, and return to idle."""
    sim = Simulator(ambient_c=20.0, seed=seed)
    sim.plant.set_warm(coolant_c=90.0, oil_c=95.0, cat_c=450.0)
    sim.sensors = type(sim.sensors)(sim.rng)  # re-initialise sensor filters on the warm state
    sim.start_engine()
    rec = Recording()
    record(sim, 60.0, rec)  # settle at idle, closed loop
    sim.driver.pedal = 1.0
    record(sim, 3.0, rec)
    sim.driver.pedal = 0.0
    record(sim, 10.0, rec)
    return rec, sim
