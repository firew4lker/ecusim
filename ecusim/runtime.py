"""Real-time runner: steps the simulator on its own thread at wall-clock pace (optionally accelerated),
publishes diagnostic snapshots for the CAN thread, and applies commands from other threads."""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable

from .obd.datasource import diag_snapshot
from .sim import Simulator

MAX_CATCHUP_STEPS = 50


class RealtimeRunner:
    def __init__(self, sim: Simulator, time_scale: float = 1.0):
        self.sim = sim
        self.time_scale = time_scale
        self.paused = False
        self.latest_diag: dict = diag_snapshot(sim)
        self.latest: dict = sim.snapshot()
        self.overruns = 0
        self._cmds: queue.Queue[Callable[[Simulator], None]] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def replace_sim(self, sim: Simulator) -> None:
        """Swap in a fresh simulator (reset / soak) from any thread."""

        def swap(_old: Simulator) -> None:
            self.sim = sim
            self.latest_diag = diag_snapshot(sim)
            self.latest = sim.snapshot()

        self._cmds.put(swap)

    def submit(self, fn: Callable[[Simulator], None]) -> None:
        """Run ``fn(sim)`` on the simulation thread before the next step."""
        self._cmds.put(fn)

    def diag_source(self) -> dict:
        return self.latest_diag

    def powered(self) -> bool:
        return bool(self.latest_diag.get("powered"))

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ecu-sim", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        dt = self.sim.dt
        last = time.monotonic()
        owed = 0.0
        while not self._stop.is_set():
            while True:
                try:
                    fn = self._cmds.get_nowait()
                except queue.Empty:
                    break
                fn(self.sim)
            now = time.monotonic()
            if not self.paused:
                owed += (now - last) * self.time_scale / dt
            last = now
            steps = int(owed)
            if steps > MAX_CATCHUP_STEPS:
                self.overruns += 1
                steps = MAX_CATCHUP_STEPS
                owed = float(steps)
            sim = self.sim
            for _ in range(steps):
                sim.step()
            owed -= steps
            if steps:
                self.latest_diag = diag_snapshot(sim)
                self.latest = sim.snapshot()
            time.sleep(max(0.0, (1.0 - owed) * dt / max(self.time_scale, 1e-6)) if not self.paused else 0.02)
