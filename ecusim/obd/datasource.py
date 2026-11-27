"""Builds the diagnostic snapshot the OBD server answers from.

Only ECU-visible values (sensor readings, ECU state) go in here - never plant truth.
The snapshot is published once per simulation step and is never mutated afterwards, so the
CAN thread never touches live simulation objects.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..sim import Simulator

_cache: dict[int, tuple] = {}


def _diag_static(diag) -> dict:
    """DTC lists, freeze frame and Mode 06 bytes: rebuilt only when they change."""
    key = (id(diag), diag.dtc.version, diag.mode06.version)
    hit = _cache.get(id(diag))
    if hit is not None and hit[0] == key:
        return hit[1]
    ff = diag.dtc.freeze_frame
    data = {
        "dtc_confirmed": tuple(diag.dtc.confirmed),
        "dtc_pending": tuple(diag.dtc.pending),
        "dtc_permanent": tuple(diag.dtc.permanent),
        "freeze_frame": (ff.code, dict(ff.values)) if ff else None,
        "mode06": diag.mode06.frozen(),
    }
    if len(_cache) > 8:
        _cache.clear()
    _cache[id(diag)] = (key, data)
    return data


def diag_snapshot(sim: "Simulator") -> dict:
    e = sim.ecu
    d = e.obd_values(sim.readings)
    d["powered"] = sim.driver.ecu_powered
    d["mil"] = e.diag.mil
    d["iumpr"] = tuple(e.diag.iumpr.values())
    d["trips"] = e.diag.trips
    d["monitors"] = tuple(
        (m.key, m.name, m.continuous, m.supported, m.ready, m.enabled_cycle, m.ready_this_cycle, m.force, m.hint)
        for m in e.diag.monitors
    )
    d.update(_diag_static(e.diag))
    return d
