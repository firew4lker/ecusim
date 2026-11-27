"""EVAP canister purge control with vapour-concentration learning (DESIGN §5.4 purge learning)."""

from __future__ import annotations

import math

from ..core.tables import clamp

PURGE_MAX_GPS = 0.8  # ECU's model of the purge valve (same hardware as the plant)
PURGE_TARGET = 0.4
RAMP_PER_S = 0.05


class PurgeControl:
    def __init__(self) -> None:
        self.conc = 0.05  # learned g fuel per g purge air (keep-alive memory)
        self.reset()

    def reset(self) -> None:
        self.duty = 0.0
        self.flow_est_gps = 0.0
        self.fuel_est_gps = 0.0
        self.active = False
        self.active_time = 0.0

    def step(self, dt: float, allowed: bool, map_kpa: float, baro_kpa: float, stft: float, fuel_gps: float,
             override: float | None = None) -> float:
        if override is not None:
            self.duty = override
        elif allowed:
            self.duty = min(PURGE_TARGET, self.duty + RAMP_PER_S * dt)
        else:
            self.duty = 0.0
        vac = max(0.0, baro_kpa - map_kpa)
        self.flow_est_gps = self.duty * PURGE_MAX_GPS * math.sqrt(clamp(vac / 30.0, 0.0, 1.0))
        self.active = self.flow_est_gps > 0.02
        self.active_time = self.active_time + dt if self.active else 0.0
        if self.active and allowed and override is None:
            # Over-estimated vapour -> engine runs lean -> STFT positive -> lower the estimate
            self.conc = clamp(self.conc - 0.5 * stft * fuel_gps / self.flow_est_gps * dt, 0.0, 0.35)
        self.fuel_est_gps = self.flow_est_gps * self.conc
        return self.duty

    def owns_error(self, stft: float) -> bool:
        """Is the trim error still being explained by purge vapour (so long-term trims must not learn it)?"""
        if not self.active:
            return False
        if self.active_time < 20.0:
            return True
        return (stft < -0.03 and self.conc < 0.34) or (stft > 0.03 and self.conc > 0.01)
