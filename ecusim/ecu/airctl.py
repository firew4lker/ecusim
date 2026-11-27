"""Electronic throttle: driver demand, idle-speed control, dashpot, cranking position (DESIGN §5.2-5.3)."""

from __future__ import annotations

import math

from ..core.tables import clamp
from ..plant.air import R_AIR, psi
from . import cal


def throttle_for_flow(flow_gps: float, map_kpa: float, baro_kpa: float, iat_c: float) -> float:
    """Inverse throttle model: opening needed to pass ``flow_gps`` at the current pressure ratio."""
    pr = clamp(map_kpa / baro_kpa, 0.0, 0.98)
    k = baro_kpa * 1000.0 / math.sqrt(R_AIR * (iat_c + 273.15)) * psi(pr)
    if k <= 0.0:
        return 1.0
    extra = flow_gps / 1000.0 / k - cal.THROTTLE_CDA_REST
    if extra <= 0.0:
        return 0.0
    c = 1.0 - extra / cal.THROTTLE_CDA_FULL
    if c <= 0.0:
        return 1.0
    return math.acos(c) / (math.pi / 2.0)


def pedal_to_throttle(pedal_pct: float) -> float:
    p = clamp(pedal_pct / 100.0, 0.0, 1.0)
    return p ** 1.6


class AirControl:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.i_term = 0.0
        self.dashpot = 0.0
        self.target_rpm = 0.0
        self.idle_active = False
        self.err = 0.0
        self.desired_gps = 0.0
        self.throttle = 0.0
        self._pedal_was_on = False
        self.run_up = True  # after start: feed-forward only until the engine first reaches target

    def step(
        self,
        dt: float,
        mode: str,
        rpm: float,
        ect: float,
        t_run: float,
        pedal_pct: float,
        cat_heat: float,
        map_kpa: float,
        baro_kpa: float,
        iat_c: float,
        ac: bool,
        vss: float,
        clutch_sw: bool = False,
        fuel_cut: bool = False,
    ) -> float:
        if mode == "crank":
            self.idle_active = False
            self.target_rpm = 0.0
            self.throttle = max(cal.CRANK_THROTTLE, pedal_to_throttle(pedal_pct))
            return self.throttle
        if mode != "run":
            self.reset()
            return 0.0

        base = cal.IDLE_BASE_RPM(ect)
        target = base + cal.FLARE_RPM * math.exp(-t_run / cal.FLARE_TAU_S) + (cal.AC_IDLE_UP_RPM if ac else 0.0)
        self.target_rpm = target

        pedal_on = pedal_pct > 1.0
        if self._pedal_was_on and not pedal_on:
            self.dashpot = cal.DASHPOT_GPS
        self._pedal_was_on = pedal_on
        self.dashpot *= math.exp(-dt / cal.DASHPOT_TAU_S)

        # During decel fuel cut hold a decel-air schedule instead of shutting the throttle (manifold vacuum,
        # smooth re-entry). Only while fuel is cut: with fuel on, extra air above idle would add torque and
        # run the engine away.
        overspeed = min(3.0, max(1.0, (rpm - 150.0) / target) ** cal.DECEL_AIR_EXP) if (fuel_cut or self.dashpot > 0.5) else 1.0
        ff = cal.IDLE_FF_GPS(ect) * (target / base) * overspeed + cat_heat * cal.CAT_HEAT_AIR_GPS
        ff += (cal.AC_AIR_GPS if ac else 0.0) + self.dashpot

        # Idle governor whenever the pedal is up and the engine is (nearly) decoupled or about to stall
        self.idle_active = not pedal_on and (vss < 3.0 or clutch_sw or rpm < target + 200.0)
        err = target - rpm
        if self.run_up and (rpm >= 0.9 * target or t_run > 3.0 or pedal_on):
            self.run_up = False
        self.err = err if self.idle_active and not self.run_up else 0.0
        if self.run_up:
            desired = ff
        elif self.idle_active:
            if err > -300.0:
                self.i_term = clamp(self.i_term + cal.IDLE_KI * err * dt, -cal.IDLE_I_LIMIT, cal.IDLE_I_LIMIT)
            p_term = clamp(cal.IDLE_KP * err, -cal.IDLE_P_NEG_LIMIT, cal.IDLE_P_POS_LIMIT)
            desired = ff + p_term + self.i_term
        else:
            desired = ff + self.i_term
        desired = max(desired, 0.5)
        self.desired_gps = desired

        x_idle = throttle_for_flow(desired, map_kpa, baro_kpa, iat_c)
        self.throttle = max(x_idle, pedal_to_throttle(pedal_pct))
        return self.throttle
