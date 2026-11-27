"""Intake air path: air filter, electronic throttle, intake manifold, cylinder filling (DESIGN §4.1)."""

from __future__ import annotations

import math

from ..calib.engine import EngineSpec
from ..core.tables import clamp, lowpass_alpha

R_AIR = 287.05
_PR_CRIT = 0.528282
_PSI_CRIT = 0.684731
_PR_LIN = 0.995


def _psi_raw(pr: float) -> float:
    return math.sqrt(max(0.0, 7.0 * (pr ** 1.4285714 - pr ** 1.7142857)))


_PSI_LIN = _psi_raw(_PR_LIN)


def psi(pr: float) -> float:
    """Compressible orifice flow function for gamma = 1.4 (linearised close to pr = 1)."""
    if pr <= _PR_CRIT:
        return _PSI_CRIT
    if pr >= 1.0:
        return 0.0
    if pr > _PR_LIN:
        return _PSI_LIN * (1.0 - pr) / (1.0 - _PR_LIN)
    return _psi_raw(pr)


def throttle_cda(spec: EngineSpec, opening: float) -> float:
    x = clamp(opening, 0.0, 1.0)
    return spec.throttle_cda_rest_m2 + spec.throttle_cda_full_m2 * (1.0 - math.cos(x * math.pi / 2.0))


def _illinois(f, lo: float, hi: float, flo: float, fhi: float, tol: float = 0.5, max_iter: int = 50) -> float:
    side = 0
    x = lo
    for _ in range(max_iter):
        x = (lo * fhi - hi * flo) / (fhi - flo)
        fx = f(x)
        if abs(fx) < tol:
            return x
        if fx * fhi > 0.0:
            hi, fhi = x, fx
            if side == -1:
                flo *= 0.5
            side = -1
        else:
            lo, flo = x, fx
            if side == 1:
                fhi *= 0.5
            side = 1
        if hi - lo < tol:
            break
    return x


class AirPath:
    def __init__(self, spec: EngineSpec, baro_pa: float):
        self.spec = spec
        self.throttle = 0.0  # actual opening 0..1
        self.map_pa = baro_pa
        self.p_up_pa = baro_pa
        self.throttle_flow = 0.0  # kg/s, this is what the MAF sees
        self.leak_flow = 0.0  # kg/s unmetered
        self.cyl_flow = 0.0  # kg/s into the cylinders
        # Fault hooks
        self.vacuum_leak_cda_m2 = 0.0
        self.throttle_deposit = 0.0  # 0..1 fraction of rest-stop area lost
        self.filter_restriction = 1.0  # multiplier on the air filter pressure drop
        self.throttle_stuck: float | None = None  # motor/gear fault: blade stuck at this opening

    def step(
        self,
        dt: float,
        throttle_target: float,
        rpm: float,
        t_up_k: float,
        t_charge_k: float,
        baro_pa: float,
        ve: float,
        extra_inflow_kgps: float = 0.0,
    ) -> None:
        s = self.spec
        if self.throttle_stuck is not None:
            self.throttle = self.throttle_stuck
        else:
            self.throttle += (clamp(throttle_target, 0.0, 1.0) - self.throttle) * lowpass_alpha(dt, s.throttle_tau_s)

        p_up = max(baro_pa - s.air_filter_k * self.filter_restriction * self.throttle_flow ** 2, 0.5 * baro_pa)
        self.p_up_pa = p_up
        cda = throttle_cda(s, self.throttle) - s.throttle_cda_rest_m2 * self.throttle_deposit
        sqrt_rt = math.sqrt(R_AIR * t_up_k)
        c_thr = max(cda, 0.0) * p_up / sqrt_rt
        c_leak = self.vacuum_leak_cda_m2 * baro_pa / sqrt_rt
        k_cyl = ve * s.displacement_m3 * (rpm / 120.0) / (R_AIR * t_charge_k)
        p0 = s.residual_p0_pa
        k_man = R_AIR * t_charge_k / s.manifold_volume_m3
        p_old = self.map_pa

        def f(p: float) -> float:
            inflow = c_thr * psi(p / p_up) + c_leak * psi(p / baro_pa) + extra_inflow_kgps
            out = k_cyl * (p - p0) if p > p0 else 0.0
            return p - p_old - dt * k_man * (inflow - out)

        lo = 100.0
        hi = max(p_up, p_old) + 500.0
        flo = f(lo)
        fhi = f(hi)
        while fhi < 0.0:  # only if leak flow dominates; widen the bracket
            hi *= 1.2
            fhi = f(hi)
        p = _illinois(f, lo, hi, flo, fhi)

        self.map_pa = p
        self.throttle_flow = c_thr * psi(p / p_up)
        self.leak_flow = c_leak * psi(p / baro_pa)
        self.cyl_flow = k_cyl * (p - p0) if p > p0 else 0.0
