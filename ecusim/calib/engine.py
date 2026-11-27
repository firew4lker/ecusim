"""Physical description of the simulated engine (the *plant*), see docs/DESIGN.md §2 and §4.

These numbers describe the real hardware. The ECU has its own calibration in
``ecusim.ecu.cal`` and deliberately does not read anything from here at run time
except where a calibration engineer would have mapped the engine (spark/MBT maps).
"""

from __future__ import annotations

from dataclasses import dataclass

from ..core.tables import Table1D, Table2D


@dataclass
class EngineSpec:
    # Geometry
    displacement_m3: float = 1.598e-3
    cylinders: int = 4
    manifold_volume_m3: float = 2.5e-3
    residual_p0_pa: float = 6000.0  # cylinder-filling offset (residual gas / backflow)

    # Throttle body (52 mm), effective Cd*A
    throttle_cda_full_m2: float = 1.7e-3
    throttle_cda_rest_m2: float = 4.0e-6  # flow at the mechanical rest stop
    throttle_tau_s: float = 0.03
    air_filter_k: float = 2.5e5  # pressure drop Pa per (kg/s)^2

    # Rotating assembly
    inertia_kgm2: float = 0.15
    compression_loss_nm: float = 6.0  # cranking heat/blow-by loss

    # Starter (referred to crankshaft)
    starter_stall_nm: float = 70.0
    starter_noload_rpm: float = 340.0
    starter_current_a: float = 180.0

    # Fuel system
    rail_kpa: float = 300.0  # returnless, gauge
    injector_flow_mg_ms: float = 2.75  # true flow at rail pressure (ECU assumes 2.71)
    lhv_j_per_kg: float = 43.0e6
    afr_stoich: float = 14.7

    # Combustion
    eta_ind_peak: float = 0.40

    # MAF sensor element tolerance (true reading gain)
    maf_gain: float = 0.99

    # Thermal
    coolant_heat_capacity_j_k: float = 24000.0
    oil_heat_capacity_j_k: float = 8000.0
    block_ua_running_w_k: float = 15.0
    block_ua_off_w_k: float = 4.0
    coolant_oil_ua_w_k: float = 60.0
    oil_ambient_ua_w_k: float = 6.0
    coolant_fuel_fraction: float = 0.27
    thermostat_open_c: float = 88.0
    thermostat_full_c: float = 100.0
    exh_manifold_heat_capacity_j_k: float = 1200.0
    cat_front_heat_capacity_j_k: float = 160.0
    cat_osc_nominal_g: float = 0.6

    # Electrical
    battery_capacity_ah: float = 60.0
    battery_r_ohm: float = 0.012
    alternator_setpoint_v: float = 14.3


SPEC = EngineSpec()

# Volumetric efficiency vs rpm (applied to (MAP - p0), so effective VE is lower at light load)
VE_RPM = Table1D(
    [0, 500, 1000, 2000, 3000, 4000, 4500, 5000, 6000, 7000],
    [0.80, 0.85, 0.88, 0.95, 1.02, 1.08, 1.10, 1.10, 1.10, 0.98],
)

# MBT spark advance (deg BTDC) vs rpm (x) and relative charge (y, 1.0 = 0.473 g/cyl)
_MBT_RPM = [600, 1000, 1500, 2000, 2500, 3000, 4000, 5000, 6000, 7000]
_MBT_LOAD = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
MBT = Table2D(
    _MBT_RPM,
    _MBT_LOAD,
    [
        [30, 34, 38, 41, 43, 44, 45, 46, 47, 47],
        [27, 31, 35, 38, 40, 41, 42, 43, 44, 45],
        [24, 28, 32, 35, 37, 38, 39, 40, 41, 42],
        [22, 26, 30, 32, 34, 35, 36, 37, 38, 39],
        [20, 24, 27, 29, 31, 32, 33, 34, 35, 36],
        [18, 22, 25, 27, 28, 29, 30, 31, 32, 33],
        [17, 20, 23, 25, 26, 27, 28, 29, 30, 31],
        [16, 19, 21, 23, 24, 25, 26, 27, 28, 29],
        [15, 18, 20, 22, 23, 24, 25, 26, 27, 28],
        [14, 17, 19, 21, 22, 23, 24, 25, 26, 27],
    ],
)


def mbt_deg(rpm: float, load_rel: float, coolant_c: float) -> float:
    """MBT including slower burn when cold."""
    cold = 5.0 * max(0.0, min(1.0, (70.0 - coolant_c) / 90.0))
    return MBT(rpm, load_rel) + cold


def klsa_deg(rpm: float, load_rel: float, coolant_c: float, iat_c: float, octane: float = 95.0) -> float:
    """Knock-limited spark advance."""
    base = mbt_deg(rpm, load_rel, coolant_c)
    hi_load = max(0.0, (load_rel - 0.6) / 0.4)
    lo_rpm = max(0.0, min(1.0, (4500.0 - rpm) / 3500.0))
    margin = 8.0 * hi_load * lo_rpm
    margin += 0.12 * max(0.0, iat_c - 30.0) * hi_load
    margin += 0.08 * max(0.0, coolant_c - 95.0) * hi_load
    margin += 1.5 * (95.0 - octane) * hi_load
    return base - margin


# Torque efficiency vs combustion lambda (1.0 at best-torque mixture)
ETA_LAMBDA = Table1D(
    [0.55, 0.65, 0.75, 0.82, 0.88, 0.95, 1.00, 1.10, 1.20, 1.30, 1.40, 1.50, 1.60],
    [0.60, 0.82, 0.93, 0.98, 1.00, 0.985, 0.965, 0.92, 0.86, 0.78, 0.66, 0.40, 0.0],
)

# Indicated-efficiency factor vs relative charge (slow burn / heat loss at light load)
ETA_LOAD = Table1D([0.05, 0.10, 0.25, 0.50, 0.80, 1.00], [0.57, 0.65, 0.74, 0.87, 0.96, 1.00])

# Injector dead time (ms) vs battery voltage
INJ_DEADTIME_MS = Table1D([6, 8, 10, 12, 14, 16], [2.5, 1.6, 1.1, 0.85, 0.65, 0.55])


def fmep_bar(rpm: float) -> float:
    k = rpm / 1000.0
    return 0.70 + 0.10 * k + 0.018 * k * k
