"""Sensor transfer functions: engineering value <-> signal voltage.

Both the sensors (forward) and the ECU's input scaling (inverse) use these, the way a real
ECU calibration contains the sensor characteristic curves.
"""

from __future__ import annotations

import math

from ..core.tables import clamp

VREF = 5.0

# --- MAF (hot film, non-linear) --------------------------------------------------------------
MAF_V0, MAF_SPAN, MAF_FULL = 1.0, 3.6, 150.0


def maf_to_v(gps: float) -> float:
    return MAF_V0 + MAF_SPAN * math.sqrt(max(gps, 0.0) / MAF_FULL)


def v_to_maf(v: float) -> float:
    x = max(0.0, (v - MAF_V0) / MAF_SPAN)
    return MAF_FULL * x * x


# --- MAP (1 bar, linear) -----------------------------------------------------------------------
MAP_K, MAP_OFF = 0.0465, -0.06


def map_to_v(kpa: float) -> float:
    return MAP_K * kpa + MAP_OFF


def v_to_map(v: float) -> float:
    return (v - MAP_OFF) / MAP_K


# --- NTC thermistors (ECT, IAT) with 2.49 kΩ pull-up -------------------------------------------
R_PULLUP, R25, BETA = 2490.0, 2500.0, 3450.0


def ntc_to_v(temp_c: float) -> float:
    r = R25 * math.exp(BETA * (1.0 / (temp_c + 273.15) - 1.0 / 298.15))
    return VREF * r / (r + R_PULLUP)


def v_to_ntc(v: float) -> float:
    v = clamp(v, 0.02, VREF - 0.02)
    r = R_PULLUP * v / (VREF - v)
    return 1.0 / (math.log(r / R25) / BETA + 1.0 / 298.15) - 273.15


# --- Throttle (dual track, complementary) and pedal (dual track, 2:1) --------------------------
def tps_a_to_v(opening: float) -> float:
    return 0.7 + 3.6 * opening


def v_to_tps_a(v: float) -> float:
    return (v - 0.7) / 3.6


def tps_b_to_v(opening: float) -> float:
    return VREF - tps_a_to_v(opening)


def v_to_tps_b(v: float) -> float:
    return v_to_tps_a(VREF - v)


def app_d_to_v(pedal: float) -> float:
    return 0.75 + 3.6 * pedal


def v_to_app_d(v: float) -> float:
    return (v - 0.75) / 3.6


def app_e_to_v(pedal: float) -> float:
    return app_d_to_v(pedal) / 2.0


def v_to_app_e(v: float) -> float:
    return v_to_app_d(2.0 * v)


# --- EVAP tank pressure (differential, -4 .. +4 kPa) --------------------------------------------
def tank_to_v(pa: float) -> float:
    return 2.5 + pa / 1600.0


def v_to_tank(v: float) -> float:
    return (v - 2.5) * 1600.0


# --- Fuel level float (pull-up, 1 V full .. 4 V empty) -----------------------------------------
def level_to_v(pct: float) -> float:
    return 4.0 - 3.0 * clamp(pct, 0.0, 100.0) / 100.0


def v_to_level(v: float) -> float:
    return (4.0 - v) / 3.0 * 100.0


# Voltage an open circuit reads (pull-up -> 5 V, pull-down -> 0 V)
OPEN_VOLTS = {
    "maf": 0.0, "map": 0.0, "ect": VREF, "iat": VREF, "tps_a": 0.0, "tps_b": 0.0, "app_d": 0.0, "app_e": 0.0,
    "tank": VREF, "fuel_level": VREF, "nb": 0.45,
}
