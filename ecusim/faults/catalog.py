"""Fault catalogue (DESIGN §8.1). Every fault acts on the plant or the sensor layer only."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable

from ..sensors.faults import (
    GAIN,
    NOISY,
    OFFSET,
    OPEN,
    SHORT_GND,
    SHORT_V,
    SLOW,
    STUCK,
)

if TYPE_CHECKING:
    from ..sim import Simulator

TOGGLE, SEVERITY, SENSOR = "toggle", "severity", "sensor"


@dataclass(frozen=True)
class FaultDef:
    key: str
    group: str
    name: str
    kind: str
    expected: str = ""
    # severity faults
    unit: str = ""
    lo: float = 0.0
    hi: float = 1.0
    default: float = 1.0
    nominal: float = 0.0
    per_cylinder: bool = False
    apply: Callable[["Simulator", float, int], None] | None = None  # (sim, value, cylinder index)
    # sensor faults
    channel: str = ""
    modes: tuple[str, ...] = ()
    amounts: dict = field(default_factory=dict)  # mode -> (unit, lo, hi, default, scale)


def _hole_cda(d_mm: float) -> float:
    return 0.6 * math.pi * (d_mm / 2000.0) ** 2


def _set_list(attr_path: str):
    def apply(sim: "Simulator", v: float, cyl: int) -> None:
        obj = sim.plant
        *parts, last = attr_path.split(".")
        for p in parts:
            obj = getattr(obj, p)
        getattr(obj, last)[cyl] = v

    return apply


def _set(attr_path: str, transform=lambda v: v):
    def apply(sim: "Simulator", v: float, cyl: int) -> None:
        root, _, rest = attr_path.partition(":")
        obj = {"plant": sim.plant, "sensors": sim.sensors, "env": sim.env}[root]
        *parts, last = rest.split(".")
        for p in parts:
            obj = getattr(obj, p)
        setattr(obj, last, transform(v))

    return apply


_TEMP = {OFFSET: ("°C", -60, 60, -25, 1.0), STUCK: ("", 0, 0, 0, 1.0), SLOW: ("s", 1, 120, 30, 1.0),
         NOISY: ("°C", 0, 20, 5, 1.0)}
_PCT = {OFFSET: ("%", -30, 30, 10, 0.01), GAIN: ("%", -50, 50, -20, 0.01), STUCK: ("", 0, 0, 0, 1.0),
        SLOW: ("s", 0.05, 2, 0.5, 1.0), NOISY: ("%", 0, 10, 2, 0.01)}
_CIRCUIT = (OPEN, SHORT_GND, SHORT_V)

FAULTS: list[FaultDef] = [
    # ---------------------------------------------------------------- air
    FaultDef("vacuum_leak", "Air", "Vacuum leak", SEVERITY, "lean, LTFT up at idle, high idle → P0171",
             "mm hole", 0.5, 8.0, 2.6, 0.0,
             apply=lambda s, v, c: setattr(s.plant.air, "vacuum_leak_cda_m2", _hole_cda(v))),
    FaultDef("pcv_hose", "Air", "PCV hose disconnected", TOGGLE,
             "unmetered air beyond trim authority: lean misfire, high idle → P0171, P030x",
             apply=lambda s, v, c: setattr(s.plant.air, "vacuum_leak_cda_m2", _hole_cda(4.5 * v))),
    FaultDef("dirty_maf", "Air", "Dirty MAF sensor", SEVERITY, "under-reads at load → P0171, severe → P0101",
             "% low", 5, 60, 40, 0, apply=_set("sensors:maf_contamination", lambda v: v / 100.0)),
    FaultDef("air_filter", "Air", "Restricted air filter", SEVERITY, "less power at WOT, usually no DTC",
             "× drop", 1, 20, 8, 1, apply=_set("plant:air.filter_restriction")),
    FaultDef("throttle_deposit", "Air", "Dirty throttle body", SEVERITY, "idle air learn compensates, no DTC",
             "%", 10, 100, 70, 0, apply=_set("plant:air.throttle_deposit", lambda v: v / 100.0)),
    FaultDef("throttle_stuck", "Air", "Throttle blade stuck", SEVERITY, "→ P2101, limp home, EPC",
             "% open", 0, 30, 4, -1,
             apply=lambda s, v, c: setattr(s.plant.air, "throttle_stuck", None if v < 0 else v / 100.0)),
    # ---------------------------------------------------------------- fuel
    FaultDef("fuel_pressure", "Fuel", "Low fuel pressure (weak pump)", SEVERITY, "lean at load → P0171",
             "% of nominal", 30, 95, 50, 100, apply=_set("plant:fuel_pressure_factor", lambda v: v / 100.0)),
    FaultDef("fuel_pump", "Fuel", "Fuel pump failed", TOGGLE, "engine dies / will not start",
             apply=_set("plant:fuel_pressure_factor", lambda v: 0.0 if v else 1.0)),
    FaultDef("injector_clogged", "Fuel", "Clogged injector", SEVERITY, "lean cylinder, misfire → P030x",
             "% flow", 20, 95, 60, 100, per_cylinder=True,
             apply=lambda s, v, c: s.plant.fuel.injector_health.__setitem__(c, v / 100.0)),
    FaultDef("injector_leak", "Fuel", "Leaking injector", SEVERITY, "rich, hard hot start → P0172",
             "mg/s", 5, 120, 70, 0, per_cylinder=True,
             apply=lambda s, v, c: s.plant.fuel.injector_leak_kgps.__setitem__(c, v * 1e-6)),
    FaultDef("injector_open", "Fuel", "Injector open circuit", TOGGLE, "→ P020x + P030x, MIL flashing",
             per_cylinder=True,
             apply=lambda s, v, c: s.plant.fuel.injector_health.__setitem__(c, 0.0 if v else 1.0)),
    FaultDef("purge_open", "Fuel", "Purge valve stuck open", TOGGLE, "rich idle with a loaded canister → P0496",
             apply=lambda s, v, c: setattr(s.plant.evap, "purge_stuck", "open" if v else None)),
    FaultDef("purge_closed", "Fuel", "Purge valve stuck closed", TOGGLE, "→ P0441",
             apply=lambda s, v, c: setattr(s.plant.evap, "purge_stuck", "closed" if v else None)),
    FaultDef("bad_fuel", "Fuel", "Low-octane fuel", SEVERITY, "knock retard, less power, no DTC",
             "RON", 80, 94, 87, 95, apply=_set("env:fuel_octane")),
    # ---------------------------------------------------------------- ignition
    FaultDef("coil", "Ignition", "Ignition coil failed", TOGGLE, "→ P035x + P030x, injector cut",
             per_cylinder=True, apply=lambda s, v, c: s.plant.engine.coil_failed.__setitem__(c, bool(v))),
    FaultDef("plug", "Ignition", "Worn spark plug", SEVERITY, "misfire under load → P030x",
             "wear %", 10, 100, 70, 0, per_cylinder=True,
             apply=lambda s, v, c: s.plant.engine.plug_wear.__setitem__(c, v / 100.0)),
    FaultDef("random_misfire", "Ignition", "Random misfire", SEVERITY, "→ P0300",
             "% events", 1, 20, 6, 0,
             apply=lambda s, v, c: s.plant.engine.__setattr__("misfire_fault", [v / 100.0] * 4)),
    # ---------------------------------------------------------------- mechanical
    FaultDef("compression", "Mechanical", "Low compression", SEVERITY, "misfire at idle → P030x",
             "% of normal", 40, 95, 60, 100, per_cylinder=True,
             apply=lambda s, v, c: s.plant.engine.compression.__setitem__(c, v / 100.0)),
    FaultDef("chain", "Mechanical", "Timing chain stretched", SEVERITY, "cam/crank offset → P0016",
             "° crank", 2, 15, 9, 0, apply=_set("plant:cam_offset_deg")),
    FaultDef("vvt_stuck", "Mechanical", "VVT solenoid stuck", SEVERITY, "→ P0011/P0012, power loss",
             "° advance", 0, 35, 25, -1,
             apply=lambda s, v, c: setattr(s.plant, "vvt_stuck_deg", None if v < 0 else v)),
    # ---------------------------------------------------------------- cooling
    FaultDef("thermostat_open", "Cooling", "Thermostat stuck open", TOGGLE, "slow warm-up → P0128",
             apply=lambda s, v, c: setattr(s.plant.thermal, "thermostat_stuck", 1.0 if v else None)),
    FaultDef("thermostat_closed", "Cooling", "Thermostat stuck closed", TOGGLE, "overheats, fan on high",
             apply=lambda s, v, c: setattr(s.plant.thermal, "thermostat_stuck", 0.0 if v else None)),
    FaultDef("fan", "Cooling", "Cooling fan failed", TOGGLE, "overheats at idle; P0480 once the fan is commanded",
             apply=lambda s, v, c: setattr(s.plant.thermal, "fan_failed", bool(v))),
    # ---------------------------------------------------------------- exhaust
    FaultDef("catalyst", "Exhaust", "Catalyst degraded", SEVERITY, "downstream O2 follows upstream → P0420",
             "% storage left", 2, 60, 6, 100, apply=_set("plant:exhaust.cat_health", lambda v: v / 100.0)),
    FaultDef("exhaust_leak", "Exhaust", "Exhaust leak before B1S1", SEVERITY, "false lean, runs rich → P2195",
             "% dilution", 2, 30, 10, 0, apply=_set("plant:exhaust.pre_sensor_leak", lambda v: v / 100.0)),
    # ---------------------------------------------------------------- EVAP
    FaultDef("evap_leak", "EVAP", "EVAP leak", SEVERITY, "1.0 mm → P0442, 0.5 mm → P0456 (US)",
             "mm", 0.3, 2.0, 1.0, 0, apply=_set("plant:evap.leak_diameter_mm")),
    FaultDef("gas_cap", "EVAP", "Fuel cap loose", TOGGLE, "→ P0455",
             apply=_set("plant:evap.leak_diameter_mm", lambda v: 4.0 if v else 0.0)),
    FaultDef("vent", "EVAP", "Vent valve stuck open", TOGGLE, "cannot seal → P0446",
             apply=_set("plant:evap.vent_stuck_open", bool)),
    # ---------------------------------------------------------------- electrical
    FaultDef("battery", "Electrical", "Weak battery", SEVERITY, "slow crank, voltage dip; severe: no start",
             "%", 10, 100, 50, 0, apply=_set("plant:elec.weak_battery", lambda v: v / 100.0)),
    FaultDef("alternator", "Electrical", "Alternator failed", TOGGLE,
             "battery lamp at once; P0562 only once the battery drains below 11.8 V (about an hour)",
             apply=_set("plant:elec.alternator_failed", bool)),
    FaultDef("regulator", "Electrical", "Voltage regulator overcharging", TOGGLE, "→ P0563",
             apply=_set("plant:elec.regulator_v", lambda v: 15.9 if v else None)),
    # ---------------------------------------------------------------- sensors
    FaultDef("s_maf", "Sensors", "MAF sensor", SENSOR, "circuit → P0102/P0103, signal → P0101",
             channel="maf", modes=_CIRCUIT + (OFFSET, GAIN, STUCK, NOISY),
             amounts={OFFSET: ("g/s", -10, 10, -0.5, 1.0), GAIN: ("%", -50, 50, -30, 0.01),
                      NOISY: ("g/s", 0, 5, 0.3, 1.0), STUCK: ("", 0, 0, 0, 1.0)}),
    FaultDef("s_map", "Sensors", "MAP sensor", SENSOR, "circuit → P0107/P0108, implausible → P0106",
             channel="map", modes=_CIRCUIT + (OFFSET, STUCK, NOISY),
             amounts={OFFSET: ("kPa", -30, 30, 15, 1.0), NOISY: ("kPa", 0, 10, 1, 1.0), STUCK: ("", 0, 0, 0, 1.0)}),
    FaultDef("s_ect", "Sensors", "Coolant temperature sensor", SENSOR,
             "open → P0118 (reads -40 °C), short → P0117, stuck → P0116", channel="ect",
             modes=_CIRCUIT + (OFFSET, STUCK, SLOW, NOISY), amounts=_TEMP),
    FaultDef("s_iat", "Sensors", "Intake air temperature sensor", SENSOR, "circuit → P0112/P0113", channel="iat",
             modes=_CIRCUIT + (OFFSET, STUCK, SLOW, NOISY), amounts=_TEMP),
    FaultDef("s_tps_a", "Sensors", "Throttle position sensor A", SENSOR, "→ P0122/P0123/P2135, limp home",
             channel="tps_a", modes=_CIRCUIT + (OFFSET, GAIN, STUCK, NOISY), amounts=_PCT),
    FaultDef("s_tps_b", "Sensors", "Throttle position sensor B", SENSOR, "→ P0222/P0223/P2135, limp home",
             channel="tps_b", modes=_CIRCUIT + (OFFSET, GAIN, STUCK, NOISY), amounts=_PCT),
    FaultDef("s_app_d", "Sensors", "Pedal position sensor D", SENSOR, "→ P2122/P2123/P2138, reduced power",
             channel="app_d", modes=_CIRCUIT + (OFFSET, GAIN, STUCK), amounts=_PCT),
    FaultDef("s_app_e", "Sensors", "Pedal position sensor E", SENSOR, "→ P2127/P2128/P2138, reduced power",
             channel="app_e", modes=_CIRCUIT + (OFFSET, GAIN, STUCK), amounts=_PCT),
    FaultDef("s_wb", "Sensors", "Upstream O2 (wideband B1S1)", SENSOR,
             "open → P0130 + open loop, slow → P0133, stuck → P0134, offset → P2195/P2196", channel="wb",
             modes=(OPEN, OFFSET, STUCK, SLOW, NOISY),
             amounts={OFFSET: ("λ", -0.1, 0.1, 0.06, 1.0), SLOW: ("s", 0.2, 3.0, 1.0, 1.0),
                      NOISY: ("λ", 0, 0.1, 0.03, 1.0), STUCK: ("", 0, 0, 0, 1.0)}),
    FaultDef("s_nb", "Sensors", "Downstream O2 (narrowband B1S2)", SENSOR,
             "open → P0140, short → P0137/P0138, slow → P0139", channel="nb",
             modes=(OPEN, SHORT_GND, SHORT_V, STUCK, SLOW),
             amounts={SLOW: ("s", 0.5, 5.0, 2.0, 1.0), STUCK: ("", 0, 0, 0, 1.0)}),
    FaultDef("wb_heater", "Sensors", "B1S1 heater open", TOGGLE, "→ P0030 + P0135, open loop",
             apply=_set("sensors:wb_heater_failed", bool)),
    FaultDef("nb_heater", "Sensors", "B1S2 heater open", TOGGLE, "→ P0036 + P0141",
             apply=_set("sensors:nb_heater_failed", bool)),
    FaultDef("s_knock", "Sensors", "Knock sensor", SENSOR, "→ P0327, safety spark retard",
             channel="knock", modes=(OPEN, SHORT_GND)),
    FaultDef("s_ckp", "Sensors", "Crankshaft position sensor", SENSOR, "engine stops / no start → P0335",
             channel="ckp", modes=(OPEN,)),
    FaultDef("s_cmp", "Sensors", "Camshaft position sensor", SENSOR, "long crank, runs → P0340",
             channel="cmp", modes=(OPEN,)),
    FaultDef("s_tank", "Sensors", "EVAP tank pressure sensor", SENSOR, "→ P0452/P0453, EVAP test disabled",
             channel="tank", modes=_CIRCUIT + (OFFSET, STUCK),
             amounts={OFFSET: ("Pa", -1500, 1500, 800, 1.0), STUCK: ("", 0, 0, 0, 1.0)}),
    FaultDef("s_level", "Sensors", "Fuel level sensor", SENSOR, "stuck → P0461", channel="fuel_level",
             modes=_CIRCUIT + (STUCK, OFFSET), amounts={OFFSET: ("%", -40, 40, 20, 1.0), STUCK: ("", 0, 0, 0, 1.0)}),
    FaultDef("s_vss", "Sensors", "Vehicle speed signal", SENSOR, "→ P0500", channel="vss", modes=(OPEN,)),
]

BY_KEY = {f.key: f for f in FAULTS}
GROUPS = list(dict.fromkeys(f.group for f in FAULTS))
