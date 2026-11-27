"""Data passed between the driver, the plant, the sensors and the ECU."""

from __future__ import annotations

from dataclasses import dataclass, field

KEY_OFF = "off"
KEY_ACC = "acc"
KEY_ON = "on"
KEY_START = "start"


@dataclass
class Environment:
    ambient_c: float = 15.0
    baro_kpa: float = 101.3
    fuel_octane: float = 95.0


@dataclass
class DriverInputs:
    key: str = KEY_OFF
    pedal: float = 0.0  # accelerator 0..1
    brake: float = 0.0  # 0..1
    clutch: float = 0.0  # 0 = released (engaged), 1 = fully pressed
    gear: int = 0  # requested gear: 0 = neutral, -1 = reverse, 1..5
    handbrake: bool = False
    ac: bool = False
    lights: bool = False

    @property
    def ecu_powered(self) -> bool:
        return self.key in (KEY_ON, KEY_START)

    @property
    def starter(self) -> bool:
        return self.key == KEY_START


@dataclass
class Actuators:
    """Everything the ECU drives."""

    throttle_target: float = 0.0  # 0..1 of opening beyond the rest stop
    pw_ms: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    spark_deg: float = 0.0  # deg BTDC
    fuel_pump: bool = False
    fan: int = 0  # 0 off, 1 low, 2 high
    wb_heater: bool = False
    nb_heater: bool = False
    vvt_target_deg: float = 0.0
    purge_duty: float = 0.0
    vent_closed: bool = False  # EVAP canister vent valve (normally open)
    throttle_enable: bool = True  # False = motor off, throttle springs to its limp position
    alternator_enable: bool = False


@dataclass
class Readings:
    """What the ECU sees through its sensors (engineering units after the ECU's own scaling)."""

    maf_gps: float = 0.0
    map_kpa: float = 101.3
    iat_c: float = 15.0
    ect_c: float = 15.0
    oil_c: float = 15.0
    tps_a_pct: float = 14.0  # absolute, % of 5 V
    tps_b_pct: float = 86.0
    app_d_pct: float = 15.0
    app_e_pct: float = 7.5
    rpm: float = 0.0
    crank_revs: float = 0.0
    cam_deg: float = 0.0
    wb_lambda: float = 1.0
    wb_pump_ma: float = 0.0
    wb_element_c: float = 15.0
    nb_volts: float = 0.45
    nb_element_c: float = 15.0
    vbatt: float = 12.6
    vss_kmh: float = 0.0
    fuel_level_pct: float = 60.0
    brake_switch: bool = False
    clutch_switch: bool = False
    knock: float = 0.0
    misfire_counts: list[int] = field(default_factory=lambda: [0, 0, 0, 0])  # cumulative, from CKP analysis
    # Raw signal voltages (the ECU range-checks these)
    maf_v: float = 1.0
    map_v: float = 4.65
    ect_v: float = 2.6
    iat_v: float = 2.6
    tps_a_v: float = 0.7
    tps_b_v: float = 4.3
    app_d_v: float = 0.75
    app_e_v: float = 0.375
    tank_v: float = 2.5
    fuel_level_v: float = 2.2
    tank_pa: float = 0.0
    # Signal presence / driver diagnostics
    ckp_ok: bool = True
    cmp_ok: bool = True
    cam_active: bool = False  # cam sensor sees teeth passing
    wb_signal_ok: bool = True
    wb_heater_ok: bool = True
    nb_heater_ok: bool = True
    knock_ok: bool = True
    injector_ok: list[bool] = field(default_factory=lambda: [True, True, True, True])
    coil_ok: list[bool] = field(default_factory=lambda: [True, True, True, True])
    fan_ok: bool = True
