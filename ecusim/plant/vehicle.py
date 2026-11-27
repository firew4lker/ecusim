"""Longitudinal vehicle and driveline: clutch (stick-slip), 5-speed gearbox, road load, brakes (DESIGN §4.9)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core.tables import clamp
from .engine import EngineCore

G = 9.81
GEAR_RATIOS = {-1: -3.50, 0: 0.0, 1: 3.55, 2: 1.90, 3: 1.28, 4: 0.95, 5: 0.76}
GEAR_NAMES = {-1: "R", 0: "N", 1: "1", 2: "2", 3: "3", 4: "4", 5: "5"}


@dataclass
class VehicleSpec:
    mass_kg: float = 1250.0
    rotating_mass_factor: float = 1.04  # wheels, axles, gearbox
    cd: float = 0.32
    frontal_area_m2: float = 2.10
    crr: float = 0.012
    wheel_radius_m: float = 0.305
    final_drive: float = 4.06
    air_density: float = 1.2
    clutch_max_nm: float = 250.0
    clutch_bite_start: float = 0.60  # pedal travel (1 = floored) where the clutch starts to bite
    clutch_bite_full: float = 0.20  # pedal travel where it is fully engaged
    gearbox_drag_nm: float = 1.5
    brake_decel: float = 9.0  # m/s² at full pedal
    handbrake_decel: float = 3.0
    shift_clutch_min: float = 0.8  # pedal travel needed to change gear


class Vehicle:
    def __init__(self, spec: VehicleSpec | None = None):
        self.spec = spec or VehicleSpec()
        self.v = 0.0  # m/s, negative when reversing
        self.gear = 0
        self.locked = False
        self.clutch_torque = 0.0
        self.clutch_capacity = 0.0
        self.odometer_m = 0.0
        self.trip_m = 0.0
        self.grade_pct = 0.0
        self.last_shift_refused = False

    # ------------------------------------------------------------------ properties
    @property
    def kmh(self) -> float:
        return self.v * 3.6

    @property
    def ratio(self) -> float:
        return GEAR_RATIOS[self.gear] * self.spec.final_drive

    def clutch_shaft_omega(self) -> float:
        """Gearbox input speed implied by the road speed."""
        return self.v / self.spec.wheel_radius_m * self.ratio

    # ------------------------------------------------------------------ driver actions
    def shift(self, gear: int, clutch_pedal: float) -> bool:
        """Request a gear. Neutral is always possible; anything else needs the clutch pressed."""
        if gear not in GEAR_RATIOS:
            raise ValueError(f"invalid gear {gear}")
        ok = gear == 0 or clutch_pedal >= self.spec.shift_clutch_min
        if gear == -1 and self.v > 1.0:
            ok = False  # reverse only when (almost) stopped
        self.last_shift_refused = not ok
        if ok and gear != self.gear:
            self.gear = gear
            self.locked = False
        return ok

    def capacity(self, clutch_pedal: float) -> float:
        s = self.spec
        x = clamp((s.clutch_bite_start - clutch_pedal) / (s.clutch_bite_start - s.clutch_bite_full), 0.0, 1.0)
        return s.clutch_max_nm * x * x

    # ------------------------------------------------------------------ dynamics
    def _resist_force(self, v: float) -> float:
        """Aero + rolling + grade, signed so that positive resists forward motion (excludes brakes)."""
        s = self.spec
        m = s.mass_kg
        f = 0.5 * s.air_density * s.cd * s.frontal_area_m2 * v * abs(v)
        if abs(v) > 0.01:
            f += s.crr * m * G * math.copysign(1.0, v)
        f += m * G * math.sin(math.atan(self.grade_pct / 100.0))
        return f

    def _brake_force(self, brake: float, handbrake: bool) -> float:
        s = self.spec
        return s.mass_kg * (clamp(brake, 0.0, 1.0) * s.brake_decel + (s.handbrake_decel if handbrake else 0.0))

    def _advance_vehicle(self, dt: float, f_drive: float, f_brake: float) -> None:
        """Integrate road speed with Coulomb-style brakes/rolling resistance that can hold the car still."""
        s = self.spec
        m_eff = s.mass_kg * s.rotating_mass_factor
        v = self.v
        f_ext = f_drive - self._resist_force(v)
        if abs(v) < 0.02:
            static = f_brake + s.crr * s.mass_kg * G
            if abs(f_ext) <= static:
                self.v = 0.0
                return
            f = f_ext - math.copysign(f_brake, f_ext)
        else:
            f = f_ext - math.copysign(f_brake, v)
        v_new = v + f / m_eff * dt
        if v != 0.0 and v_new * v < 0.0 and abs(f_ext) <= f_brake + s.crr * s.mass_kg * G:
            v_new = 0.0  # brakes stop the car rather than reversing it
        self.v = v_new

    def step(self, dt: float, eng: EngineCore, clutch_pedal: float, brake: float, handbrake: bool) -> None:
        s = self.spec
        j_e = eng.spec.inertia_kgm2
        t_e = eng.torque_net
        omega_e = eng.omega
        f_brake = self._brake_force(brake, handbrake)
        cap = self.capacity(clutch_pedal) if self.gear != 0 else 0.0
        self.clutch_capacity = cap
        ratio = self.ratio

        if cap <= 0.0 or ratio == 0.0:
            self.locked = False
            self.clutch_torque = 0.0
            eng.set_speed(omega_e + t_e / j_e * dt, dt)
            self._advance_vehicle(dt, 0.0, f_brake)
        else:
            r = s.wheel_radius_m
            m_eff = s.mass_kg * s.rotating_mass_factor
            j_v = m_eff * r * r / (ratio * ratio)
            omega_c = self.v / r * ratio
            # Road load referred to the clutch shaft (brakes oppose motion; at rest they react as needed)
            f_res = self._resist_force(self.v)
            if abs(self.v) > 0.02:
                f_res += math.copysign(f_brake, self.v)
            t_road = f_res * r / ratio + math.copysign(s.gearbox_drag_nm, omega_c) * (abs(omega_c) > 1.0)
            t_lock = (j_v * t_e + j_e * t_road) / (j_e + j_v)

            held = abs(self.v) <= 0.02 and f_brake > 0.0
            if self.locked and (abs(t_lock) > cap or held and abs(t_e) > cap):
                self.locked = False

            if self.locked:
                t_c = t_lock
            else:
                slip = omega_e - omega_c
                t_c = math.copysign(cap, slip) if abs(slip) > 1e-6 else clamp(t_lock, -cap, cap)
            self.clutch_torque = t_c

            new_e = omega_e + (t_e - t_c) / j_e * dt
            self._advance_vehicle(dt, t_c * ratio / r, f_brake)
            new_c = self.v / r * ratio

            if self.locked:
                omega = (j_e * new_e + j_v * new_c) / (j_e + j_v)
                if omega < 0.0:
                    omega = 0.0
                    self.locked = False
                eng.set_speed(omega, dt)
                if self.locked:
                    self.v = omega / ratio * r
            else:
                slip_before = omega_e - omega_c
                slip_after = new_e - new_c
                if (slip_before * slip_after <= 0.0 or abs(slip_after) < 0.2) and abs(t_lock) <= cap:
                    omega = (j_e * new_e + j_v * new_c) / (j_e + j_v)
                    if omega >= 0.0 and not (held and abs(t_e) > cap):
                        self.locked = True
                        eng.set_speed(omega, dt)
                        self.v = omega / ratio * r
                    else:
                        eng.set_speed(new_e, dt)
                else:
                    eng.set_speed(new_e, dt)

        d = abs(self.v) * dt
        self.odometer_m += d
        self.trip_m += d
