"""Fuel control: lambda target, closed loop (STFT), adaptives (LTFT), DFCO, power enrichment,
transient compensation and injector pulse width (DESIGN §5.4)."""

from __future__ import annotations

import math

from ..core.tables import clamp
from ..plant.fuel import film_params
from . import cal

# J1979 PID 03 fuel system status
FS_NONE = 0
FS_OL_TEMP = 1
FS_CL = 2
FS_OL_DRIVE = 4
FS_OL_FAULT = 8
FS_CL_FAULT = 16

CELL_IDLE, CELL_LOW, CELL_MID, CELL_HIGH = range(4)
CELL_NAMES = ("idle", "low", "mid", "high")


class FuelControl:
    def __init__(self) -> None:
        self.ltft = [0.0, 0.0, 0.0, 0.0]  # keep-alive memory
        self.reset()

    def reset(self) -> None:
        self.status = FS_NONE
        self.lambda_cmd = 1.0
        self.lambda_target_cl = 1.0
        self.closed_loop = False
        self.stft = 0.0
        self.stft_i = 0.0
        self.cell = CELL_IDLE
        self.dtrim = 0.0
        self.film_est = 0.0
        self.dfco = False
        self.dfco_timer = 0.0
        self.pe = False
        self.pe_timer = 0.0
        self.rev_cut = False
        self.cl_hold = 0.0
        self.fuel_mg = 0.0
        self.pw_ms = 0.0
        self.desired_gps = 0.0
        self.injected_gps = 0.0
        self.mod = 0.0
        self.wb_filt = 1.0
        self.ase = 0.0
        self.cat_purge = 0.0  # post fuel-cut enrichment to re-centre catalyst oxygen storage

    @property
    def ltft_active(self) -> float:
        return self.ltft[self.cell]

    def step(
        self,
        dt: float,
        t: float,
        mode: str,
        synced: bool,
        rpm: float,
        cyl_flow_gps: float,
        load_rel: float,
        ect: float,
        ect_start: float,
        t_run: float,
        pedal_pct: float,
        idle_active: bool,
        vss: float,
        map_kpa: float,
        baro_kpa: float,
        vbatt: float,
        wb_lambda: float,
        wb_ready: bool,
        nb_volts: float,
        nb_ready: bool,
        cat_est_c: float,
        cat_heat: float,
        wb_failed: bool = False,
        purge_fuel_gps: float = 0.0,
        ltft_hold: bool = False,
        rev_limit: float | None = None,
    ) -> None:
        if mode not in ("crank", "run") or not synced:
            self.status = FS_NONE if mode != "crank" else FS_OL_TEMP
            self.pw_ms = 0.0
            self.fuel_mg = 0.0
            self.stft = 0.0
            self.stft_i = 0.0
            self.closed_loop = False
            self.film_est *= math.exp(-dt / 30.0)
            return

        self.wb_filt += (wb_lambda - self.wb_filt) * min(1.0, dt / 0.05)

        # ---- cell selection for adaptives
        if idle_active and rpm < 1100.0:
            self.cell = CELL_IDLE
        elif load_rel < 0.35:
            self.cell = CELL_LOW
        elif load_rel < 0.60:
            self.cell = CELL_MID
        else:
            self.cell = CELL_HIGH

        cut = False
        if mode == "crank":
            self.status = FS_OL_TEMP
            lam = cal.LAMBDA_CRANK(ect)
            self.closed_loop = False
            self.dfco = self.pe = self.rev_cut = False
        else:
            # Rev limiter (lower in limp home)
            cut_at = rev_limit if rev_limit is not None else cal.REV_LIMIT_CUT
            if rpm > cut_at:
                self.rev_cut = True
            elif rpm < cut_at - 100.0:
                self.rev_cut = False

            # Decel fuel cut
            dfco_ok = (
                ect > cal.DFCO_MIN_ECT_C
                and pedal_pct < 1.0
                and t_run > 10.0
                and (rpm > 1800.0 or (vss > 5.0 and rpm > 1300.0))
            )
            if self.dfco:
                exit_rpm = 1100.0 if vss > 5.0 else 1350.0
                if pedal_pct >= 1.0 or rpm < exit_rpm:
                    self.dfco = False
                    self.dfco_timer = 0.0
                    self.cl_hold = 1.0
                    self.cat_purge = cal.CAT_PURGE_MAX_S
            else:
                self.dfco_timer = self.dfco_timer + dt if dfco_ok else 0.0
                if self.dfco_timer > cal.DFCO_DELAY_S:
                    self.dfco = True

            # Power enrichment
            if self.pe:
                if load_rel < cal.PE_EXIT_LOAD or pedal_pct < cal.PE_ENTER_PEDAL - 10.0:
                    self.pe = False
                    self.pe_timer = 0.0
                    self.cl_hold = 1.0
            else:
                if load_rel > cal.PE_ENTER_LOAD and pedal_pct > cal.PE_ENTER_PEDAL:
                    self.pe_timer += dt
                    if self.pe_timer > 0.5:
                        self.pe = True
                else:
                    self.pe_timer = 0.0

            self.ase = cal.ASE_INITIAL(ect_start) * math.exp(-t_run / cal.ASE_TAU_S(ect_start))
            lam_ol = cal.LAMBDA_WARMUP(ect) / (1.0 + self.ase)
            self.cl_hold = max(0.0, self.cl_hold - dt)
            if self.cat_purge > 0.0:
                self.cat_purge = 0.0 if (nb_ready and nb_volts > 0.6) else max(0.0, self.cat_purge - dt)
                self.cl_hold = max(self.cl_hold, 0.2 if self.cat_purge > 0.0 else 0.0)
            cl_ok = (
                not wb_failed
                and wb_ready
                and t_run > cal.CL_DELAY_S(ect_start)
                and ect >= cal.CL_MIN_ECT_C
                and not self.pe
                and not self.dfco
                and not self.rev_cut
                and self.cl_hold <= 0.0
            )

            if self.dfco or self.rev_cut:
                cut = True
                self.status = FS_OL_DRIVE
                lam = cal.LAMBDA_WARMUP(ect)
                self.closed_loop = False
            elif self.pe:
                self.status = FS_OL_DRIVE
                lam = min(cal.PE_LAMBDA(rpm), lam_ol)
                self.closed_loop = False
            elif self.cat_purge > 0.0 and not wb_failed:
                self.status = FS_OL_DRIVE
                lam = cal.CAT_PURGE_LAMBDA
                self.closed_loop = False
            elif wb_failed:
                self.status = FS_OL_FAULT
                lam = min(lam_ol, cal.FAULT_LAMBDA)
                self.closed_loop = False
            elif cl_ok:
                self.status = FS_CL
                self.closed_loop = True
                lam = 1.0 + cal.CAT_HEAT_LEAN * cat_heat + self.dtrim
            else:
                self.status = FS_OL_TEMP
                lam = lam_ol
                self.closed_loop = False

        # ---- closed-loop trims
        if self.closed_loop:
            self.lambda_target_cl = lam
            err = self.wb_filt - lam  # positive = lean -> add fuel
            self.stft_i = clamp(self.stft_i + cal.STFT_KI * err * dt, -cal.TRIM_LIMIT, cal.TRIM_LIMIT)
            self.stft = clamp(cal.STFT_KP * err + self.stft_i, -cal.TRIM_LIMIT, cal.TRIM_LIMIT)
            if ect > cal.LTFT_MIN_ECT_C and not ltft_hold:
                d = clamp(cal.LTFT_RATE * self.stft_i, -cal.LTFT_MAX_SLEW, cal.LTFT_MAX_SLEW) * dt
                new = clamp(self.ltft[self.cell] + d, -cal.TRIM_LIMIT, cal.TRIM_LIMIT)
                d = new - self.ltft[self.cell]
                self.ltft[self.cell] = new
                self.stft_i -= d
            if nb_ready and cat_est_c > 400.0 and ect > 60.0:
                self.dtrim = clamp(
                    self.dtrim + cal.DTRIM_GAIN * (nb_volts - cal.DTRIM_TARGET_V) * dt,
                    -cal.DTRIM_LIMIT,
                    cal.DTRIM_LIMIT,
                )
            self.mod = cal.LAMBDA_MOD_AMPL * math.sin(2.0 * math.pi * cal.LAMBDA_MOD_HZ * t)
        else:
            self.stft = 0.0
            self.stft_i = 0.0
            self.mod = 0.0
        self.lambda_cmd = lam

        # ---- fuel mass and transient compensation
        mult = 1.0 + self.stft + self.ltft[self.cell]
        desired = 0.0 if cut else cyl_flow_gps / (cal.AFR_STOICH * lam * (1.0 + self.mod)) * mult
        desired = max(0.0, desired - purge_fuel_gps) if desired > 0.0 else 0.0
        self.desired_gps = desired
        x_e, tau_e = film_params(ect, map_kpa * 1000.0)
        evap = self.film_est / tau_e
        inj = max(0.0, (desired - evap) / (1.0 - x_e)) if desired > 0.0 else 0.0
        self.film_est = max(0.0, self.film_est + dt * (x_e * inj - evap))
        self.injected_gps = inj

        injections_per_s = 4.0 * rpm / 120.0
        if inj <= 0.0 or injections_per_s <= 0.0:
            self.fuel_mg = 0.0
            self.pw_ms = 0.0
            return
        self.fuel_mg = inj * 1000.0 / injections_per_s
        flow = cal.INJ_FLOW_MG_MS * math.sqrt(max(cal.RAIL_KPA + baro_kpa - map_kpa, 50.0) / cal.RAIL_KPA)
        pw = self.fuel_mg / flow + cal.INJ_DEADTIME_MS(vbatt)
        cycle_ms = 120000.0 / max(rpm, 1.0)
        self.pw_ms = min(pw, 0.9 * cycle_ms)
