"""ECU calibration (DESIGN §5). Values a calibration engineer would have mapped on the dyno."""

from __future__ import annotations

from ..calib.engine import _MBT_LOAD, _MBT_RPM, MBT, VE_RPM, klsa_deg
from ..core.tables import Table1D, Table2D

# --- Physical constants the ECU assumes -------------------------------------------------------
DISPLACEMENT_M3 = 1.598e-3
MANIFOLD_VOLUME_M3 = 2.5e-3
FULL_CHARGE_G = 1.184 * 0.3995  # g/cyl at STP
INJ_FLOW_MG_MS = 2.71  # nominal at 300 kPa
RAIL_KPA = 300.0
INJ_DEADTIME_MS = Table1D([6, 8, 10, 12, 14, 16], [2.5, 1.6, 1.1, 0.85, 0.65, 0.55])
VE_NOMINAL = VE_RPM  # speed-density backup / cranking air estimate
THROTTLE_CDA_FULL = 1.7e-3
THROTTLE_CDA_REST = 4.0e-6
AFR_STOICH = 14.7

# --- Modes ------------------------------------------------------------------------------------
RUN_RPM = 450.0  # crank -> run
STALL_RPM = 250.0
SYNC_REVS = 2.0  # CKP gap + CMP edge
REV_LIMIT_CUT = 6700.0
REV_LIMIT_RESUME = 6600.0

# --- Fuelling ---------------------------------------------------------------------------------
ECT_AXIS = [-30, -10, 0, 15, 30, 40, 50, 70, 85, 100]
LAMBDA_CRANK = Table1D(ECT_AXIS, [0.62, 0.70, 0.74, 0.80, 0.85, 0.88, 0.90, 0.95, 0.97, 0.97])
LAMBDA_WARMUP = Table1D(ECT_AXIS, [0.80, 0.85, 0.88, 0.93, 0.96, 0.97, 0.98, 1.00, 1.00, 1.00])
ASE_INITIAL = Table1D([-30, 0, 15, 40, 70, 90], [0.35, 0.20, 0.12, 0.08, 0.04, 0.02])
ASE_TAU_S = Table1D([-30, 0, 15, 40, 70, 90], [40.0, 15.0, 8.0, 6.0, 4.0, 3.0])
CL_DELAY_S = Table1D([-30, -20, 0, 15, 40, 80], [120.0, 90.0, 45.0, 22.0, 12.0, 8.0])
CL_MIN_ECT_C = 10.0
PE_LAMBDA = Table1D([1000, 3000, 5000, 6500], [0.92, 0.88, 0.86, 0.85])
PE_ENTER_LOAD = 0.80
PE_EXIT_LOAD = 0.72
PE_ENTER_PEDAL = 70.0  # % pedal travel
STFT_KP = 0.25
STFT_KI = 1.5
TRIM_LIMIT = 0.25
LTFT_RATE = 0.05  # fraction of STFT integrator moved per second
LTFT_MAX_SLEW = 0.002  # per second
LTFT_MIN_ECT_C = 70.0
LAMBDA_MOD_AMPL = 0.015
LAMBDA_MOD_HZ = 1.0
DTRIM_GAIN = 0.004  # lambda per volt per second
DTRIM_LIMIT = 0.02
DTRIM_TARGET_V = 0.65
DFCO_MIN_ECT_C = 60.0
CAT_PURGE_MAX_S = 3.0  # rich after fuel cut until the downstream sensor goes rich
CAT_PURGE_LAMBDA = 0.92
FAULT_LAMBDA = 0.97  # open loop with a failed upstream sensor
LIMP_REV_LIMIT = 2500.0
DFCO_DELAY_S = 0.3

# --- Idle / air -------------------------------------------------------------------------------
IDLE_BASE_RPM = Table1D([-30, 0, 15, 40, 70, 85], [1350, 1250, 1150, 1000, 850, 750])
IDLE_FF_GPS = Table1D([-30, 0, 15, 40, 70, 90], [6.0, 4.8, 4.3, 3.6, 3.0, 2.7])  # at base idle rpm
FLARE_RPM = 350.0
FLARE_TAU_S = 2.5
IDLE_KP = 0.008  # g/s per rpm
IDLE_KI = 0.006  # g/s per rpm per s
IDLE_I_LIMIT = 3.0
IDLE_P_NEG_LIMIT = 1.0  # g/s
IDLE_P_POS_LIMIT = 5.0  # anti-stall
DECEL_AIR_EXP = 0.7
CRANK_THROTTLE = 0.035
DASHPOT_GPS = 3.0
DASHPOT_TAU_S = 1.0
AC_IDLE_UP_RPM = 100.0
AC_AIR_GPS = 0.8

# --- Spark ------------------------------------------------------------------------------------
CRANK_SPARK = 8.0
IDLE_SPARK = Table1D([-30, 0, 15, 40, 70, 90], [16.0, 15.0, 14.0, 14.0, 15.0, 15.0])
IDLE_SPARK_KP = 0.03  # deg per rpm
IDLE_SPARK_LIMIT = 8.0


def _build_spark_table() -> Table2D:
    rows = []
    for load in _MBT_LOAD:
        row = []
        for rpm in _MBT_RPM:
            mbt = MBT(rpm, load)
            row.append(round(min(mbt - 1.0, klsa_deg(rpm, load, 90.0, 30.0) - 1.5), 1))
        rows.append(row)
    return Table2D(_MBT_RPM, _MBT_LOAD, rows)


SPARK_BASE = _build_spark_table()

# --- Catalyst heating -------------------------------------------------------------------------
CAT_HEAT_ECT_MIN = 5.0
CAT_HEAT_ECT_MAX = 60.0
CAT_HEAT_START_S = 1.5
CAT_HEAT_END_S = 30.0
CAT_HEAT_SPARK = 0.0
CAT_HEAT_AIR_GPS = 3.0
CAT_HEAT_LEAN = 0.01

# --- O2 heaters -------------------------------------------------------------------------------
WB_HEATER_DELAY_S = 2.0
WB_READY_C = 700.0
NB_DEWPOINT_CAT_C = 200.0
NB_DEWPOINT_HOLD_S = 10.0
NB_READY_C = 350.0

# --- Cooling fan ------------------------------------------------------------------------------
FAN_LOW_ON, FAN_LOW_OFF = 98.0, 94.0
FAN_HIGH_ON, FAN_HIGH_OFF = 105.0, 101.0

# --- VVT --------------------------------------------------------------------------------------
VVT_TARGET = Table2D(
    [1000, 1500, 2500, 3500, 4500, 5500, 6500],
    [0.2, 0.4, 0.6, 0.8, 1.0],
    [
        [0, 10, 20, 15, 10, 5, 0],
        [0, 15, 30, 25, 15, 8, 0],
        [0, 15, 30, 30, 20, 10, 5],
        [0, 10, 25, 30, 25, 15, 10],
        [0, 8, 20, 30, 28, 20, 12],
    ],
)
VVT_MIN_OIL_C = 40.0

# --- Knock control ------------------------------------------------------------------------------
KNOCK_MARGIN = 0.5  # signal above the rpm-dependent background that counts as knock
KNOCK_STEP_DEG = 2.0
KNOCK_MAX_DEG = 8.0
KNOCK_RECOVER_DEG_S = 0.5
KNOCK_FAILSAFE_DEG = 5.0

# --- Purge --------------------------------------------------------------------------------------
PURGE_MIN_RUN_S = 60.0
