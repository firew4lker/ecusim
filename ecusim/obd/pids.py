# SPDX-License-Identifier: GPL-3.0-or-later
"""SAE J1979 Mode 01 PID registry: encoders (ECU side) and decoders (tester side) (DESIGN §7.4).

Encoders take the diagnostic snapshot dict built by ``ecusim.obd.datasource``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


def u8(x: float) -> int:
    return max(0, min(255, int(round(x))))


def u16(x: float) -> bytes:
    v = max(0, min(65535, int(round(x))))
    return bytes([v >> 8, v & 0xFF])


def s16(x: float) -> bytes:
    v = max(-32768, min(32767, int(round(x))))
    return (v & 0xFFFF).to_bytes(2, "big")


def pct8(x: float) -> bytes:
    return bytes([u8(x * 255.0 / 100.0)])


def temp8(x: float) -> bytes:
    return bytes([u8(x + 40.0)])


def trim8(x: float) -> bytes:
    return bytes([u8(x * 128.0 / 100.0 + 128.0)])


def _w(b: bytes) -> int:
    return (b[0] << 8) | b[1]


@dataclass(frozen=True)
class Pid:
    pid: int
    name: str
    unit: str
    length: int
    encode: Callable[[dict], bytes]
    decode: Callable[[bytes], object]


def _wb_volts(lam: float) -> float:
    return max(0.0, min(7.99, 2.5 + 2.0 * (lam - 1.0)))


def _enc_24(d: dict) -> bytes:
    if not d["wb_ready"]:
        return b"\x00\x00\x00\x00"
    lam = min(d["wb_lambda"], 1.999)
    return u16(lam * 32768.0) + u16(_wb_volts(lam) * 8192.0)


def _enc_34(d: dict) -> bytes:
    if not d["wb_ready"]:
        return b"\x00\x00\x80\x00"
    lam = min(d["wb_lambda"], 1.999)
    return u16(lam * 32768.0) + u16((d["wb_pump_ma"] + 128.0) * 256.0)


def _enc_15(d: dict) -> bytes:
    v = bytes([u8(d["nb_volts"] * 200.0)])
    if d["fuel_status"] == 2 and d["nb_ready"]:
        return v + trim8(d["nb_trim_pct"])
    return v + b"\xff"


_DEFS: list[Pid] = [
    Pid(0x01, "Monitor status since DTCs cleared", "", 4, lambda d: bytes(d["mon01"]), lambda b: tuple(b)),
    Pid(0x03, "Fuel system status", "", 2, lambda d: bytes([d["fuel_status"], 0]), lambda b: (b[0], b[1])),
    Pid(0x04, "Calculated engine load", "%", 1, lambda d: pct8(d["calc_load"]), lambda b: b[0] * 100 / 255),
    Pid(0x05, "Engine coolant temperature", "°C", 1, lambda d: temp8(d["ect"]), lambda b: b[0] - 40),
    Pid(0x06, "Short term fuel trim bank 1", "%", 1, lambda d: trim8(d["stft"]), lambda b: (b[0] - 128) * 100 / 128),
    Pid(0x07, "Long term fuel trim bank 1", "%", 1, lambda d: trim8(d["ltft"]), lambda b: (b[0] - 128) * 100 / 128),
    Pid(0x0B, "Intake manifold absolute pressure", "kPa", 1, lambda d: bytes([u8(d["map"])]), lambda b: b[0]),
    Pid(0x0C, "Engine speed", "rpm", 2, lambda d: u16(d["rpm"] * 4.0), lambda b: _w(b) / 4),
    Pid(0x0D, "Vehicle speed", "km/h", 1, lambda d: bytes([u8(d["vss"])]), lambda b: b[0]),
    Pid(0x0E, "Timing advance", "°", 1, lambda d: bytes([u8((d["spark"] + 64.0) * 2.0)]), lambda b: b[0] / 2 - 64),
    Pid(0x0F, "Intake air temperature", "°C", 1, lambda d: temp8(d["iat"]), lambda b: b[0] - 40),
    Pid(0x10, "MAF air flow rate", "g/s", 2, lambda d: u16(d["maf"] * 100.0), lambda b: _w(b) / 100),
    Pid(0x11, "Throttle position", "%", 1, lambda d: pct8(d["tps_a"]), lambda b: b[0] * 100 / 255),
    Pid(0x13, "Oxygen sensors present", "", 1, lambda d: b"\x03", lambda b: b[0]),
    Pid(0x15, "O2 B1S2 voltage / trim", "V,%", 2, _enc_15,
        lambda b: (b[0] / 200, None if b[1] == 0xFF else (b[1] - 128) * 100 / 128)),
    Pid(0x1C, "OBD standard", "", 1, lambda d: bytes([d["obd_standard"]]), lambda b: b[0]),
    Pid(0x1F, "Run time since engine start", "s", 2, lambda d: u16(d["run_time"]), lambda b: _w(b)),
    Pid(0x21, "Distance with MIL on", "km", 2, lambda d: u16(d["dist_mil"]), lambda b: _w(b)),
    Pid(0x24, "O2 S1 equivalence ratio / voltage", "λ,V", 4, _enc_24,
        lambda b: (_w(b[0:2]) * 2 / 65536, _w(b[2:4]) * 8 / 65536)),
    Pid(0x2E, "Commanded evaporative purge", "%", 1, lambda d: pct8(d["purge"]), lambda b: b[0] * 100 / 255),
    Pid(0x2F, "Fuel tank level input", "%", 1, lambda d: pct8(d["fuel_level"]), lambda b: b[0] * 100 / 255),
    Pid(0x30, "Warm-ups since codes cleared", "", 1, lambda d: bytes([u8(d["warmups"])]), lambda b: b[0]),
    Pid(0x31, "Distance since codes cleared", "km", 2, lambda d: u16(d["dist_clear"]), lambda b: _w(b)),
    Pid(0x32, "Evap system vapour pressure", "Pa", 2, lambda d: s16(d["evap_pa"] * 4.0),
        lambda b: int.from_bytes(b[:2], "big", signed=True) / 4),
    Pid(0x33, "Absolute barometric pressure", "kPa", 1, lambda d: bytes([u8(d["baro"])]), lambda b: b[0]),
    Pid(0x34, "O2 S1 equivalence ratio / current", "λ,mA", 4, _enc_34,
        lambda b: (_w(b[0:2]) * 2 / 65536, _w(b[2:4]) / 256 - 128)),
    Pid(0x3C, "Catalyst temperature B1S1", "°C", 2, lambda d: u16((d["cat_temp"] + 40.0) * 10.0),
        lambda b: _w(b) / 10 - 40),
    Pid(0x41, "Monitor status this drive cycle", "", 4, lambda d: bytes(d["mon41"]), lambda b: tuple(b)),
    Pid(0x42, "Control module voltage", "V", 2, lambda d: u16(d["vbatt"] * 1000.0), lambda b: _w(b) / 1000),
    Pid(0x43, "Absolute load value", "%", 2, lambda d: u16(d["abs_load"] * 255.0 / 100.0), lambda b: _w(b) * 100 / 255),
    Pid(0x44, "Commanded equivalence ratio", "λ", 2, lambda d: u16(min(d["lambda_cmd"], 1.999) * 32768.0),
        lambda b: _w(b) * 2 / 65536),
    Pid(0x45, "Relative throttle position", "%", 1, lambda d: pct8(d["tps_rel"]), lambda b: b[0] * 100 / 255),
    Pid(0x46, "Ambient air temperature", "°C", 1, lambda d: temp8(d["ambient"]), lambda b: b[0] - 40),
    Pid(0x47, "Absolute throttle position B", "%", 1, lambda d: pct8(d["tps_b"]), lambda b: b[0] * 100 / 255),
    Pid(0x49, "Accelerator pedal position D", "%", 1, lambda d: pct8(d["app_d"]), lambda b: b[0] * 100 / 255),
    Pid(0x4A, "Accelerator pedal position E", "%", 1, lambda d: pct8(d["app_e"]), lambda b: b[0] * 100 / 255),
    Pid(0x4C, "Commanded throttle actuator", "%", 1, lambda d: pct8(d["throttle_cmd"]), lambda b: b[0] * 100 / 255),
    Pid(0x4D, "Time run with MIL on", "min", 2, lambda d: u16(d["time_mil"]), lambda b: _w(b)),
    Pid(0x4E, "Time since codes cleared", "min", 2, lambda d: u16(d["time_clear"]), lambda b: _w(b)),
    Pid(0x51, "Fuel type", "", 1, lambda d: b"\x01", lambda b: b[0]),
    Pid(0x5C, "Engine oil temperature", "°C", 1, lambda d: temp8(d["oil"]), lambda b: b[0] - 40),
    Pid(0x5E, "Engine fuel rate", "L/h", 2, lambda d: u16(d["fuel_rate"] * 20.0), lambda b: _w(b) / 20),
]

MODE01: dict[int, Pid] = {p.pid: p for p in _DEFS}
SUPPORT_PIDS = (0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0)


def support_bitmap(supported: set[int], base: int) -> bytes:
    """4-byte bitmap for PIDs base+1 .. base+0x20; the last bit flags the next range."""
    bits = 0
    for n in range(1, 33):
        pid = base + n
        if n == 32:
            hit = any(p > base + 0x20 for p in supported)
        else:
            hit = pid in supported
        if hit:
            bits |= 1 << (32 - n)
    return bits.to_bytes(4, "big")


def active_support_pids(supported: set[int]) -> set[int]:
    """Support PIDs the ECU answers: 0x00 always, the next one only if anything lies beyond it."""
    out = {0x00}
    for base in SUPPORT_PIDS[1:]:
        if any(p > base for p in supported):
            out.add(base)
    return out


def decode_bitmap(data: bytes, base: int) -> set[int]:
    bits = int.from_bytes(data[:4], "big")
    return {base + n for n in range(1, 33) if bits & (1 << (32 - n))}
