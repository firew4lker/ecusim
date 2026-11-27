# SPDX-License-Identifier: GPL-3.0-or-later
"""DTC catalogue (DESIGN §6.3) and SAE J2012 two-byte encoding."""

from __future__ import annotations

DESCRIPTIONS: dict[str, str] = {
    "P0010": "Intake camshaft actuator circuit",
    "P0011": "Intake camshaft timing over-advanced",
    "P0012": "Intake camshaft timing over-retarded",
    "P0016": "Crankshaft/camshaft position correlation",
    "P0030": "HO2S heater control circuit B1S1",
    "P0036": "HO2S heater control circuit B1S2",
    "P0101": "MAF circuit range/performance",
    "P0102": "MAF circuit low",
    "P0103": "MAF circuit high",
    "P0106": "MAP circuit range/performance",
    "P0107": "MAP circuit low",
    "P0108": "MAP circuit high",
    "P0111": "IAT circuit range/performance",
    "P0112": "IAT circuit low",
    "P0113": "IAT circuit high",
    "P0116": "ECT circuit range/performance",
    "P0117": "ECT circuit low",
    "P0118": "ECT circuit high",
    "P0121": "TPS A circuit range/performance",
    "P0122": "TPS A circuit low",
    "P0123": "TPS A circuit high",
    "P0128": "Coolant below thermostat regulating temperature",
    "P0130": "O2 sensor circuit B1S1",
    "P0131": "O2 sensor circuit low B1S1",
    "P0132": "O2 sensor circuit high B1S1",
    "P0133": "O2 sensor slow response B1S1",
    "P0134": "O2 sensor no activity B1S1",
    "P0135": "O2 sensor heater circuit B1S1",
    "P0136": "O2 sensor circuit B1S2",
    "P0137": "O2 sensor circuit low B1S2",
    "P0138": "O2 sensor circuit high B1S2",
    "P0139": "O2 sensor slow response B1S2",
    "P0140": "O2 sensor no activity B1S2",
    "P0141": "O2 sensor heater circuit B1S2",
    "P0171": "System too lean bank 1",
    "P0172": "System too rich bank 1",
    "P0201": "Injector circuit cylinder 1",
    "P0202": "Injector circuit cylinder 2",
    "P0203": "Injector circuit cylinder 3",
    "P0204": "Injector circuit cylinder 4",
    "P0221": "TPS B circuit range/performance",
    "P0222": "TPS B circuit low",
    "P0223": "TPS B circuit high",
    "P0300": "Random/multiple cylinder misfire detected",
    "P0301": "Cylinder 1 misfire detected",
    "P0302": "Cylinder 2 misfire detected",
    "P0303": "Cylinder 3 misfire detected",
    "P0304": "Cylinder 4 misfire detected",
    "P0325": "Knock sensor circuit",
    "P0327": "Knock sensor circuit low",
    "P0328": "Knock sensor circuit high",
    "P0335": "Crankshaft position sensor circuit",
    "P0336": "Crankshaft position sensor range/performance",
    "P0340": "Camshaft position sensor circuit",
    "P0341": "Camshaft position sensor range/performance",
    "P0351": "Ignition coil 1 primary circuit",
    "P0352": "Ignition coil 2 primary circuit",
    "P0353": "Ignition coil 3 primary circuit",
    "P0354": "Ignition coil 4 primary circuit",
    "P0420": "Catalyst efficiency below threshold bank 1",
    "P0441": "EVAP incorrect purge flow",
    "P0442": "EVAP small leak detected",
    "P0443": "EVAP purge valve circuit",
    "P0446": "EVAP vent control performance",
    "P0451": "EVAP pressure sensor range/performance",
    "P0452": "EVAP pressure sensor low",
    "P0453": "EVAP pressure sensor high",
    "P0455": "EVAP large leak detected",
    "P0456": "EVAP very small leak detected",
    "P0461": "Fuel level sensor range/performance",
    "P0480": "Cooling fan 1 control circuit",
    "P0496": "EVAP high purge flow",
    "P0500": "Vehicle speed sensor",
    "P0506": "Idle control system rpm lower than expected",
    "P0507": "Idle control system rpm higher than expected",
    "P0562": "System voltage low",
    "P0563": "System voltage high",
    "P2101": "Throttle actuator control motor circuit range/performance",
    "P2122": "Pedal position sensor D circuit low",
    "P2123": "Pedal position sensor D circuit high",
    "P2127": "Pedal position sensor E circuit low",
    "P2128": "Pedal position sensor E circuit high",
    "P2135": "Throttle position sensor A/B voltage correlation",
    "P2138": "Pedal position sensor D/E voltage correlation",
    "P2195": "O2 sensor signal stuck lean B1S1",
    "P2196": "O2 sensor signal stuck rich B1S1",
}

# Misfire and fuel-system faults may overwrite an older freeze frame and store one at pending.
PRIORITY_CODES = {"P0171", "P0172", "P0300", "P0301", "P0302", "P0303", "P0304"}

_LETTER = {"P": 0, "C": 1, "B": 2, "U": 3}
_LETTER_REV = {v: k for k, v in _LETTER.items()}


def describe(code: str) -> str:
    return DESCRIPTIONS.get(code, "Unknown")


def encode_dtc(code: str) -> bytes:
    """'P0171' -> b'\\x01\\x71' (SAE J2012)."""
    code = code.upper()
    if len(code) != 5 or code[0] not in _LETTER:
        raise ValueError(f"bad DTC {code!r}")
    first = int(code[1], 16)
    if first > 3:
        raise ValueError(f"bad DTC {code!r}")
    hi = (_LETTER[code[0]] << 6) | (first << 4) | int(code[2], 16)
    lo = int(code[3:5], 16)
    return bytes([hi, lo])


def decode_dtc(b: bytes) -> str:
    hi, lo = b[0], b[1]
    return f"{_LETTER_REV[hi >> 6]}{(hi >> 4) & 0x3}{hi & 0xF:X}{lo:02X}"
