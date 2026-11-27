"""Mode 09 vehicle information: VIN (with check digit), calibration ID, CVN, ECU name (DESIGN §7.5)."""

from __future__ import annotations

import zlib
from dataclasses import dataclass

_TRANSLIT = {
    **{str(d): d for d in range(10)},
    **dict(zip("ABCDEFGH", range(1, 9))),
    **dict(zip("JKLMN", range(1, 6))),
    "P": 7,
    "R": 9,
    **dict(zip("STUVWXYZ", range(2, 10))),
}
_WEIGHTS = (8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2)

OBD_STANDARD = {"us": 0x01, "eu": 0x06, "world": 0x09}


def vin_check_digit(vin: str) -> str:
    vin = vin.upper()
    if len(vin) != 17:
        raise ValueError("VIN must be 17 characters")
    total = 0
    for ch, w in zip(vin, _WEIGHTS):
        if ch not in _TRANSLIT:
            raise ValueError(f"invalid VIN character {ch!r} (I, O and Q are not allowed)")
        total += _TRANSLIT[ch] * w
    r = total % 11
    return "X" if r == 10 else str(r)


def vin_is_valid(vin: str) -> bool:
    try:
        return vin_check_digit(vin) == vin.upper()[8]
    except ValueError:
        return False


def with_check_digit(vin: str) -> str:
    """Return ``vin`` with position 9 replaced by the correct check digit."""
    vin = vin.upper()
    probe = vin[:8] + "0" + vin[9:]
    return vin[:8] + vin_check_digit(probe) + vin[9:]


DEFAULT_VIN = with_check_digit("1SMEC16D0FS000001")


def _calibration_crc() -> int:
    """CVN: CRC-32 over the ECU calibration tables (changes whenever the calibration changes)."""
    from ..ecu import cal

    parts = []
    for name in sorted(dir(cal)):
        if name.isupper():
            v = getattr(cal, name)
            if hasattr(v, "xs"):
                v = (getattr(v, "xs"), getattr(v, "ys", None), getattr(v, "z", None))
            parts.append(f"{name}={v!r}")
    return zlib.crc32("\n".join(parts).encode())


@dataclass
class VehicleInfo:
    vin: str = DEFAULT_VIN
    calid: str = "SIM16DOHC-E0101"
    ecu_acronym: str = "ECM"
    ecu_name: str = "EngineControl"
    region: str = "eu"

    def __post_init__(self) -> None:
        self.vin = self.vin.upper()
        if len(self.vin) != 17:
            raise ValueError("VIN must be 17 characters")
        if len(self.calid) > 16:
            raise ValueError("Calibration ID is at most 16 characters")
        if self.region not in OBD_STANDARD:
            raise ValueError(f"region must be one of {sorted(OBD_STANDARD)}")

    @property
    def obd_standard(self) -> int:
        return OBD_STANDARD[self.region]

    @property
    def vin_bytes(self) -> bytes:
        return self.vin.encode("ascii")

    @property
    def calid_bytes(self) -> bytes:
        return self.calid.encode("ascii").ljust(16, b"\x00")

    @property
    def cvn_bytes(self) -> bytes:
        return _calibration_crc().to_bytes(4, "big")

    @property
    def ecu_name_bytes(self) -> bytes:
        """J1979 format: 4-char acronym, '-', 15-char name, NUL padded (20 bytes)."""
        acr = self.ecu_acronym.encode("ascii")[:4].ljust(4, b"\x00")
        name = self.ecu_name.encode("ascii")[:15].ljust(15, b"\x00")
        return acr + b"-" + name
