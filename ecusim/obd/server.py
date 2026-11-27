# SPDX-License-Identifier: GPL-3.0-or-later
"""OBD-II request handling per SAE J1979 on ISO 15765-4 (DESIGN §7.3).

Transport-independent: ``handle(request, functional)`` returns the response bytes, or ``None``
when the ECU must stay silent.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from ..ecu.diag.codes import encode_dtc
from .pids import MODE01, SUPPORT_PIDS, active_support_pids, support_bitmap
from .vehicle_info import VehicleInfo

NRC_SERVICE_NOT_SUPPORTED = 0x11
NRC_SUBFUNCTION_NOT_SUPPORTED = 0x12
NRC_INCORRECT_LENGTH = 0x13
NRC_CONDITIONS_NOT_CORRECT = 0x22

MODE09_SUPPORT = (0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0)
MODE06_SUPPORT = MODE09_SUPPORT

# Freeze frame PIDs (DESIGN §6.5); PID 02 is the DTC that caused it
FREEZE_PIDS = {0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x1F, 0x2F, 0x33,
               0x42, 0x44, 0x46}


@dataclass
class ServerConfig:
    clear_while_running: bool = False  # J1979 allows ECUs to reject Mode 04 with the engine running


class ObdServer:
    def __init__(
        self,
        source: Callable[[], dict | None],
        info: VehicleInfo | None = None,
        config: ServerConfig | None = None,
        on_clear: Callable[[], None] | None = None,
    ):
        self.source = source
        self.info = info or VehicleInfo()
        self.cfg = config or ServerConfig()
        self.on_clear = on_clear
        self.mode01_pids = set(MODE01)
        self.mode09 = {
            0x02: lambda d: (1, self.info.vin_bytes),
            0x04: lambda d: (1, self.info.calid_bytes),
            0x06: lambda d: (1, self.info.cvn_bytes),
            0x08: lambda d: (len(d["iumpr"]), b"".join(v.to_bytes(2, "big") for v in d["iumpr"])),
            0x0A: lambda d: (1, self.info.ecu_name_bytes),
        }

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _nrc(sid: int, code: int, functional: bool) -> bytes | None:
        return None if functional else bytes([0x7F, sid, code])

    # ------------------------------------------------------------------ entry point
    def handle(self, req: bytes, functional: bool) -> bytes | None:
        d = self.source()
        if d is None or not d.get("powered", False) or not req:
            return None
        sid = req[0]
        if sid == 0x01:
            return self._mode01(req, d, functional)
        if sid == 0x02:
            return self._mode02(req, d, functional)
        if sid == 0x06:
            return self._mode06(req, d, functional)
        if sid == 0x09:
            return self._mode09(req, d, functional)
        if sid in (0x03, 0x07, 0x0A):
            return self._dtcs(req, d, functional)
        if sid == 0x04:
            return self._clear(req, d, functional)
        return self._nrc(sid, NRC_SERVICE_NOT_SUPPORTED, functional)

    # ------------------------------------------------------------------ Mode 01
    def _mode01(self, req: bytes, d: dict, functional: bool) -> bytes | None:
        pids = req[1:]
        if not 1 <= len(pids) <= 6:
            return self._nrc(0x01, NRC_INCORRECT_LENGTH, functional)
        d = dict(d, obd_standard=self.info.obd_standard)
        support = active_support_pids(self.mode01_pids)
        out = bytearray([0x41])
        for pid in pids:
            if pid in support:
                out += bytes([pid]) + support_bitmap(self.mode01_pids, pid)
            elif pid in MODE01 and pid in self.mode01_pids:
                out += bytes([pid]) + MODE01[pid].encode(d)
        if len(out) == 1:
            return self._nrc(0x01, NRC_SUBFUNCTION_NOT_SUPPORTED, functional)
        return bytes(out)

    # ------------------------------------------------------------------ Mode 09
    # ------------------------------------------------------------------ Mode 02
    def _mode02(self, req: bytes, d: dict, functional: bool) -> bytes | None:
        body = req[1:]
        if not body or len(body) % 2 or len(body) > 6:
            return self._nrc(0x02, NRC_INCORRECT_LENGTH, functional)
        ff = d.get("freeze_frame")
        out = bytearray([0x42])
        for pid, frame in zip(body[0::2], body[1::2]):
            if frame != 0:
                continue
            if pid in SUPPORT_PIDS:
                if pid == 0x00 or any(p > pid for p in FREEZE_PIDS):
                    out += bytes([pid, 0]) + support_bitmap(FREEZE_PIDS, pid)
            elif pid == 0x02:
                out += bytes([0x02, 0]) + (encode_dtc(ff[0]) if ff else b"\x00\x00")
            elif ff and pid in FREEZE_PIDS and pid in MODE01:
                vals = dict(ff[1], obd_standard=self.info.obd_standard)
                out += bytes([pid, 0]) + MODE01[pid].encode(vals)
        if len(out) == 1:
            return self._nrc(0x02, NRC_SUBFUNCTION_NOT_SUPPORTED, functional)
        return bytes(out)

    # ------------------------------------------------------------------ Mode 06
    def _mode06(self, req: bytes, d: dict, functional: bool) -> bytes | None:
        mids = req[1:]
        results: dict[int, bytes] = d.get("mode06", {})
        if not 1 <= len(mids) <= 6:
            return self._nrc(0x06, NRC_INCORRECT_LENGTH, functional)
        supported = set(results)
        if all(m in MODE06_SUPPORT for m in mids):
            out = bytearray([0x46])
            for m in mids:
                if m == 0x00 or any(s > m for s in supported):
                    out += bytes([m]) + support_bitmap(supported, m)
            return bytes(out) if len(out) > 1 else self._nrc(0x06, NRC_SUBFUNCTION_NOT_SUPPORTED, functional)
        if len(mids) != 1 or mids[0] not in results:
            return self._nrc(0x06, NRC_SUBFUNCTION_NOT_SUPPORTED, functional)
        return b"\x46" + results[mids[0]]

    def _mode09(self, req: bytes, d: dict, functional: bool) -> bytes | None:
        types = req[1:]
        if not 1 <= len(types) <= 6:
            return self._nrc(0x09, NRC_INCORRECT_LENGTH, functional)
        supported = set(self.mode09)
        if all(t in MODE09_SUPPORT for t in types):
            out = bytearray([0x49])
            for t in types:
                if t == 0x00 or any(s > t for s in supported):
                    out += bytes([t]) + support_bitmap(supported, t)
            return bytes(out) if len(out) > 1 else self._nrc(0x09, NRC_SUBFUNCTION_NOT_SUPPORTED, functional)
        if len(types) != 1:
            # Only one non-support info type per request
            return self._nrc(0x09, NRC_SUBFUNCTION_NOT_SUPPORTED, functional)
        t = types[0]
        if t not in self.mode09:
            return self._nrc(0x09, NRC_SUBFUNCTION_NOT_SUPPORTED, functional)
        count, data = self.mode09[t](d)
        return bytes([0x49, t, count]) + data

    # ------------------------------------------------------------------ Modes 03/07/0A/04 (DTC store arrives in M4)
    def _dtcs(self, req: bytes, d: dict, functional: bool) -> bytes | None:
        if len(req) != 1:
            return self._nrc(req[0], NRC_INCORRECT_LENGTH, functional)
        key = {0x03: "dtc_confirmed", 0x07: "dtc_pending", 0x0A: "dtc_permanent"}[req[0]]
        codes = d.get(key, ())
        return bytes([req[0] + 0x40, len(codes)]) + b"".join(encode_dtc(c) for c in codes)

    def _clear(self, req: bytes, d: dict, functional: bool) -> bytes | None:
        if len(req) != 1:
            return self._nrc(0x04, NRC_INCORRECT_LENGTH, functional)
        if d.get("running") and not self.cfg.clear_while_running:
            # J1979: conditions not correct is reported even on functional requests
            return bytes([0x7F, 0x04, NRC_CONDITIONS_NOT_CORRECT])
        if self.on_clear:
            self.on_clear()
        return b"\x44"
