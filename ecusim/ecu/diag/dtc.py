# SPDX-License-Identifier: GPL-3.0-or-later
"""DTC lifecycle (DESIGN §6.1): pending -> confirmed on the 2nd consecutive failing trip, MIL,
permanent DTCs, MIL off after 3 good trips, erase after 40 warm-ups, Mode 04 clear."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Callable

from .codes import PRIORITY_CODES

GOOD_TRIPS_FOR_MIL_OFF = 3
WARMUPS_TO_ERASE = 40


@dataclass
class DtcRecord:
    code: str
    pending: bool = False
    confirmed: bool = False
    permanent: bool = False
    mil: bool = False  # this DTC is commanding the MIL
    failed_prev_trip: bool = False
    failed_this_trip: bool = False
    passed_this_trip: bool = False
    good_trips: int = 0
    warmups_since_fail: int = 0
    fail_count: int = 0


@dataclass
class FreezeFrame:
    code: str
    values: dict = field(default_factory=dict)


class DtcManager:
    def __init__(self, capture: Callable[[], dict] | None = None):
        self.records: dict[str, DtcRecord] = {}
        self.freeze_frame: FreezeFrame | None = None
        self.capture = capture or (lambda: {})
        self.version = 0  # bumps on every visible change (for cheap snapshot caching)

    # ------------------------------------------------------------------ queries
    def _with(self, attr: str) -> list[str]:
        return sorted(c for c, r in self.records.items() if getattr(r, attr))

    @property
    def confirmed(self) -> list[str]:
        return self._with("confirmed")

    @property
    def pending(self) -> list[str]:
        return self._with("pending")

    @property
    def permanent(self) -> list[str]:
        return self._with("permanent")

    @property
    def mil_on(self) -> bool:
        return any(r.mil for r in self.records.values())

    # ------------------------------------------------------------------ test results
    def _store_freeze(self, code: str) -> None:
        if self.freeze_frame is None or (code in PRIORITY_CODES and self.freeze_frame.code not in PRIORITY_CODES):
            self.freeze_frame = FreezeFrame(code, dict(self.capture()))

    def report(self, code: str, failed: bool) -> None:
        """A monitor finished an evaluation of ``code`` this trip."""
        r = self.records.get(code)
        if r is None:
            if not failed:
                return  # nothing stored, nothing to do (permanent-only records are kept below)
            r = self.records[code] = DtcRecord(code)
        if not failed:
            if not r.passed_this_trip:
                r.passed_this_trip = True
            return
        r.failed_this_trip = True
        r.fail_count += 1
        r.warmups_since_fail = 0
        changed = False
        if not r.pending:
            r.pending = True
            changed = True
            if code in PRIORITY_CODES:
                self._store_freeze(code)
        if r.failed_prev_trip and not r.mil:
            r.confirmed = r.mil = r.permanent = True
            r.good_trips = 0
            self._store_freeze(code)
            changed = True
        if r.mil:
            r.good_trips = 0
        if changed:
            self.version += 1

    # ------------------------------------------------------------------ cycle events
    def end_trip(self) -> None:
        for code in list(self.records):
            r = self.records[code]
            if r.failed_this_trip:
                r.failed_prev_trip = True
            elif r.passed_this_trip:
                r.failed_prev_trip = False
                r.pending = False
                if r.mil:
                    r.good_trips += 1
                    if r.good_trips >= GOOD_TRIPS_FOR_MIL_OFF:
                        r.mil = False
                        r.permanent = False
                elif r.permanent:
                    r.permanent = False  # monitor passed after a clear / MIL off
            r.failed_this_trip = r.passed_this_trip = False
            if not (r.pending or r.confirmed or r.permanent or r.failed_prev_trip):
                del self.records[code]
        self.version += 1

    def warmup_cycle(self) -> None:
        for code in list(self.records):
            r = self.records[code]
            if r.confirmed and not r.mil and not r.failed_this_trip:
                r.warmups_since_fail += 1
                if r.warmups_since_fail >= WARMUPS_TO_ERASE:
                    r.confirmed = False
                    if not (r.pending or r.permanent):
                        del self.records[code]
        if self.freeze_frame and self.freeze_frame.code not in self.records:
            self.freeze_frame = None
        self.version += 1

    def clear(self) -> None:
        """Mode 04: everything except permanent DTCs."""
        for code in list(self.records):
            r = self.records[code]
            if r.permanent:
                self.records[code] = DtcRecord(code, permanent=True)
            else:
                del self.records[code]
        self.freeze_frame = None
        self.version += 1

    # ------------------------------------------------------------------ persistence
    def export(self, include_kam: bool = True) -> dict:
        recs = [asdict(r) for r in self.records.values()]
        out = {"records": recs}
        if include_kam and self.freeze_frame:
            out["freeze_frame"] = {"code": self.freeze_frame.code, "values": self.freeze_frame.values}
        return out

    def load(self, data: dict) -> None:
        self.records = {}
        for d in data.get("records", []):
            r = DtcRecord(**{k: v for k, v in d.items() if k in DtcRecord.__dataclass_fields__})
            self.records[r.code] = r
        ff = data.get("freeze_frame")
        self.freeze_frame = FreezeFrame(ff["code"], dict(ff["values"])) if ff else None
        self.version += 1

    def load_permanent_only(self, codes: list[str]) -> None:
        """After a battery disconnect only non-volatile permanent DTCs survive."""
        self.records = {c: DtcRecord(c, permanent=True) for c in codes}
        self.freeze_frame = None
        self.version += 1
