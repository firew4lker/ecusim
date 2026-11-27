"""On-board monitor test results for Mode 06 (DESIGN §6.4), stored as already-scaled raw integers."""

from __future__ import annotations

# Unit and scaling IDs (SAE J1979 appendix)
UAS_RAW = 0x01
UAS_MV = 0x0B  # 0.001 V
UAS_MS = 0x10  # 1 ms
UAS_COUNTS = 0x24
UAS_PCT = 0x2F  # 0.01 %
UAS_PCT_SIGNED = 0xAF  # 0.01 %, signed


def _u16(v: float, signed: bool) -> int:
    v = int(round(v))
    if signed:
        v = max(-32768, min(32767, v))
        return v & 0xFFFF
    return max(0, min(65535, v))


class Mode06Store:
    def __init__(self) -> None:
        self.supported: set[int] = set()
        self.results: dict[int, dict[int, tuple[int, int, int, int]]] = {}
        self.done: set[tuple[int, int]] = set()  # tests completed since DTCs were cleared
        self.version = 0

    def declare(self, mid: int, tids: list[tuple[int, int, float, float]]) -> None:
        """Declare a supported MID with (tid, uasid, min, max) rows; values start at zero."""
        self.supported.add(mid)
        rows = self.results.setdefault(mid, {})
        for tid, uas, lo, hi in tids:
            signed = uas >= 0x80
            rows.setdefault(tid, (uas, 0, _u16(lo, signed), _u16(hi, signed)))
        self.version += 1

    def set(self, mid: int, tid: int, value: float, lo: float | None = None, hi: float | None = None) -> None:
        uas, _old, old_lo, old_hi = self.results[mid][tid]
        signed = uas >= 0x80
        self.results[mid][tid] = (
            uas,
            _u16(value, signed),
            old_lo if lo is None else _u16(lo, signed),
            old_hi if hi is None else _u16(hi, signed),
        )
        self.done.add((mid, tid))
        self.version += 1

    def clear(self) -> None:
        for rows in self.results.values():
            for tid, (uas, _v, lo, hi) in rows.items():
                rows[tid] = (uas, 0, lo, hi)
        self.done.clear()
        self.version += 1

    def encode(self, mid: int) -> bytes:
        out = bytearray()
        for tid in sorted(self.results.get(mid, {})):
            uas, v, lo, hi = self.results[mid][tid]
            if (mid, tid) not in self.done:
                v = lo = hi = 0  # J1979: value and limits read 0 until the test has completed
            out += bytes([mid, tid, uas]) + v.to_bytes(2, "big") + lo.to_bytes(2, "big") + hi.to_bytes(2, "big")
        return bytes(out)

    def frozen(self) -> dict[int, bytes]:
        return {mid: self.encode(mid) for mid in self.supported}

    def export(self) -> dict:
        out: dict = {str(mid): {str(t): list(v) for t, v in rows.items()} for mid, rows in self.results.items()}
        out["done"] = [list(k) for k in sorted(self.done)]
        return out

    def load(self, data: dict) -> None:
        self.done = {tuple(k) for k in data.get("done", []) if tuple(k)[0] in self.results}
        for mid_s, rows in data.items():
            if mid_s == "done":
                continue
            mid = int(mid_s)
            if mid not in self.results:
                continue
            for tid_s, v in rows.items():
                tid = int(tid_s)
                if tid in self.results[mid]:
                    self.results[mid][tid] = tuple(int(x) for x in v)
        self.version += 1
