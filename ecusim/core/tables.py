"""Lookup tables and small numeric helpers (pure Python, no numpy, for speed on scalars)."""

from __future__ import annotations

from bisect import bisect_right
from typing import Sequence


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def interp1(xs: Sequence[float], ys: Sequence[float], x: float) -> float:
    """Linear interpolation with end clamping."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect_right(xs, x) - 1
    x0 = xs[i]
    return ys[i] + (ys[i + 1] - ys[i]) * (x - x0) / (xs[i + 1] - x0)


class Table1D:
    __slots__ = ("xs", "ys")

    def __init__(self, xs: Sequence[float], ys: Sequence[float]):
        if len(xs) != len(ys) or len(xs) < 2:
            raise ValueError("Table1D needs matching axes with at least 2 points")
        if any(b <= a for a, b in zip(xs, xs[1:])):
            raise ValueError("Table1D x axis must be strictly increasing")
        self.xs = list(xs)
        self.ys = list(ys)

    def __call__(self, x: float) -> float:
        return interp1(self.xs, self.ys, x)


class Table2D:
    """Bilinear table. ``z[row][col]``: rows follow the y axis, columns the x axis."""

    __slots__ = ("xs", "ys", "z")

    def __init__(self, xs: Sequence[float], ys: Sequence[float], z: Sequence[Sequence[float]]):
        if len(z) != len(ys) or any(len(row) != len(xs) for row in z):
            raise ValueError("Table2D shape mismatch")
        for axis in (xs, ys):
            if any(b <= a for a, b in zip(axis, axis[1:])):
                raise ValueError("Table2D axes must be strictly increasing")
        self.xs = list(xs)
        self.ys = list(ys)
        self.z = [list(r) for r in z]

    @staticmethod
    def _locate(axis: list[float], v: float) -> tuple[int, float]:
        if v <= axis[0]:
            return 0, 0.0
        if v >= axis[-1]:
            return len(axis) - 2, 1.0
        i = bisect_right(axis, v) - 1
        return i, (v - axis[i]) / (axis[i + 1] - axis[i])

    def __call__(self, x: float, y: float) -> float:
        i, fx = self._locate(self.xs, x)
        j, fy = self._locate(self.ys, y)
        z0 = self.z[j]
        z1 = self.z[j + 1]
        a = z0[i] + (z0[i + 1] - z0[i]) * fx
        b = z1[i] + (z1[i + 1] - z1[i]) * fx
        return a + (b - a) * fy


def lowpass_alpha(dt: float, tau: float) -> float:
    """Exact discrete first-order filter coefficient."""
    import math

    if tau <= 0.0:
        return 1.0
    return 1.0 - math.exp(-dt / tau)
