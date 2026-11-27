"""Sensor fault modes (DESIGN §8): applied in the sensor layer so the ECU must detect them."""

from __future__ import annotations

from dataclasses import dataclass

OPEN = "open"
SHORT_GND = "short_gnd"
SHORT_V = "short_5v"
OFFSET = "offset"
GAIN = "gain"
STUCK = "stuck"
SLOW = "slow"
NOISY = "noisy"

CIRCUIT_MODES = (OPEN, SHORT_GND, SHORT_V)
SIGNAL_MODES = (OFFSET, GAIN, STUCK, SLOW, NOISY)
ALL_MODES = CIRCUIT_MODES + SIGNAL_MODES

MODE_LABELS = {
    OPEN: "open circuit",
    SHORT_GND: "short to ground",
    SHORT_V: "short to 5 V",
    OFFSET: "offset",
    GAIN: "gain error",
    STUCK: "stuck at current value",
    SLOW: "slow response",
    NOISY: "noisy / erratic",
}


@dataclass(frozen=True)
class SensorFault:
    mode: str
    amount: float = 0.0  # offset in engineering units, gain fraction, extra time constant (s), noise sigma
