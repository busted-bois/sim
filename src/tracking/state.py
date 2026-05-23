"""Tracking state vectors in LOCAL_NED relative to arm origin."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class TrackingHealth:
    status: str
    reason: str
    imu_sample_count: int
    imu_rate_hz: float | None
    vision_correction_count: int
    origin_set: bool
    armed: bool


@dataclass(frozen=True, slots=True)
class TrackingState:
    sim_time_ns: int
    t_s: float
    position_ned: tuple[float, float, float]
    velocity_ned: tuple[float, float, float]
    attitude_rpy: tuple[float, float, float]
    armed: bool
    origin_set: bool

    @property
    def altitude_m(self) -> float:
        return max(0.0, -self.position_ned[2])


def _is_finite3(a: float, b: float, c: float) -> bool:
    return math.isfinite(a) and math.isfinite(b) and math.isfinite(c)
