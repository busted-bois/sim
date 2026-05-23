"""Low-rate vision correction using gate/ring detections."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.tracking.camera import bearing_yaw_rad, normalized_to_pixel
from src.tracking.imu_propagator import ImuPropagator
from src.vision.processing import blue_ring_info_normalized


@dataclass(frozen=True, slots=True)
class VisionCorrectionResult:
    applied: bool
    yaw_delta_rad: float
    reason: str


class VisionCorrector:
    def __init__(
        self,
        *,
        yaw_gain: float = 0.15,
        max_yaw_step_rad: float = 0.12,
        max_center_offset: float = 0.35,
        pitch_up_degrees: float = 20.0,
    ) -> None:
        self._yaw_gain = float(yaw_gain)
        self._max_yaw_step_rad = max(0.01, float(max_yaw_step_rad))
        self._max_center_offset = max(0.05, float(max_center_offset))
        self._pitch_up_degrees = float(pitch_up_degrees)

    def correct(self, image_rgb: np.ndarray, propagator: ImuPropagator) -> VisionCorrectionResult:
        info = blue_ring_info_normalized(image_rgb)
        if info is None:
            return VisionCorrectionResult(False, 0.0, "no_landmark")
        nx, ny, r_frac = info
        if r_frac <= 0.0 or abs(nx) > self._max_center_offset:
            return VisionCorrectionResult(False, 0.0, "landmark_out_of_range")

        u, v = normalized_to_pixel(nx, ny)
        yaw_err = bearing_yaw_rad(u, v, pitch_up_degrees=self._pitch_up_degrees)
        step = _clamp(yaw_err * self._yaw_gain, -self._max_yaw_step_rad, self._max_yaw_step_rad)
        propagator.state.yaw = _wrap_pi(propagator.state.yaw + step)
        return VisionCorrectionResult(True, step, "landmark_yaw_blue")


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
