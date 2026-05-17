from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class VisionFrame:
    seq: int
    timestamp_s: float
    frame_age_s: float
    width: int
    height: int
    image_rgb: np.ndarray


@dataclass(frozen=True, slots=True)
class VisionStats:
    configured_fps: float
    active_fps: float
    capture_attempts: int
    capture_successes: int
    capture_failures: int
    scheduler_dropped_ticks: int
    consumer_dropped_frames: int
    latest_seq: int
    latest_frame_age_s: float
    effective_capture_hz: float


class VisionFeed:
    def __init__(self, client=None, config: dict | None = None) -> None:
        self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_latest(self) -> VisionFrame | None:
        return None

    def get_stats(self) -> VisionStats:
        return VisionStats(
            configured_fps=0.0,
            active_fps=0.0,
            capture_attempts=0,
            capture_successes=0,
            capture_failures=0,
            scheduler_dropped_ticks=0,
            consumer_dropped_frames=0,
            latest_seq=0,
            latest_frame_age_s=0.0,
            effective_capture_hz=0.0,
        )
