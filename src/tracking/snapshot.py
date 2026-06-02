"""Unified downstream tracking payload."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.tracking.state import TrackingHealth, TrackingState

if TYPE_CHECKING:
    import numpy as np


@dataclass(frozen=True, slots=True)
class TrackingSnapshot:
    sim_time_ns: int
    image_rgb: np.ndarray | None
    position_ned: tuple[float, float, float]
    velocity_ned: tuple[float, float, float]
    attitude_rpy: tuple[float, float, float]
    health: TrackingHealth
    state: TrackingState | None = None
