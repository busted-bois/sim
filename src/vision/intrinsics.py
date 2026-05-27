"""Official challenge pinhole camera (640x360, square pixels, no distortion)."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any, Final

WIDTH_PX: Final[int] = 640
HEIGHT_PX: Final[int] = 360
FX_PX: Final[float] = 320.0
FY_PX: Final[float] = 320.0
CX_PX: Final[float] = 320.0
CY_PX: Final[float] = 180.0
OFFICIAL_PHYSICS_HZ: Final[float] = 120.0
OFFICIAL_VISION_FPS: Final[float] = 30.0
OFFICIAL_CAMERA_PITCH_UP_DEG: Final[float] = 20.0


def official_resolution() -> tuple[int, int]:
    return WIDTH_PX, HEIGHT_PX


def official_resolution_list() -> list[int]:
    return [WIDTH_PX, HEIGHT_PX]


def horizontal_fov_degrees() -> float:
    return math.degrees(2.0 * math.atan(WIDTH_PX / (2.0 * FX_PX)))


def camera_matrix_k() -> list[list[float]]:
    return [
        [FX_PX, 0.0, CX_PX],
        [0.0, FY_PX, CY_PX],
        [0.0, 0.0, 1.0],
    ]


def horizontal_fov_degrees_from_vision(vision_cfg: Mapping[str, Any] | None) -> float:
    if vision_cfg is None:
        return horizontal_fov_degrees()
    return float(vision_cfg.get("fov_degrees", horizontal_fov_degrees()))


def yaw_mapping_half_fov_degrees(
    vision_cfg: Mapping[str, Any] | None, *, floor_deg: float = 10.0
) -> float:
    return max(floor_deg, horizontal_fov_degrees_from_vision(vision_cfg) / 2.0)
