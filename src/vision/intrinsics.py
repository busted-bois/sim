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
