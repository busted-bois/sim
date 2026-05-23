"""Camera pitch compensation for body-frame bearings."""

from __future__ import annotations

import math

from src.vision.intrinsics import CX_PX, CY_PX, FX_PX, FY_PX


def pixel_to_camera_ray(u: float, v: float) -> tuple[float, float, float]:
    """Unit ray in camera frame (x right, y down, z forward)."""
    x = (float(u) - CX_PX) / FX_PX
    y = (float(v) - CY_PX) / FY_PX
    z = 1.0
    norm = math.sqrt(x * x + y * y + z * z)
    return x / norm, y / norm, z / norm


def camera_ray_to_body_ned(
    u: float,
    v: float,
    *,
    pitch_up_degrees: float = 20.0,
) -> tuple[float, float, float]:
    """Map pixel to forward/right/down unit vector in body NED (X fwd, Y right, Z down)."""
    cx, cy, cz = pixel_to_camera_ray(u, v)
    pitch_rad = math.radians(-float(pitch_up_degrees))
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    # Rotate camera ray about camera Y into body: forward X, right Y, down Z
    bx = cp * cz - sp * cy
    by = cx
    bz = sp * cz + cp * cy
    norm = math.sqrt(bx * bx + by * by + bz * bz)
    return bx / norm, by / norm, bz / norm


def bearing_yaw_rad(u: float, v: float, *, pitch_up_degrees: float = 20.0) -> float:
    """Horizontal bearing in body frame (radians, + right)."""
    bx, by, _bz = camera_ray_to_body_ned(u, v, pitch_up_degrees=pitch_up_degrees)
    return math.atan2(by, bx)
