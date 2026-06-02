"""Camera pitch compensation for body-frame and LOCAL NED bearings."""

from __future__ import annotations

import math

from src.tracking.imu_propagator import rotate_body_to_ned
from src.vision.intrinsics import CX_PX, CY_PX, FX_PX, FY_PX, HEIGHT_PX, WIDTH_PX


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


def normalized_to_pixel(nx: float, ny: float) -> tuple[float, float]:
    """Map normalized image coords (see vision.processing) to pixel u, v."""
    u = 0.5 * WIDTH_PX * (1.0 + float(nx))
    v = 0.5 * HEIGHT_PX * (1.0 + float(ny))
    return u, v


def pixel_ray_to_local_ned(
    u: float,
    v: float,
    *,
    roll: float,
    pitch: float,
    yaw: float,
    pitch_up_degrees: float = 20.0,
) -> tuple[float, float, float]:
    """Camera pixel to LOCAL NED unit vector via body mount + vehicle attitude."""
    bx, by, bz = camera_ray_to_body_ned(u, v, pitch_up_degrees=pitch_up_degrees)
    return rotate_body_to_ned(bx, by, bz, roll, pitch, yaw)


def bearing_local_yaw_rad(
    u: float,
    v: float,
    *,
    roll: float,
    pitch: float,
    yaw: float,
    pitch_up_degrees: float = 20.0,
) -> float:
    """Horizontal LOCAL NED bearing (radians, + east of north)."""
    nx, ny, _nz = pixel_ray_to_local_ned(
        u,
        v,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        pitch_up_degrees=pitch_up_degrees,
    )
    return math.atan2(ny, nx)
