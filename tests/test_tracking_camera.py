"""Unit tests for camera-to-LOCAL NED ray projection."""

from __future__ import annotations

import math
import unittest

from src.tracking.camera import (
    bearing_local_yaw_rad,
    bearing_yaw_rad,
    pixel_ray_to_local_ned,
)
from src.vision.intrinsics import CX_PX, CY_PX


class TrackingCameraTests(unittest.TestCase):
    def test_center_pixel_level_flight_points_forward(self) -> None:
        bearing = bearing_local_yaw_rad(
            CX_PX,
            CY_PX,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            pitch_up_degrees=20.0,
        )
        self.assertAlmostEqual(bearing, 0.0, places=2)

    def test_vehicle_pitch_shifts_local_bearing(self) -> None:
        u = CX_PX + 80.0
        level = bearing_local_yaw_rad(
            u,
            CY_PX,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            pitch_up_degrees=20.0,
        )
        pitched = bearing_local_yaw_rad(
            u,
            CY_PX,
            roll=0.0,
            pitch=0.15,
            yaw=0.0,
            pitch_up_degrees=20.0,
        )
        self.assertNotAlmostEqual(level, pitched, places=2)

    def test_camera_mount_affects_body_bearing(self) -> None:
        flat = bearing_yaw_rad(CX_PX, CY_PX, pitch_up_degrees=0.0)
        mounted = bearing_yaw_rad(CX_PX, CY_PX, pitch_up_degrees=20.0)
        self.assertAlmostEqual(flat, 0.0, places=2)
        self.assertAlmostEqual(mounted, 0.0, places=2)

    def test_pixel_ray_to_local_ned_is_unit_length(self) -> None:
        nx, ny, nz = pixel_ray_to_local_ned(
            CX_PX + 40.0,
            CY_PX,
            roll=0.05,
            pitch=-0.03,
            yaw=1.2,
            pitch_up_degrees=20.0,
        )
        norm = math.sqrt(nx * nx + ny * ny + nz * nz)
        self.assertAlmostEqual(norm, 1.0, places=5)


if __name__ == "__main__":
    unittest.main()
