"""Unit tests for SLAM-inspired exploration helpers."""

from __future__ import annotations

import math
import unittest

from src.control.exploration.slam import ExplorationSlam, parse_slam_settings


class ExplorationSlamTests(unittest.TestCase):
    def test_loop_closure_after_path(self) -> None:
        slam = ExplorationSlam(
            parse_slam_settings({"loop_closure_radius_m": 2.0, "loop_closure_min_path_m": 5.0}),
            spawn_x_m=0.0,
            spawn_y_m=0.0,
            spawn_yaw_rad=0.0,
            start_s=0.0,
        )
        slam.update_pose(6.0, 0.0, 0.0)
        slam.update_pose(6.0, 6.0, math.pi / 2)
        slam.update_pose(0.5, 0.5, math.pi)
        self.assertTrue(slam.loop_closure_detected())
        self.assertTrue(slam.consume_loop_closure(100.0))
        self.assertFalse(slam.consume_loop_closure(110.0))

    def test_active_exploration_bias_bounded(self) -> None:
        slam = ExplorationSlam(
            parse_slam_settings({"active_exploration_gain_deg_s": 15.0}),
            spawn_x_m=0.0,
            spawn_y_m=0.0,
            spawn_yaw_rad=0.0,
            start_s=0.0,
        )
        for i in range(36):
            slam._bearing_visits[i] = 10
        slam._bearing_visits[5] = 0
        slam._yaw = 0.0
        bias = slam.active_exploration_yaw_bias_deg()
        self.assertLessEqual(abs(bias), 15.0)

    def test_landmark_registers_cell(self) -> None:
        slam = ExplorationSlam(
            parse_slam_settings({}),
            spawn_x_m=0.0,
            spawn_y_m=0.0,
            spawn_yaw_rad=0.0,
            start_s=0.0,
        )
        slam.register_landmark("blue", 0.0, half_fov_rad=math.radians(45.0))
        self.assertEqual(slam.status().landmark_count, 1)


if __name__ == "__main__":
    unittest.main()
