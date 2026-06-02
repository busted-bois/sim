"""Unit tests for SLAM-inspired exploration helpers."""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from src.control.exploration.slam import ExplorationSlam, parse_slam_settings


class ExplorationSlamTests(unittest.TestCase):
    def test_loop_closure_after_path(self) -> None:
        slam = ExplorationSlam(
            parse_slam_settings({"loop_closure_radius_m": 2.0, "loop_closure_min_path_m": 5.0}),
            spawn_x_m=0.0,
            spawn_y_m=0.0,
            spawn_yaw_rad=0.0,
        )
        slam.update_pose(6.0, 0.0, 0.0)
        slam.update_pose(6.0, 6.0, math.pi / 2)
        slam.update_pose(0.5, 0.5, math.pi)
        self.assertTrue(slam.loop_closure_detected())
        self.assertTrue(slam.consume_loop_closure(100.0))
        self.assertFalse(slam.consume_loop_closure(110.0))

    def test_log_odds_and_frontier_bias(self) -> None:
        slam = ExplorationSlam(
            parse_slam_settings({"active_exploration_gain_deg_s": 15.0}),
            spawn_x_m=0.0,
            spawn_y_m=0.0,
            spawn_yaw_rad=0.0,
        )
        scores = [2.0, 8.0, 9.0, 8.0, 2.0]
        slam.integrate_depth_columns(5, scores, yaw_rad=0.0, half_fov_rad=math.radians(45.0))
        bias = slam.exploration_yaw_bias_deg()
        self.assertLessEqual(abs(bias), 15.0)
        self.assertGreater(slam.status().known_cells, 0)

    def test_landmark_uses_depth_range(self) -> None:
        slam = ExplorationSlam(
            parse_slam_settings({"landmark_default_range_m": 6.0}),
            spawn_x_m=0.0,
            spawn_y_m=0.0,
            spawn_yaw_rad=0.0,
        )
        slam.integrate_depth_columns(
            5, [1.0, 9.0, 9.5, 9.0, 1.0], yaw_rad=0.0, half_fov_rad=math.radians(45.0)
        )
        slam.register_landmark("blue", 0.0, half_fov_rad=0.0)
        with tempfile.TemporaryDirectory() as tmp:
            data = json.loads(slam.export_map(Path(tmp) / "m.json").read_text(encoding="utf-8"))
        self.assertLess(data["landmarks"][0]["range_m"], 6.0)

    def test_imu_yaw_assist(self) -> None:
        slam = ExplorationSlam(parse_slam_settings({"imu_yaw_assist_gain": 1.0}), **self._spawn())
        rate = slam.imu_yaw_assist_deg_s(0.1)
        self.assertGreater(rate, 0.0)

    def test_tilted_pitch_changes_depth_bearing(self) -> None:
        slam = ExplorationSlam(parse_slam_settings({}), **self._spawn())
        scores = [2.0, 8.0, 9.0, 8.0, 2.0]
        slam.integrate_depth_columns(
            5,
            scores,
            yaw_rad=0.0,
            half_fov_rad=math.radians(45.0),
            roll_rad=0.0,
            pitch_rad=0.0,
            pitch_up_degrees=20.0,
            column_center_u=[128.0, 192.0, 320.0, 448.0, 512.0],
            image_center_v=180.0,
        )
        level_cells = set(slam._log_odds.keys())
        slam_tilt = ExplorationSlam(parse_slam_settings({}), **self._spawn())
        slam_tilt.integrate_depth_columns(
            5,
            scores,
            yaw_rad=0.0,
            half_fov_rad=math.radians(45.0),
            roll_rad=0.0,
            pitch_rad=0.2,
            pitch_up_degrees=20.0,
            column_center_u=[128.0, 192.0, 320.0, 448.0, 512.0],
            image_center_v=180.0,
        )
        tilt_cells = set(slam_tilt._log_odds.keys())
        self.assertNotEqual(level_cells, tilt_cells)

    def test_export_map_writes_json(self) -> None:
        slam = ExplorationSlam(parse_slam_settings({"export_enabled": True}), **self._spawn())
        slam.update_pose(1.0, 0.0, 0.0)
        slam.register_landmark("red", 0.2, half_fov_rad=0.2, range_m=4.0)
        with tempfile.TemporaryDirectory() as tmp:
            path = slam.export_map(Path(tmp) / "map.json")
            assert path is not None
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(len(data["landmarks"]), 1)
            self.assertIn("metrics", data)

    def test_export_map_omits_ned_payload_by_default(self) -> None:
        slam = ExplorationSlam(parse_slam_settings({"export_enabled": True}), **self._spawn())
        ned_extra = {"snapshot": {"has_position": True}, "health": {"status": "ok"}}
        with tempfile.TemporaryDirectory() as tmp:
            path = slam.export_map(Path(tmp) / "map.json", ned=ned_extra)
            assert path is not None
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertNotIn("ned", data)

    def test_export_map_includes_ned_payload_when_enabled(self) -> None:
        slam = ExplorationSlam(
            parse_slam_settings({"export_enabled": True, "include_ned_payload": True}),
            **self._spawn(),
        )
        ned_extra = {"snapshot": {"has_position": True}, "health": {"status": "ok"}}
        with tempfile.TemporaryDirectory() as tmp:
            path = slam.export_map(Path(tmp) / "map.json", ned=ned_extra)
            assert path is not None
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(data["ned"], ned_extra)

    @staticmethod
    def _spawn() -> dict[str, float]:
        return {"spawn_x_m": 0.0, "spawn_y_m": 0.0, "spawn_yaw_rad": 0.0}


if __name__ == "__main__":
    unittest.main()
