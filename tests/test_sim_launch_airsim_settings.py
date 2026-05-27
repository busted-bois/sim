"""AirSim settings.json must enable PX4 TCP HIL for uv run sim (MAVLink transport)."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from src import airsim_settings, sim_launch


class TestPx4MavlinkSettings(unittest.TestCase):
    def test_ensure_px4_hil_settings_enables_hil_tcp(self) -> None:
        config = {
            "vision": {"camera_name": "0", "resolution": [640, 360], "fov_degrees": 90},
            "camera": {"pose_offset": [0.35, 0.0, -0.05], "pitch_up_degrees": 20.0},
            "control": {
                "mavlink": {
                    "airsim_profile": {
                        "qgc_port": 14550,
                        "control_port_local": 14540,
                        "control_port_remote": 14580,
                    }
                }
            },
        }
        with tempfile.TemporaryDirectory() as tmp:
            settings_path = Path(tmp) / "settings.json"

            def fake_path() -> Path:
                return settings_path

            with patch.object(airsim_settings, "settings_path", fake_path):
                airsim_settings.ensure_px4_hil_settings(41451, config)

            data = json.loads(settings_path.read_text(encoding="utf-8"))
            px4 = data["Vehicles"][airsim_settings.PX4_VEHICLE_NAME]
            self.assertTrue(px4.get("UseTcp"))
            self.assertEqual(px4.get("TcpPort"), airsim_settings.PX4_HIL_TCP_PORT)
            self.assertEqual(px4.get("VehicleType"), "PX4Multirotor")
            self.assertEqual(px4.get("QgcPort"), 14550)
            self.assertIn("Cameras", px4)

    def test_px4_hil_vehicle_never_disables_tcp(self) -> None:
        config = {
            "control": {
                "mavlink": {
                    "airsim_profile": {
                        "lock_step": False,
                    }
                }
            }
        }
        vehicle = airsim_settings.px4_hil_vehicle_from_config(config, enable_trace=False)
        airsim_settings.assert_px4_hil_vehicle(vehicle)

    def test_assert_px4_hil_rejects_udp_only(self) -> None:
        with self.assertRaises(ValueError):
            airsim_settings.assert_px4_hil_vehicle(
                {"VehicleType": "PX4Multirotor", "UseTcp": False, "TcpPort": 4560}
            )

    def test_ensure_launch_settings_routes_mavlink_only(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            settings_path = Path(tmp) / "settings.json"

            def fake_path() -> Path:
                return settings_path

            with patch.object(airsim_settings, "settings_path", fake_path):
                path = airsim_settings.ensure_launch_settings(
                    "mavlink",
                    41451,
                    {},
                    view_mode="Fpv",
                )
                self.assertIs(path, settings_path)
                self.assertIsNone(
                    airsim_settings.ensure_launch_settings("airsim", 41451, {})
                )

    def test_sim_launch_wrapper_matches_module(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            settings_path = Path(tmp) / "settings.json"

            def fake_path() -> Path:
                return settings_path

            with patch.object(airsim_settings, "settings_path", fake_path):
                sim_launch._ensure_px4_mavlink_settings(41451, config={})

            px4 = json.loads(settings_path.read_text())["Vehicles"]["PX4"]
            self.assertTrue(px4["UseTcp"])


if __name__ == "__main__":
    unittest.main()
