"""AirSim settings.json must enable PX4 TCP HIL for uv run sim (MAVLink transport)."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from src import sim_launch


class TestPx4MavlinkSettings(unittest.TestCase):
    def test_ensure_px4_mavlink_settings_enables_hil_tcp(self) -> None:
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

            with patch.object(sim_launch, "_airsim_settings_path", fake_path):
                sim_launch._ensure_px4_mavlink_settings(41451, config=config)

            data = json.loads(settings_path.read_text(encoding="utf-8"))
            px4 = data["Vehicles"]["PX4"]
            self.assertTrue(px4.get("UseTcp"))
            self.assertEqual(px4.get("TcpPort"), sim_launch.PX4_HIL_TCP_PORT)
            self.assertEqual(px4.get("VehicleType"), "PX4Multirotor")
            self.assertEqual(px4.get("QgcPort"), 14550)
            self.assertIn("Cameras", px4)

    def test_px4_hil_vehicle_settings_never_disables_tcp(self) -> None:
        config = {
            "control": {
                "mavlink": {
                    "airsim_profile": {
                        "lock_step": False,
                    }
                }
            }
        }
        vehicle = sim_launch._px4_hil_vehicle_settings(config, enable_trace=False)
        self.assertTrue(vehicle["UseTcp"])
        self.assertEqual(vehicle["TcpPort"], 4560)


if __name__ == "__main__":
    unittest.main()
