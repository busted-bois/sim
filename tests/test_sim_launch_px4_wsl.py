"""Tests for MAVLink one-command PX4 WSL autostart helpers."""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from src import mavlink_prereq, sim_launch


class TestPx4WslAutostart(unittest.TestCase):
    def test_wants_mavlink_session_from_hud(self) -> None:
        config = {
            "control": {
                "transport": "airsim",
                "mavlink": {"position_hud": {"enabled": True}},
            }
        }
        self.assertTrue(mavlink_prereq.wants_mavlink_session(config))

    def test_bootstrap_skips_when_heartbeat_present(self) -> None:
        config = {"control": {"transport": "mavlink", "mavlink": {"px4_wsl": {"auto_start": True}}}}
        with patch(
            "src.mavlink_endpoints.first_mavlink_heartbeat_endpoint",
            return_value="udpin:0.0.0.0:14550",
        ):
            with patch.object(sim_launch, "_start_px4_wsl") as start_mock:
                sim_launch._bootstrap_px4_wsl_for_mavlink(config)
                start_mock.assert_not_called()

    def test_bootstrap_starts_px4_when_no_heartbeat(self) -> None:
        config = {"control": {"transport": "mavlink", "mavlink": {"px4_wsl": {"auto_start": True}}}}
        proc = MagicMock()
        proc.stdout = iter([])
        with patch("src.sim_launch.sys.platform", "win32"):
            with patch(
                "src.mavlink_endpoints.first_mavlink_heartbeat_endpoint",
                return_value=None,
            ):
                with patch.object(mavlink_prereq, "wsl_available", return_value=(True, "")):
                    with patch.object(mavlink_prereq, "px4_wsl_binary_ready", return_value=True):
                        with patch.object(sim_launch, "_start_px4_wsl", return_value=proc):
                            with patch.object(
                                sim_launch,
                                "_wait_for_px4_airsim_connected",
                                return_value=True,
                            ):
                                sim_launch._bootstrap_px4_wsl_for_mavlink(config)


if __name__ == "__main__":
    unittest.main()
