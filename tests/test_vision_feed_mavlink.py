"""Vision feed must capture frames for MAVLink + autonomous_explore."""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock

from src.vision.feed import AirSimRpcVisionFeed, vision_feed_from_config


class TestVisionFeedMavlink(unittest.TestCase):
    def test_vision_feed_from_config_uses_airsim_rpc_when_client_set(self) -> None:
        client = MagicMock()
        config = {"vision": {"enabled": True, "fps": 30.0}}
        feed = vision_feed_from_config(config, airsim_client=client)
        self.assertIsInstance(feed, AirSimRpcVisionFeed)
        self.assertTrue(feed.enabled)

    def test_altitude_transition_creeps_forward(self) -> None:
        from src.control.exploration.scheduler import (
            ExplorationScheduler,
            build_wander_tick_input,
            parse_exploration_settings,
        )

        sched = ExplorationScheduler(
            parse_exploration_settings(
                {
                    "panorama_creep_speed_ms": 0.4,
                    "altitude_layers_m": [3.5, 5.0],
                },
                hold_altitude_m=5.0,
                max_altitude_m=50.0,
            ),
            start_s=0.0,
            initial_yaw_rad=0.0,
            initial_z_ned=-5.0,
        )
        out = sched.tick(
            build_wander_tick_input(
                now_s=1.0,
                dt_s=0.1,
                yaw_rad=0.0,
                z_ned=-5.0,
                cos_yaw=1.0,
                sin_yaw=0.0,
                base_vz=0.0,
            )
        )
        self.assertGreater(out.fwd_speed, 0.0)


if __name__ == "__main__":
    unittest.main()
