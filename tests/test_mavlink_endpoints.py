import os
import unittest
from unittest.mock import patch

from src.mavlink_endpoints import resolve_control_transport


class ResolveControlTransportTests(unittest.TestCase):
    def test_defaults_to_airsim(self) -> None:
        self.assertEqual(resolve_control_transport({}), "airsim")

    def test_auto_uses_mavlink_when_heartbeat_seen(self) -> None:
        config = {"control": {"transport": "auto"}}
        with patch(
            "src.mavlink_endpoints.first_mavlink_heartbeat_endpoint",
            return_value="udp:127.0.0.1:14540",
        ):
            self.assertEqual(resolve_control_transport(config), "mavlink")

    def test_auto_falls_back_to_airsim_when_no_heartbeat(self) -> None:
        config = {"control": {"transport": "auto"}}
        with patch("src.mavlink_endpoints.first_mavlink_heartbeat_endpoint", return_value=None):
            self.assertEqual(resolve_control_transport(config), "airsim")

    def test_mavlink_simpleflight_profile_forces_airsim(self) -> None:
        config = {
            "control": {
                "transport": "mavlink",
                "mavlink": {"airsim_profile": {"vehicle_type": "SimpleFlight"}},
            }
        }
        self.assertEqual(resolve_control_transport(config), "airsim")

    def test_override_allows_mavlink_with_simpleflight_profile(self) -> None:
        config = {
            "control": {
                "transport": "mavlink",
                "mavlink": {"airsim_profile": {"vehicle_type": "SimpleFlight"}},
            }
        }
        with patch.dict(os.environ, {"AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT": "1"}, clear=False):
            self.assertEqual(resolve_control_transport(config), "mavlink")

    def test_unsupported_transport_falls_back_to_airsim(self) -> None:
        config = {"control": {"transport": "weird_mode"}}
        self.assertEqual(resolve_control_transport(config), "airsim")


if __name__ == "__main__":
    unittest.main()
