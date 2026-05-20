import unittest

from src.mavlink_endpoints import (
    candidate_mavlink_endpoints,
    mavlink_endpoint_from_config,
    resolve_control_transport,
)


class ResolveControlTransportTests(unittest.TestCase):
    def test_always_mavlink(self) -> None:
        self.assertEqual(resolve_control_transport({}), "mavlink")
        self.assertEqual(
            resolve_control_transport({"control": {"transport": "legacy"}}),
            "mavlink",
        )


class MavlinkEndpointConfigTests(unittest.TestCase):
    def test_default_endpoint(self) -> None:
        self.assertEqual(
            mavlink_endpoint_from_config({}),
            "udpin:0.0.0.0:14550",
        )

    def test_bridge_profile_ports_in_candidates(self) -> None:
        config = {
            "control": {
                "mavlink": {
                    "bridge_profile": {"qgc_port": 14555},
                    "endpoint": "udpin:0.0.0.0:14555",
                }
            }
        }
        endpoints = candidate_mavlink_endpoints(config)
        self.assertIn("udpin:0.0.0.0:14555", endpoints)


if __name__ == "__main__":
    unittest.main()
