import unittest

from src.mavlink.attitude_bridge import AttitudeTelemetryBridge
from src.mavlink.integration import (
    attach_attitude_bridge,
    create_attitude_bridge,
    get_attitude_health,
    get_attitude_sample,
)


class _FakeClient:
    pass


class AttitudeIntegrationTests(unittest.TestCase):
    def test_attach_and_getters(self) -> None:
        client = _FakeClient()
        bridge = create_attitude_bridge({"enabled": True, "request_hz": 50.0})
        attach_attitude_bridge(client, bridge)
        self.assertIs(get_attitude_sample(client), None)
        health = get_attitude_health(client)
        assert health is not None
        self.assertEqual(health.status, "missing")

    def test_from_sim_config_factory(self) -> None:
        bridge = AttitudeTelemetryBridge.from_sim_config({})
        self.assertTrue(bridge.enabled)


if __name__ == "__main__":
    unittest.main()
