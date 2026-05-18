import unittest

from src.mavlink.attitude_bridge import AttitudeTelemetryBridge
from src.mavlink.attitude_store import AttitudeStore
from src.mavlink.messages import MAVLINK_MSG_ID_ATTITUDE
from tests.mavlink_fakes import FakeMav, FakeMessage


class AttitudeBridgeTests(unittest.TestCase):
    def test_on_message_stores_attitude(self) -> None:
        store = AttitudeStore()
        bridge = AttitudeTelemetryBridge(store, enabled=True, log_messages=False)
        bridge.on_message(
            FakeMessage(
                "ATTITUDE",
                time_boot_ms=42,
                roll=0.1,
                pitch=-0.2,
                yaw=0.3,
                rollspeed=0.01,
                pitchspeed=0.02,
                yawspeed=0.03,
                source_system=1,
                source_component=1,
            )
        )
        sample = store.get()
        assert sample is not None
        self.assertEqual(sample.time_boot_ms, 42)
        self.assertAlmostEqual(sample.roll, 0.1)
        self.assertAlmostEqual(sample.pitch, -0.2)

    def test_request_interval_when_enabled(self) -> None:
        store = AttitudeStore()
        bridge = AttitudeTelemetryBridge(store, enabled=True, request_hz=50.0)
        mav = FakeMav()
        bridge.request_interval(mav)
        self.assertEqual(len(mav.message_interval_calls), 1)
        msg_id, interval_us = mav.message_interval_calls[0]
        self.assertEqual(msg_id, MAVLINK_MSG_ID_ATTITUDE)
        self.assertEqual(interval_us, 20_000)

    def test_request_interval_skipped_when_disabled(self) -> None:
        store = AttitudeStore()
        bridge = AttitudeTelemetryBridge(store, enabled=False)
        mav = FakeMav()
        bridge.request_interval(mav)
        self.assertEqual(mav.message_interval_calls, [])

    def test_ignores_non_attitude_messages(self) -> None:
        store = AttitudeStore()
        bridge = AttitudeTelemetryBridge(store)
        bridge.on_message(FakeMessage("HEARTBEAT", base_mode=0))
        self.assertIsNone(store.get())


if __name__ == "__main__":
    unittest.main()
