import time
import unittest

from pymavlink import mavutil

from src.control.mavlink_client import PymavlinkFlightClient
from src.control.utils import _yaw_from_orientation
from tests.mavlink_fakes import FakeMavConnection, FakeMessage


class PymavlinkNedEnvironmentTests(unittest.TestCase):
    def test_confirm_connection_requests_attitude_interval(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        connection = FakeMavConnection(heartbeat, [])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            request_state_messages_on_connect=True,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
        finally:
            client.close()

        requested_ids = {int(call[0]) for call in connection.mav.message_interval_calls}
        self.assertIn(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, requested_ids)

    def test_telemetry_updates_ned_environment_and_orientation(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        local = FakeMessage(
            "LOCAL_POSITION_NED",
            x=5.0,
            y=-2.0,
            z=-4.0,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            time_boot_ms=1000,
        )
        attitude = FakeMessage(
            "ATTITUDE",
            roll=0.0,
            pitch=0.0,
            yaw=0.5,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
            time_boot_ms=1000,
        )
        connection = FakeMavConnection(heartbeat, [local, attitude])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            deadline = time.monotonic() + 2.0
            snap = client.get_ned_environment().snapshot()
            while not snap.has_attitude and time.monotonic() < deadline:
                time.sleep(0.02)
                snap = client.get_ned_environment().snapshot()

            self.assertTrue(snap.has_position)
            self.assertAlmostEqual(snap.position.x, 5.0, places=4)
            self.assertAlmostEqual(snap.position.y, -2.0, places=4)
            self.assertTrue(snap.has_attitude)
            assert snap.attitude is not None
            self.assertAlmostEqual(snap.attitude.yaw, 0.5, places=4)

            state = client.getMultirotorState().kinematics_estimated
            yaw = _yaw_from_orientation(state.orientation)
            self.assertAlmostEqual(yaw, 0.5, places=4)
        finally:
            client.close()


if __name__ == "__main__":
    unittest.main()
