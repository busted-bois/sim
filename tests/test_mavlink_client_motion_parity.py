import unittest

from pymavlink import mavutil

from src.control.mavlink_client import (
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PymavlinkFlightClient,
)
from tests.mavlink_fakes import FakeMavConnection, FakeMessage, fake_mavlink_monotonic_sleep


def _client(connection: FakeMavConnection) -> PymavlinkFlightClient:
    return PymavlinkFlightClient(
        endpoint="udpin:0.0.0.0:14550",
        guided_custom_mode=PX4_CUSTOM_MAIN_MODE_AUTO,
        send_timesync_requests=False,
        prepare_for_flight_on_connect=False,
        request_state_messages_on_connect=False,
        highres_imu_enabled=False,
        command_rate_hz=50.0,
        connection_factory=lambda *a, **k: connection,
    )


class PymavlinkFlightClientMotionParityTests(unittest.TestCase):
    def test_velocity_stream_sends_ned_velocity_components(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=3, source_component=1)
        connection = FakeMavConnection(heartbeat, [])
        client = _client(connection)
        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep():
                client.moveByVelocityAsync(1.25, -0.5, 0.25, 0.1).join()
        finally:
            client.close()

        self.assertTrue(connection.mav.position_target_calls)
        call = connection.mav.position_target_calls[0]
        self.assertEqual(call[3], mavutil.mavlink.MAV_FRAME_LOCAL_NED)
        self.assertAlmostEqual(call[8], 1.25, places=5)
        self.assertAlmostEqual(call[9], -0.5, places=5)
        self.assertAlmostEqual(call[10], 0.25, places=5)

    def test_attitude_roll_matches_airsim_mapped_euler(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=3, source_component=1)
        connection = FakeMavConnection(heartbeat, [])
        client = _client(connection)
        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep():
                client.moveByRollPitchYawThrottleAsync(0.08, 0.05, -0.03, 0.6, 0.08).join()
        finally:
            client.close()

        raw = PymavlinkFlightClient._quaternion_from_euler(0.08, -0.05, 0.03)
        expected = PymavlinkFlightClient._normalize_quaternion(raw)
        quat = connection.mav.attitude_target_calls[0][4]
        for a, b in zip(quat, expected, strict=True):
            self.assertAlmostEqual(a, b, places=5)

    def test_offboard_velocity_primes_setpoints_before_mode_command(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=3, source_component=1)
        connection = FakeMavConnection(heartbeat, [])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            guided_custom_mode=PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            request_state_messages_on_connect=False,
            highres_imu_enabled=False,
            command_rate_hz=50.0,
            connection_factory=lambda *a, **k: connection,
        )
        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep():
                client.moveByVelocityAsync(0.5, 0.0, 0.0, 0.1).join()
        finally:
            client.close()

        mode_calls = [
            call
            for call in connection.mav.command_long_calls
            if call[2] == mavutil.mavlink.MAV_CMD_DO_SET_MODE
        ]
        self.assertTrue(mode_calls)
        self.assertEqual(mode_calls[0][5], PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        self.assertGreaterEqual(len(connection.mav.position_target_calls), 10)
        for call in connection.mav.position_target_calls[:10]:
            self.assertAlmostEqual(call[8], 0.0, places=5)
            self.assertAlmostEqual(call[9], 0.0, places=5)
            self.assertAlmostEqual(call[10], 0.0, places=5)
