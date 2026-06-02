import math
import unittest

from pymavlink import mavutil

from src.control.flight_client import build_attitude_only_type_mask, build_body_rate_type_mask
from src.control.mavlink_client import PX4_CUSTOM_MAIN_MODE_AUTO, PymavlinkFlightClient
from tests.mavlink_fakes import FakeMavConnection, FakeMessage, fake_mavlink_monotonic_sleep


class PymavlinkFlightClientSetAttitudeTargetTests(unittest.TestCase):
    def _client(self, connection: FakeMavConnection, **kwargs: object) -> PymavlinkFlightClient:
        defaults: dict[str, object] = {
            "endpoint": "udpin:0.0.0.0:14550",
            "send_timesync_requests": False,
            "prepare_for_flight_on_connect": False,
            "request_state_messages_on_connect": False,
            "highres_imu_enabled": False,
            "command_rate_hz": 50.0,
            "connection_factory": lambda *a, **k: connection,
        }
        defaults.update(kwargs)
        return PymavlinkFlightClient(**defaults)  # type: ignore[arg-type]

    def test_roll_pitch_yaw_stream_sends_guided_then_attitude_with_expected_mask(
        self,
    ) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=7, source_component=3)
        connection = FakeMavConnection(heartbeat, [])
        client = self._client(connection)
        try:
            client.confirmConnection()
            raw = PymavlinkFlightClient._quaternion_from_euler(0.11, -0.22, -0.33)
            expected_quat = PymavlinkFlightClient._normalize_quaternion(raw)
            with fake_mavlink_monotonic_sleep():
                client.moveByRollPitchYawThrottleAsync(0.11, 0.22, 0.33, 0.72, 0.12).join()
        finally:
            client.close()

        self.assertTrue(connection.mav.command_long_calls)
        self.assertEqual(
            connection.mav.command_long_calls[0][2],
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        )
        self.assertTrue(connection.mav.attitude_target_calls)
        call = connection.mav.attitude_target_calls[0]
        self.assertEqual(call[3], build_attitude_only_type_mask())
        quat = call[4]
        self.assertEqual(len(quat), 4)
        for a, b in zip(quat, expected_quat, strict=True):
            self.assertAlmostEqual(a, b, places=6)
        self.assertAlmostEqual(call[8], 0.72, places=6)

    def test_throttle_clamped_to_unit_interval(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=7, source_component=3)
        connection = FakeMavConnection(heartbeat, [])
        client = self._client(connection)
        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep():
                client.moveByRollPitchYawThrottleAsync(0.0, 0.0, 0.0, 2.0, 0.08).join()
        finally:
            client.close()

        thrusts = [c[8] for c in connection.mav.attitude_target_calls]
        self.assertTrue(thrusts)
        for t in thrusts:
            self.assertGreaterEqual(t, 0.0)
            self.assertLessEqual(t, 1.0)
        self.assertAlmostEqual(thrusts[0], 1.0, places=6)

    def test_angle_rate_stream_uses_attitude_ignore_mask_and_negates_pitch_yaw_rates(
        self,
    ) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=7, source_component=3)
        connection = FakeMavConnection(heartbeat, [])
        client = self._client(connection)
        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep():
                client.moveByAngleRateThrottleAsync(0.5, 0.25, -0.125, 0.4, 0.1).join()
        finally:
            client.close()

        self.assertTrue(connection.mav.attitude_target_calls)
        call = connection.mav.attitude_target_calls[0]
        self.assertEqual(call[3], build_body_rate_type_mask())
        self.assertEqual(list(call[4]), [1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(call[5], 0.5, places=6)
        self.assertAlmostEqual(call[6], -0.25, places=6)
        self.assertAlmostEqual(call[7], 0.125, places=6)
        self.assertAlmostEqual(call[8], 0.4, places=6)

    def test_rotate_by_yaw_rate_deg_per_second_converted_to_rad_s(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=7, source_component=3)
        connection = FakeMavConnection(heartbeat, [])
        client = self._client(connection)
        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep():
                client.rotateByYawRateAsync(30.0, 0.09).join()
        finally:
            client.close()

        self.assertTrue(connection.mav.attitude_target_calls)
        call = connection.mav.attitude_target_calls[0]
        self.assertAlmostEqual(call[7], math.radians(30.0), places=6)

    def test_normalize_quaternion_zero_is_identity(self) -> None:
        q = PymavlinkFlightClient._normalize_quaternion((0.0, 0.0, 0.0, 0.0))
        self.assertEqual(q, (1.0, 0.0, 0.0, 0.0))

    def test_attitude_target_throttle_body_z_sets_mask_bit(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=7, source_component=3)
        connection = FakeMavConnection(heartbeat, [])
        client = self._client(
            connection,
            attitude_target_throttle_body_z=True,
        )
        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep():
                client.moveByRollPitchYawThrottleAsync(0.01, 0.0, 0.0, 0.5, 0.06).join()
        finally:
            client.close()

        self.assertEqual(
            connection.mav.attitude_target_calls[0][3],
            build_attitude_only_type_mask(throttle_body_z=True),
        )

    def test_rate_target_throttle_body_z_sets_mask_bit(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=7, source_component=3)
        connection = FakeMavConnection(heartbeat, [])
        client = self._client(connection, attitude_target_throttle_body_z=True)
        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep():
                client.moveByAngleRateThrottleAsync(0.1, 0.0, 0.0, 0.5, 0.06).join()
        finally:
            client.close()

        self.assertEqual(
            connection.mav.attitude_target_calls[0][3],
            build_body_rate_type_mask(throttle_body_z=True),
        )

    def test_guided_mode_throttled_between_streams(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=7, source_component=3)
        connection = FakeMavConnection(heartbeat, [])
        client = self._client(connection, guided_custom_mode=PX4_CUSTOM_MAIN_MODE_AUTO)
        mode = mavutil.mavlink.MAV_CMD_DO_SET_MODE

        def guided_count() -> int:
            return sum(1 for c in connection.mav.command_long_calls if c[2] == mode)

        try:
            client.confirmConnection()
            with fake_mavlink_monotonic_sleep() as clock:
                client.moveByRollPitchYawThrottleAsync(0.02, 0.0, 0.0, 0.55, 0.06).join()
                n1 = guided_count()
                self.assertGreaterEqual(n1, 1)
                client.moveByRollPitchYawThrottleAsync(0.02, 0.0, 0.0, 0.55, 0.06).join()
                self.assertEqual(guided_count(), n1)
                clock["t"] += 1.5
                client.moveByRollPitchYawThrottleAsync(0.02, 0.0, 0.0, 0.55, 0.06).join()
                self.assertEqual(guided_count(), n1 + 1)
        finally:
            client.close()
