import time
import unittest

from pymavlink import mavutil

from src.control.highres_imu import (
    HIGHRES_IMU_UPDATED_XACC,
    HighresImuSample,
)
from src.control.mavlink_client import PymavlinkFlightClient
from tests.mavlink_fakes import FakeMavConnection, FakeMessage


class PymavlinkFlightClientHighresImuTests(unittest.TestCase):
    def test_confirm_connection_requests_highres_imu_interval(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        connection = FakeMavConnection(heartbeat, [])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            request_state_messages_on_connect=True,
            highres_imu_enabled=True,
            highres_imu_request_hz=40.0,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
        finally:
            client.close()

        requested_ids = {int(call[0]) for call in connection.mav.message_interval_calls}
        self.assertIn(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, requested_ids)
        self.assertIn(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, requested_ids)
        self.assertIn(mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, requested_ids)
        highres_call = next(
            call
            for call in connection.mav.message_interval_calls
            if int(call[0]) == mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU
        )
        self.assertEqual(int(highres_call[1]), 25_000)

    def test_highres_imu_message_is_stored_and_health_updated(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        highres = FakeMessage(
            "HIGHRES_IMU",
            time_usec=123456,
            xacc=1.1,
            yacc=2.2,
            zacc=3.3,
            xgyro=0.1,
            ygyro=0.2,
            zgyro=0.3,
            xmag=0.01,
            ymag=0.02,
            zmag=0.03,
            abs_pressure=1013.25,
            diff_pressure=0.0,
            pressure_alt=120.0,
            temperature=24.5,
            fields_updated=0,
            id=2,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [highres])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            highres_imu_enabled=True,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            sample = self._wait_for_sample(client)
            health = client.getHighresImuHealth()
        finally:
            client.close()

        assert sample is not None
        self.assertEqual(sample.sensor_id, 2)
        self.assertAlmostEqual(sample.xacc or 0.0, 1.1)
        self.assertAlmostEqual(sample.zgyro or 0.0, 0.3)
        self.assertEqual(sample.source_system, 42)
        self.assertEqual(sample.source_component, 24)
        self.assertIsNotNone(health)
        assert health is not None
        self.assertGreaterEqual(health.sample_count, 1)
        self.assertEqual(health.status, "ok")

    def test_partial_fields_updated_merge_preserves_prior_values(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        full = FakeMessage(
            "HIGHRES_IMU",
            time_usec=100,
            xacc=1.0,
            yacc=2.0,
            zacc=3.0,
            xgyro=4.0,
            ygyro=5.0,
            zgyro=6.0,
            xmag=7.0,
            ymag=8.0,
            zmag=9.0,
            abs_pressure=10.0,
            diff_pressure=11.0,
            pressure_alt=12.0,
            temperature=13.0,
            fields_updated=0,
            id=0,
            source_system=42,
            source_component=24,
        )
        partial = FakeMessage(
            "HIGHRES_IMU",
            time_usec=101,
            xacc=99.0,
            yacc=99.0,
            zacc=99.0,
            xgyro=99.0,
            ygyro=99.0,
            zgyro=99.0,
            xmag=99.0,
            ymag=99.0,
            zmag=99.0,
            abs_pressure=99.0,
            diff_pressure=99.0,
            pressure_alt=99.0,
            temperature=99.0,
            fields_updated=HIGHRES_IMU_UPDATED_XACC,
            id=0,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [full, partial])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            highres_imu_enabled=True,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            sample = self._wait_for_sample(client, min_count=2)
        finally:
            client.close()

        assert sample is not None
        self.assertEqual(sample.time_usec, 101)
        self.assertEqual(sample.xacc, 99.0)
        self.assertEqual(sample.yacc, 2.0)
        self.assertEqual(sample.zgyro, 6.0)
        self.assertEqual(sample.temperature, 13.0)

    def test_highres_imu_health_turns_stale_when_updates_stop(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        highres = FakeMessage(
            "HIGHRES_IMU",
            time_usec=123456,
            xacc=1.0,
            yacc=2.0,
            zacc=3.0,
            xgyro=0.1,
            ygyro=0.2,
            zgyro=0.3,
            xmag=0.0,
            ymag=0.0,
            zmag=0.0,
            abs_pressure=1013.25,
            diff_pressure=0.0,
            pressure_alt=120.0,
            temperature=24.5,
            fields_updated=0,
            id=0,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [highres])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            highres_imu_enabled=True,
            highres_imu_max_staleness_ms=5.0,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            self._wait_for_sample(client)
            time.sleep(0.03)
            health = client.getHighresImuHealth()
        finally:
            client.close()

        self.assertIsNotNone(health)
        assert health is not None
        self.assertEqual(health.status, "stale")
        self.assertGreater(health.update_age_ms or 0.0, 5.0)

    @staticmethod
    def _wait_for_sample(
        client: PymavlinkFlightClient,
        *,
        min_count: int = 1,
        timeout_s: float = 1.0,
    ) -> HighresImuSample | None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            health = client.getHighresImuHealth()
            if health is not None and health.sample_count >= min_count:
                return client.getHighresImu()
            time.sleep(0.01)
        return client.getHighresImu()


if __name__ == "__main__":
    unittest.main()
