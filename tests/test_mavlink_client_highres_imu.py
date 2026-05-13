import queue
import time
import unittest
from types import SimpleNamespace

from pymavlink import mavutil

from src.control.flight_client import AirSimAdapter
from src.control.highres_imu import (
    HIGHRES_IMU_UPDATED_XACC,
    HighresImuSample,
)
from src.control.mavlink_client import PymavlinkFlightClient


class _FakeMessage:
    def __init__(self, message_type: str, **fields) -> None:
        self._message_type = message_type
        self._source_system = int(fields.pop("source_system", 1))
        self._source_component = int(fields.pop("source_component", 1))
        for key, value in fields.items():
            setattr(self, key, value)

    def get_type(self) -> str:
        return self._message_type

    def get_srcSystem(self) -> int:
        return self._source_system

    def get_srcComponent(self) -> int:
        return self._source_component


class _FakeMavSender:
    def __init__(self) -> None:
        self.command_long_calls: list[tuple] = []
        self.message_interval_calls: list[tuple] = []
        self.position_target_calls: list[tuple] = []
        self.attitude_target_calls: list[tuple] = []
        self.timesync_calls: list[tuple[int, int]] = []

    def command_long_send(self, *args) -> None:
        self.command_long_calls.append(args)

    def message_interval_send(self, *args) -> None:
        self.message_interval_calls.append(args)

    def set_position_target_local_ned_send(self, *args) -> None:
        self.position_target_calls.append(args)

    def set_attitude_target_send(self, *args) -> None:
        self.attitude_target_calls.append(args)

    def timesync_send(self, tc1: int, ts1: int) -> None:
        self.timesync_calls.append((tc1, ts1))


class _FakeMavConnection:
    def __init__(self, heartbeat: _FakeMessage, queued_messages: list[_FakeMessage]) -> None:
        self._heartbeat = heartbeat
        self._queue: queue.Queue[_FakeMessage] = queue.Queue()
        for message in queued_messages:
            self.push_message(message)
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self.mav = _FakeMavSender()
        self.closed = False

    def wait_heartbeat(self, timeout: float | None = None):
        _ = timeout
        return self._heartbeat

    def recv_match(self, type=None, blocking=True, timeout=None):
        _ = blocking
        deadline = time.time() + (timeout or 0.0)
        while True:
            remaining = max(0.0, deadline - time.time()) if timeout is not None else None
            try:
                message = self._queue.get(timeout=remaining)
            except queue.Empty:
                return None
            if type is None or message.get_type() in type:
                return message

    def close(self) -> None:
        self.closed = True

    def push_message(self, message: _FakeMessage) -> None:
        self._queue.put(message)


class PymavlinkFlightClientHighresImuTests(unittest.TestCase):
    def test_confirm_connection_requests_highres_imu_interval(self) -> None:
        heartbeat = _FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        connection = _FakeMavConnection(heartbeat, [])
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
        heartbeat = _FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        highres = _FakeMessage(
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
        connection = _FakeMavConnection(heartbeat, [highres])
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
        heartbeat = _FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        full = _FakeMessage(
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
        partial = _FakeMessage(
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
        connection = _FakeMavConnection(heartbeat, [full, partial])
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
        heartbeat = _FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        highres = _FakeMessage(
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
        connection = _FakeMavConnection(heartbeat, [highres])
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


class AirSimAdapterHighresImuTests(unittest.TestCase):
    def test_adapter_maps_airsim_sensor_calls_to_highres_imu_shape(self) -> None:
        fake_client = SimpleNamespace(
            getImuData=lambda: SimpleNamespace(
                time_stamp=123,
                angular_velocity=SimpleNamespace(x_val=0.1, y_val=0.2, z_val=0.3),
                linear_acceleration=SimpleNamespace(x_val=1.1, y_val=1.2, z_val=1.3),
            ),
            getMagnetometerData=lambda: SimpleNamespace(
                magnetic_field_body=SimpleNamespace(x_val=0.01, y_val=0.02, z_val=0.03)
            ),
            getBarometerData=lambda: SimpleNamespace(
                pressure=1012.5,
                altitude=88.0,
                qnh=25.0,
            ),
        )
        adapter = AirSimAdapter(fake_client)

        sample = adapter.getHighresImu()
        health = adapter.getHighresImuHealth()

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.transport, "airsim")
        self.assertEqual(sample.time_usec, 123)
        self.assertAlmostEqual(sample.xacc or 0.0, 1.1)
        self.assertAlmostEqual(sample.zgyro or 0.0, 0.3)
        self.assertAlmostEqual(sample.abs_pressure or 0.0, 1012.5)
        self.assertIsNone(sample.temperature)
        self.assertIsNotNone(health)
        assert health is not None
        self.assertEqual(health.status, "ok")


if __name__ == "__main__":
    unittest.main()
