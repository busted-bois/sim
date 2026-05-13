import os
import queue
import unittest
from unittest.mock import patch

from src.control.highres_imu import HighresImuHealth, HighresImuSample, SensorSnapshot
from src.preflight import _mavlink_highres_imu
from src.sim_launch import main_highres_imu_smoke


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
        self.message_interval_calls: list[tuple] = []

    def message_interval_send(self, *args) -> None:
        self.message_interval_calls.append(args)


class _FakeMavConnection:
    def __init__(self, heartbeat: _FakeMessage, queued_messages: list[_FakeMessage]) -> None:
        self._heartbeat = heartbeat
        self._queue: queue.Queue[_FakeMessage] = queue.Queue()
        for message in queued_messages:
            self._queue.put(message)
        self.mav = _FakeMavSender()

    def wait_heartbeat(self, timeout: float | None = None):
        _ = timeout
        return self._heartbeat

    def recv_match(self, type=None, blocking=True, timeout=None):
        _ = type, blocking, timeout
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def close(self) -> None:
        return


class HighresImuFollowupTests(unittest.TestCase):
    def test_sample_age_and_snapshot_age_helpers(self) -> None:
        sample = HighresImuSample(
            time_usec=123,
            xacc=1.0,
            yacc=2.0,
            zacc=3.0,
            xgyro=0.1,
            ygyro=0.2,
            zgyro=0.3,
            xmag=0.0,
            ymag=0.0,
            zmag=0.0,
            abs_pressure=1000.0,
            diff_pressure=0.0,
            pressure_alt=50.0,
            temperature=20.0,
            fields_updated=0,
            sensor_id=7,
            source_system=1,
            source_component=1,
            local_received_monotonic_ns=1_000_000_000,
            transport="mavlink",
        )
        snapshot = SensorSnapshot(
            state="state",
            highres_imu=sample,
            highres_imu_health=HighresImuHealth(
                status="ok",
                reason="fresh",
                enabled=True,
                sample_count=1,
                stream_rate_hz=20.0,
                update_age_ms=5.0,
                max_staleness_ms=1000.0,
                sensor_count=1,
                active_sensor_ids=(7,),
                expected_rate_hz=20.0,
            ),
            captured_monotonic_ns=1_005_000_000,
            transport="mavlink",
        )

        self.assertAlmostEqual(sample.age_ms(1_010_000_000), 10.0)
        self.assertAlmostEqual(snapshot.imu_age_ms() or 0.0, 5.0)

    def test_preflight_highres_imu_probe_reports_sensor_id(self) -> None:
        config = {
            "control": {
                "transport": "mavlink",
                "mavlink": {
                    "endpoint": "udpin:0.0.0.0:14550",
                    "auto_discover_endpoints": False,
                    "highres_imu": {"request_hz": 20.0},
                }
            }
        }
        heartbeat = _FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        highres = _FakeMessage("HIGHRES_IMU", id=3, source_system=42, source_component=24)
        connection = _FakeMavConnection(heartbeat, [highres])

        ok, detail, endpoints = _mavlink_highres_imu(
            config,
            connection_factory=lambda *args, **kwargs: connection,
        )

        self.assertTrue(ok)
        self.assertIn("sensor_id=3", detail)
        self.assertEqual(endpoints, ["udpin:0.0.0.0:14550"])
        self.assertEqual(len(connection.mav.message_interval_calls), 1)

    def test_sim_highres_imu_smoke_launches_highres_script(self) -> None:
        old_transport = os.environ.get("AIGP_CONTROL_TRANSPORT")
        old_allow = os.environ.get("AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT")
        try:
            with patch("src.sim_launch.launch") as launch_mock:
                main_highres_imu_smoke()

            launch_mock.assert_called_once_with(script_path="src/highres_imu_smoke.py")
            self.assertEqual(os.environ.get("AIGP_CONTROL_TRANSPORT"), "mavlink")
            self.assertEqual(os.environ.get("AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT"), "1")
        finally:
            if old_transport is None:
                os.environ.pop("AIGP_CONTROL_TRANSPORT", None)
            else:
                os.environ["AIGP_CONTROL_TRANSPORT"] = old_transport
            if old_allow is None:
                os.environ.pop("AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT", None)
            else:
                os.environ["AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT"] = old_allow


if __name__ == "__main__":
    unittest.main()
