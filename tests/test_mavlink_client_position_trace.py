"""Integration tests for position trace wiring in PymavlinkFlightClient."""

from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path

from src.control.mavlink_client import PymavlinkFlightClient
from src.position_trace import PositionTraceStore
from tests.mavlink_fakes import FakeMavConnection, FakeMessage, fake_mavlink_monotonic_sleep


class PymavlinkFlightClientPositionTraceTests(unittest.TestCase):
    def test_local_position_ned_records_and_flushes_csv(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        local_position = FakeMessage(
            "LOCAL_POSITION_NED",
            time_boot_ms=1000,
            x=1.0,
            y=2.0,
            z=-5.0,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [local_position])
        with tempfile.TemporaryDirectory() as tmp:
            trace_path = Path(tmp) / "position_trace.csv"
            trace_store = PositionTraceStore(trace_path, max_speed_ms=20.0)
            client = PymavlinkFlightClient(
                endpoint="udpin:0.0.0.0:14550",
                send_timesync_requests=False,
                prepare_for_flight_on_connect=False,
                request_state_messages_on_connect=False,
                highres_imu_enabled=False,
                sim_config={},
                position_trace=trace_store,
                connection_factory=lambda *args, **kwargs: connection,
            )
            try:
                with fake_mavlink_monotonic_sleep():
                    client.confirmConnection()
                    self._wait_for_trace(client, min_count=1)
            finally:
                client.close()

            self.assertTrue(trace_path.is_file())
            snapshot = trace_store.snapshot()
            self.assertIsNotNone(snapshot.latest)
            assert snapshot.latest is not None
            self.assertAlmostEqual(snapshot.latest[2], 0.0)
            self.assertAlmostEqual(snapshot.latest[4], 0.0)

    @staticmethod
    def _wait_for_trace(
        client: PymavlinkFlightClient,
        *,
        min_count: int = 1,
        timeout_s: float = 1.0,
    ) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            snapshot = client.getPositionTraceSnapshot()
            if snapshot is not None and snapshot.health.accepted_count >= min_count:
                return
            time.sleep(0.01)
        snapshot = client.getPositionTraceSnapshot()
        if snapshot is None or snapshot.health.accepted_count < min_count:
            raise AssertionError("position trace samples not received in time")


if __name__ == "__main__":
    unittest.main()
