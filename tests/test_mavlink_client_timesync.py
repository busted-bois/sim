import time
import unittest

from src.control.mavlink_client import PymavlinkFlightClient
from tests.mavlink_fakes import FakeMavConnection, FakeMessage


class PymavlinkFlightClientTimesyncTests(unittest.TestCase):
    def test_sync_probe_mode_skips_flight_setup_commands(self) -> None:
        heartbeat = FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            timesync_log_messages=False,
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            request_state_messages_on_connect=False,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
        finally:
            client.close()

        self.assertEqual(connection.mav.command_long_calls, [])
        self.assertEqual(connection.mav.message_interval_calls, [])

    def test_confirm_connection_processes_timesync_request_and_responds(self) -> None:
        heartbeat = FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        timesync_request = FakeMessage(
            "TIMESYNC",
            tc1=0,
            ts1=123456789,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [timesync_request])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            timesync_log_messages=False,
            send_timesync_requests=False,
            timesync_min_stable_samples=1,
            timesync_max_stable_rtt_ns=10_000_000_000_000,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            deadline = time.time() + 1.0
            while time.time() < deadline:
                snapshot = client.getTimesyncSnapshot()
                if snapshot.message_count >= 1:
                    break
                time.sleep(0.01)
            snapshot = client.getTimesyncSnapshot()

            self.assertEqual(snapshot.message_count, 1)
            self.assertIsNotNone(snapshot.last_request)
            self.assertIsNone(snapshot.last_response)
            assert snapshot.last_request is not None
            self.assertEqual(snapshot.last_request.tc1, 0)
            self.assertEqual(snapshot.last_request.ts1, 123456789)
            self.assertEqual(len(connection.mav.timesync_calls), 1)
            tc1, ts1 = connection.mav.timesync_calls[0]
            self.assertNotEqual(tc1, 0)
            self.assertEqual(ts1, 123456789)
        finally:
            client.close()
        self.assertTrue(connection.closed)

    def test_timesync_response_is_stored_without_reply(self) -> None:
        heartbeat = FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        timesync_response = FakeMessage(
            "TIMESYNC",
            tc1=987654321,
            ts1=222,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [timesync_response])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            timesync_log_messages=False,
            send_timesync_requests=False,
            timesync_min_stable_samples=1,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            deadline = time.time() + 1.0
            while time.time() < deadline:
                snapshot = client.getTimesyncSnapshot()
                if snapshot.message_count >= 1:
                    break
                time.sleep(0.01)
            snapshot = client.getTimesyncSnapshot()
        finally:
            client.close()

        self.assertEqual(snapshot.message_count, 1)
        self.assertIsNotNone(snapshot.last_response)
        self.assertEqual(connection.mav.timesync_calls, [])

    def test_client_sends_periodic_timesync_requests(self) -> None:
        heartbeat = FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            timesync_log_messages=False,
            send_timesync_requests=True,
            timesync_request_interval_s=0.1,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            deadline = time.time() + 0.5
            while time.time() < deadline:
                if connection.mav.timesync_calls:
                    break
                time.sleep(0.01)
            snapshot = client.getTimesyncSnapshot()
        finally:
            client.close()

        self.assertGreaterEqual(len(connection.mav.timesync_calls), 1)
        first_tc1, first_ts1 = connection.mav.timesync_calls[0]
        self.assertEqual(first_tc1, 0)
        self.assertGreater(first_ts1, 0)
        self.assertGreaterEqual(snapshot.outbound_request_count, 1)

    def test_response_to_outbound_request_produces_measurement(self) -> None:
        heartbeat = FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            timesync_log_messages=False,
            send_timesync_requests=False,
            timesync_min_stable_samples=1,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            request_sent_monotonic_ns = time.monotonic_ns()
            request_sent_wall_ns = time.time_ns()
            client._record_outbound_timesync_request = self._fixed_outbound_request_factory(  # type: ignore[method-assign]
                client,
                ts1=request_sent_wall_ns,
                sent_monotonic_ns=request_sent_monotonic_ns,
                sent_wall_ns=request_sent_wall_ns,
            )
            client._send_timesync_request()
            connection.push_message(
                FakeMessage(
                    "TIMESYNC",
                    tc1=request_sent_wall_ns + 500_000,
                    ts1=request_sent_wall_ns,
                    target_system=255,
                    target_component=1,
                    source_system=42,
                    source_component=24,
                )
            )
            deadline = time.time() + 1.0
            while time.time() < deadline:
                snapshot = client.getTimesyncSnapshot()
                if snapshot.matched_response_count >= 1:
                    break
                time.sleep(0.01)
            snapshot = client.getTimesyncSnapshot()
        finally:
            client.close()

        self.assertEqual(snapshot.matched_response_count, 1)
        self.assertIsNotNone(snapshot.last_measurement)
        assert snapshot.last_measurement is not None
        self.assertGreaterEqual(snapshot.last_measurement.rtt_monotonic_ns, 0)
        self.assertEqual(snapshot.sync_health.status, "stable")
        self.assertIsNotNone(snapshot.stable_offset_ns)
        self.assertIsNotNone(snapshot.stable_rtt_ns)

    @staticmethod
    def _fixed_outbound_request_factory(client, **kwargs):
        def _record():
            return client._timesync_store.record_outbound_request(**kwargs)

        return _record


if __name__ == "__main__":
    unittest.main()
