import queue
import time
import unittest

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


class PymavlinkFlightClientTimesyncTests(unittest.TestCase):
    def test_sync_probe_mode_skips_flight_setup_commands(self) -> None:
        heartbeat = _FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        connection = _FakeMavConnection(heartbeat, [])
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
        heartbeat = _FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        timesync_request = _FakeMessage(
            "TIMESYNC",
            tc1=0,
            ts1=123456789,
            source_system=42,
            source_component=24,
        )
        connection = _FakeMavConnection(heartbeat, [timesync_request])
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
        heartbeat = _FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        timesync_response = _FakeMessage(
            "TIMESYNC",
            tc1=987654321,
            ts1=222,
            source_system=42,
            source_component=24,
        )
        connection = _FakeMavConnection(heartbeat, [timesync_response])
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
        heartbeat = _FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        connection = _FakeMavConnection(heartbeat, [])
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
        heartbeat = _FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        connection = _FakeMavConnection(heartbeat, [])
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
                _FakeMessage(
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
