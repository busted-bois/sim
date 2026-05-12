import unittest

from src.control.mavlink_timesync import TimesyncEvent, TimesyncStore, parse_timesync_message


class _FakeTimesyncMessage:
    def __init__(
        self,
        *,
        tc1: int,
        ts1: int,
        target_system: int = 0,
        target_component: int = 0,
        source_system: int = 1,
        source_component: int = 1,
    ) -> None:
        self.tc1 = tc1
        self.ts1 = ts1
        self.target_system = target_system
        self.target_component = target_component
        self._source_system = source_system
        self._source_component = source_component

    def get_type(self) -> str:
        return "TIMESYNC"

    def get_srcSystem(self) -> int:
        return self._source_system

    def get_srcComponent(self) -> int:
        return self._source_component


class _FakeHeartbeatMessage:
    def get_type(self) -> str:
        return "HEARTBEAT"


class ParseTimesyncMessageTests(unittest.TestCase):
    def test_parses_request_fields(self) -> None:
        message = _FakeTimesyncMessage(tc1=0, ts1=1234, source_system=7, source_component=9)

        event = parse_timesync_message(
            message,
            received_monotonic_ns=10,
            received_wall_ns=20,
        )

        self.assertTrue(event.is_request)
        self.assertFalse(event.is_response)
        self.assertEqual(event.tc1, 0)
        self.assertEqual(event.ts1, 1234)
        self.assertEqual(event.target_system, 0)
        self.assertEqual(event.target_component, 0)
        self.assertEqual(event.source_system, 7)
        self.assertEqual(event.source_component, 9)
        self.assertEqual(event.received_monotonic_ns, 10)
        self.assertEqual(event.received_wall_ns, 20)

    def test_rejects_non_timesync_message(self) -> None:
        with self.assertRaisesRegex(ValueError, "TIMESYNC"):
            parse_timesync_message(_FakeHeartbeatMessage())


class TimesyncStoreTests(unittest.TestCase):
    def test_store_tracks_request_and_response_separately(self) -> None:
        store = TimesyncStore()
        request = _FakeTimesyncMessage(tc1=0, ts1=111)
        response = _FakeTimesyncMessage(
            tc1=222,
            ts1=111,
            target_system=255,
            target_component=1,
            source_system=42,
            source_component=24,
        )

        request_event = store.handle_message(
            request,
            received_monotonic_ns=100,
            received_wall_ns=200,
        )
        response_event = store.handle_message(
            response,
            received_monotonic_ns=300,
            received_wall_ns=400,
        )
        snapshot = store.snapshot()

        self.assertEqual(snapshot.message_count, 2)
        self.assertEqual(snapshot.last_message, response_event)
        self.assertEqual(snapshot.last_request, request_event)
        self.assertEqual(snapshot.last_response, response_event)
        self.assertTrue(snapshot.last_response.is_response)
        self.assertEqual(snapshot.last_response.target_system, 255)
        self.assertEqual(snapshot.last_response.target_component, 1)
        self.assertEqual(snapshot.last_response.source_system, 42)
        self.assertEqual(snapshot.last_response.source_component, 24)

    def test_record_event_allows_reusing_parsed_timesync(self) -> None:
        store = TimesyncStore()
        event = TimesyncEvent(
            tc1=0,
            ts1=999,
            target_system=1,
            target_component=2,
            source_system=3,
            source_component=4,
            received_monotonic_ns=5,
            received_wall_ns=6,
        )

        store.record_event(event)
        snapshot = store.snapshot()

        self.assertEqual(snapshot.message_count, 1)
        self.assertEqual(snapshot.last_message, event)
        self.assertEqual(snapshot.last_request, event)
        self.assertIsNone(snapshot.last_response)

    def test_store_defaults_missing_target_ids_to_zero(self) -> None:
        class _MinimalTimesyncMessage:
            tc1 = 333
            ts1 = 444

            def get_type(self) -> str:
                return "TIMESYNC"

        snapshot = TimesyncStore()
        event = snapshot.handle_message(_MinimalTimesyncMessage())

        self.assertEqual(event.target_system, 0)
        self.assertEqual(event.target_component, 0)
        self.assertIsNone(event.source_system)
        self.assertIsNone(event.source_component)


if __name__ == "__main__":
    unittest.main()
