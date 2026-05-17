import unittest

from src.control.mavlink_timesync import (
    TimesyncEvent,
    TimesyncStore,
    parse_timesync_message,
)


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

    def test_rejects_missing_required_fields(self) -> None:
        class _MissingTs1Message:
            tc1 = 0

            def get_type(self) -> str:
                return "TIMESYNC"

        with self.assertRaisesRegex(ValueError, "ts1"):
            parse_timesync_message(_MissingTs1Message())


class TimesyncStoreTests(unittest.TestCase):
    def _record_matched_response(
        self,
        store: TimesyncStore,
        *,
        ts1: int,
        sent_monotonic_ns: int,
        sent_wall_ns: int,
        rtt_monotonic_ns: int,
        rtt_wall_ns: int,
        offset_ns: int,
        target_system: int = 255,
        target_component: int = 1,
    ) -> None:
        store.record_outbound_request(
            ts1=ts1,
            sent_monotonic_ns=sent_monotonic_ns,
            sent_wall_ns=sent_wall_ns,
        )
        received_monotonic_ns = sent_monotonic_ns + rtt_monotonic_ns
        received_wall_ns = sent_wall_ns + rtt_wall_ns
        tc1 = ((sent_wall_ns + received_wall_ns) // 2) + offset_ns
        store.handle_message(
            _FakeTimesyncMessage(
                tc1=tc1,
                ts1=ts1,
                target_system=target_system,
                target_component=target_component,
            ),
            received_monotonic_ns=received_monotonic_ns,
            received_wall_ns=received_wall_ns,
        )

    def test_store_starts_empty(self) -> None:
        snapshot = TimesyncStore().snapshot()

        self.assertEqual(snapshot.message_count, 0)
        self.assertEqual(snapshot.outbound_request_count, 0)
        self.assertEqual(snapshot.matched_response_count, 0)
        self.assertIsNone(snapshot.last_message)
        self.assertIsNone(snapshot.last_request)
        self.assertIsNone(snapshot.last_response)
        self.assertIsNone(snapshot.last_measurement)
        self.assertIsNone(snapshot.best_measurement)
        self.assertEqual(snapshot.sync_health.status, "unsynced")
        self.assertIsNone(snapshot.stable_offset_ns)
        self.assertIsNone(snapshot.stable_rtt_ns)

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

    def test_response_matching_request_computes_measurement(self) -> None:
        store = TimesyncStore(local_system=255, local_component=1)
        request = store.record_outbound_request(
            ts1=2_000,
            sent_monotonic_ns=100,
            sent_wall_ns=2_000,
        )
        response = _FakeTimesyncMessage(
            tc1=3_500,
            ts1=request.ts1,
            target_system=255,
            target_component=1,
        )

        store.handle_message(
            response,
            received_monotonic_ns=180,
            received_wall_ns=2_120,
        )
        snapshot = store.snapshot()

        self.assertEqual(snapshot.outbound_request_count, 1)
        self.assertEqual(snapshot.matched_response_count, 1)
        self.assertEqual(snapshot.pending_request_count, 0)
        self.assertIsNotNone(snapshot.last_measurement)
        assert snapshot.last_measurement is not None
        self.assertEqual(snapshot.last_measurement.rtt_monotonic_ns, 80)
        self.assertEqual(snapshot.last_measurement.rtt_wall_ns, 120)
        self.assertEqual(snapshot.last_measurement.offset_ns, 1_440)
        self.assertEqual(snapshot.estimated_offset_ns, 1_440)
        self.assertEqual(snapshot.estimated_rtt_ns, 120)

    def test_best_measurement_prefers_lower_rtt(self) -> None:
        store = TimesyncStore(local_system=255, local_component=1)
        store.record_outbound_request(ts1=1_000, sent_monotonic_ns=100, sent_wall_ns=1_000)
        store.handle_message(
            _FakeTimesyncMessage(tc1=1_400, ts1=1_000, target_system=255, target_component=1),
            received_monotonic_ns=220,
            received_wall_ns=1_150,
        )
        store.record_outbound_request(ts1=2_000, sent_monotonic_ns=300, sent_wall_ns=2_000)
        store.handle_message(
            _FakeTimesyncMessage(tc1=2_470, ts1=2_000, target_system=255, target_component=1),
            received_monotonic_ns=360,
            received_wall_ns=2_060,
        )
        snapshot = store.snapshot()

        assert snapshot.best_measurement is not None
        self.assertEqual(snapshot.best_measurement.ts1, 2_000)
        self.assertEqual(snapshot.estimated_rtt_ns, 60)
        self.assertEqual(snapshot.estimated_offset_ns, 440)

    def test_legacy_broadcast_response_is_marked(self) -> None:
        store = TimesyncStore(local_system=255, local_component=1)
        store.record_outbound_request(ts1=5_000, sent_monotonic_ns=100, sent_wall_ns=5_000)
        store.handle_message(
            _FakeTimesyncMessage(tc1=5_060, ts1=5_000, target_system=0, target_component=0),
            received_monotonic_ns=130,
            received_wall_ns=5_030,
        )
        snapshot = store.snapshot()

        self.assertTrue(snapshot.remote_supports_legacy_broadcast)
        assert snapshot.last_measurement is not None
        self.assertTrue(snapshot.last_measurement.legacy_broadcast_response)

    def test_unmatched_response_does_not_create_measurement(self) -> None:
        store = TimesyncStore(local_system=255, local_component=1)
        store.record_outbound_request(ts1=7_000, sent_monotonic_ns=100, sent_wall_ns=7_000)
        store.handle_message(
            _FakeTimesyncMessage(tc1=7_100, ts1=9_999, target_system=255, target_component=1),
            received_monotonic_ns=140,
            received_wall_ns=7_040,
        )
        snapshot = store.snapshot()

        self.assertEqual(snapshot.matched_response_count, 0)
        self.assertEqual(snapshot.pending_request_count, 1)
        self.assertIsNone(snapshot.last_measurement)

    def test_record_outbound_request_sets_last_outbound_request(self) -> None:
        store = TimesyncStore()
        request = store.record_outbound_request(ts1=123, sent_monotonic_ns=1, sent_wall_ns=123)
        snapshot = store.snapshot()

        self.assertEqual(snapshot.last_outbound_request, request)
        self.assertEqual(snapshot.outbound_request_count, 1)

    def test_sync_health_warms_up_then_becomes_stable(self) -> None:
        store = TimesyncStore(
            local_system=255,
            local_component=1,
            min_stable_samples=3,
            stable_window_size=5,
            stable_best_subset_size=3,
        )

        self._record_matched_response(
            store,
            ts1=1_000,
            sent_monotonic_ns=100,
            sent_wall_ns=1_000,
            rtt_monotonic_ns=40,
            rtt_wall_ns=40,
            offset_ns=100,
        )
        first = store.snapshot()
        self.assertEqual(first.sync_health.status, "warming_up")
        self.assertEqual(first.sync_health.stable_sample_count, 1)

        self._record_matched_response(
            store,
            ts1=2_000,
            sent_monotonic_ns=200,
            sent_wall_ns=2_000,
            rtt_monotonic_ns=44,
            rtt_wall_ns=44,
            offset_ns=102,
        )
        self._record_matched_response(
            store,
            ts1=3_000,
            sent_monotonic_ns=300,
            sent_wall_ns=3_000,
            rtt_monotonic_ns=48,
            rtt_wall_ns=48,
            offset_ns=101,
        )
        snapshot = store.snapshot()

        self.assertEqual(snapshot.sync_health.status, "stable")
        self.assertEqual(snapshot.sync_health.sample_count, 3)
        self.assertEqual(snapshot.sync_health.stable_sample_count, 3)
        self.assertEqual(snapshot.stable_offset_ns, 101)
        self.assertEqual(snapshot.stable_rtt_ns, 44)
        self.assertEqual(snapshot.offset_jitter_ns, 2)

    def test_stable_estimate_prefers_lowest_rtt_subset(self) -> None:
        store = TimesyncStore(
            local_system=255,
            local_component=1,
            min_stable_samples=2,
            stable_window_size=5,
            stable_best_subset_size=2,
        )

        self._record_matched_response(
            store,
            ts1=10_000,
            sent_monotonic_ns=100,
            sent_wall_ns=10_000,
            rtt_monotonic_ns=200,
            rtt_wall_ns=200,
            offset_ns=1_000,
        )
        self._record_matched_response(
            store,
            ts1=11_000,
            sent_monotonic_ns=200,
            sent_wall_ns=11_000,
            rtt_monotonic_ns=20,
            rtt_wall_ns=20,
            offset_ns=100,
        )
        self._record_matched_response(
            store,
            ts1=12_000,
            sent_monotonic_ns=300,
            sent_wall_ns=12_000,
            rtt_monotonic_ns=30,
            rtt_wall_ns=30,
            offset_ns=200,
        )
        snapshot = store.snapshot()

        self.assertEqual(snapshot.stable_offset_ns, 150)
        self.assertEqual(snapshot.stable_rtt_ns, 25)
        self.assertEqual(snapshot.sync_health.status, "stable")

    def test_large_offset_jitter_marks_health_degraded(self) -> None:
        store = TimesyncStore(
            local_system=255,
            local_component=1,
            min_stable_samples=3,
            stable_window_size=5,
            stable_best_subset_size=3,
            max_offset_jitter_ns=5,
        )

        for index, offset_ns in enumerate((10, 30, 50), start=1):
            self._record_matched_response(
                store,
                ts1=20_000 + index,
                sent_monotonic_ns=100 * index,
                sent_wall_ns=20_000 + (100 * index),
                rtt_monotonic_ns=40,
                rtt_wall_ns=40,
                offset_ns=offset_ns,
            )
        snapshot = store.snapshot()

        self.assertEqual(snapshot.sync_health.status, "degraded")
        self.assertIn("offset jitter too high", snapshot.sync_health.reason)
        self.assertEqual(snapshot.stable_offset_ns, 30)
        self.assertEqual(snapshot.offset_jitter_ns, 40)


if __name__ == "__main__":
    unittest.main()
