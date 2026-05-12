from __future__ import annotations

import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from statistics import median
from typing import Protocol

__all__ = [
    "TimesyncEvent",
    "TimesyncHealth",
    "TimesyncMeasurement",
    "TimesyncOutboundRequest",
    "TimesyncSnapshot",
    "TimesyncStore",
    "parse_timesync_message",
]

_TIMESYNC_MESSAGE_TYPE = "TIMESYNC"


class _TimesyncMessageLike(Protocol):
    tc1: object
    ts1: object

    def get_type(self) -> str: ...


@dataclass(frozen=True, slots=True)
class TimesyncEvent:
    tc1: int
    ts1: int
    target_system: int
    target_component: int
    source_system: int | None
    source_component: int | None
    received_monotonic_ns: int
    received_wall_ns: int

    @property
    def is_request(self) -> bool:
        return self.tc1 == 0

    @property
    def is_response(self) -> bool:
        return self.tc1 != 0


@dataclass(frozen=True, slots=True)
class TimesyncOutboundRequest:
    ts1: int
    target_system: int
    target_component: int
    sent_monotonic_ns: int
    sent_wall_ns: int


@dataclass(frozen=True, slots=True)
class TimesyncMeasurement:
    ts1: int
    remote_tc1: int
    request_sent_monotonic_ns: int
    response_received_monotonic_ns: int
    request_sent_wall_ns: int
    response_received_wall_ns: int
    rtt_monotonic_ns: int
    rtt_wall_ns: int
    offset_ns: int
    response_target_system: int
    response_target_component: int
    legacy_broadcast_response: bool


@dataclass(frozen=True, slots=True)
class TimesyncHealth:
    status: str
    sample_count: int
    stable_sample_count: int
    stable_offset_ns: int | None
    stable_rtt_ns: int | None
    offset_jitter_ns: int | None
    reason: str


@dataclass(frozen=True, slots=True)
class TimesyncSnapshot:
    message_count: int
    request_count: int
    response_count: int
    outbound_request_count: int
    matched_response_count: int
    pending_request_count: int
    last_message: TimesyncEvent | None
    last_request: TimesyncEvent | None
    last_response: TimesyncEvent | None
    last_outbound_request: TimesyncOutboundRequest | None
    last_measurement: TimesyncMeasurement | None
    best_measurement: TimesyncMeasurement | None
    estimated_offset_ns: int | None
    estimated_rtt_ns: int | None
    stable_offset_ns: int | None
    stable_rtt_ns: int | None
    offset_jitter_ns: int | None
    sync_health: TimesyncHealth
    remote_supports_legacy_broadcast: bool

    @classmethod
    def empty(cls) -> TimesyncSnapshot:
        health = TimesyncHealth(
            status="unsynced",
            sample_count=0,
            stable_sample_count=0,
            stable_offset_ns=None,
            stable_rtt_ns=None,
            offset_jitter_ns=None,
            reason="no matched TIMESYNC responses yet",
        )
        return cls(
            message_count=0,
            request_count=0,
            response_count=0,
            outbound_request_count=0,
            matched_response_count=0,
            pending_request_count=0,
            last_message=None,
            last_request=None,
            last_response=None,
            last_outbound_request=None,
            last_measurement=None,
            best_measurement=None,
            estimated_offset_ns=None,
            estimated_rtt_ns=None,
            stable_offset_ns=None,
            stable_rtt_ns=None,
            offset_jitter_ns=None,
            sync_health=health,
            remote_supports_legacy_broadcast=False,
        )


def parse_timesync_message(
    message: _TimesyncMessageLike | object,
    *,
    received_monotonic_ns: int | None = None,
    received_wall_ns: int | None = None,
) -> TimesyncEvent:
    message_type = getattr(message, "get_type", None)
    if callable(message_type) and message_type() != _TIMESYNC_MESSAGE_TYPE:
        raise ValueError(f"Expected a {_TIMESYNC_MESSAGE_TYPE} MAVLink message.")

    tc1 = int(_required_attr(message, "tc1"))
    ts1 = int(_required_attr(message, "ts1"))
    target_system = int(getattr(message, "target_system", 0))
    target_component = int(getattr(message, "target_component", 0))
    source_system = _optional_source_id(message, "get_srcSystem")
    source_component = _optional_source_id(message, "get_srcComponent")
    monotonic_ns, wall_ns = _receive_timestamps(
        received_monotonic_ns=received_monotonic_ns,
        received_wall_ns=received_wall_ns,
    )

    return TimesyncEvent(
        tc1=tc1,
        ts1=ts1,
        target_system=target_system,
        target_component=target_component,
        source_system=source_system,
        source_component=source_component,
        received_monotonic_ns=monotonic_ns,
        received_wall_ns=wall_ns,
    )


class TimesyncStore:
    """Thread-safe state for MAVLink TIMESYNC requests, replies, and sync estimates."""

    def __init__(
        self,
        *,
        local_system: int | None = None,
        local_component: int | None = None,
        pending_request_limit: int = 64,
        stable_window_size: int = 9,
        stable_best_subset_size: int = 5,
        min_stable_samples: int = 3,
        max_stable_rtt_ns: int = 250_000_000,
        max_offset_jitter_ns: int = 50_000_000,
    ) -> None:
        self._lock = threading.Lock()
        self._snapshot = TimesyncSnapshot.empty()
        self._local_system = local_system
        self._local_component = local_component
        self._pending_request_limit = max(1, int(pending_request_limit))
        self._stable_window_size = max(1, int(stable_window_size))
        self._stable_best_subset_size = max(1, int(stable_best_subset_size))
        self._min_stable_samples = max(1, int(min_stable_samples))
        self._max_stable_rtt_ns = max(1, int(max_stable_rtt_ns))
        self._max_offset_jitter_ns = max(1, int(max_offset_jitter_ns))
        self._pending_requests: OrderedDict[int, TimesyncOutboundRequest] = OrderedDict()
        self._recent_measurements: list[TimesyncMeasurement] = []

    def record_outbound_request(
        self,
        *,
        ts1: int | None = None,
        target_system: int = 0,
        target_component: int = 0,
        sent_monotonic_ns: int | None = None,
        sent_wall_ns: int | None = None,
    ) -> TimesyncOutboundRequest:
        monotonic_ns, wall_ns = _receive_timestamps(
            received_monotonic_ns=sent_monotonic_ns,
            received_wall_ns=sent_wall_ns,
        )
        request_ts1 = wall_ns if ts1 is None else int(ts1)
        request = TimesyncOutboundRequest(
            ts1=request_ts1,
            target_system=int(target_system),
            target_component=int(target_component),
            sent_monotonic_ns=monotonic_ns,
            sent_wall_ns=wall_ns,
        )
        with self._lock:
            self._pending_requests[request.ts1] = request
            self._prune_pending_requests_locked()
            snapshot = self._snapshot
            self._snapshot = TimesyncSnapshot(
                message_count=snapshot.message_count,
                request_count=snapshot.request_count,
                response_count=snapshot.response_count,
                outbound_request_count=snapshot.outbound_request_count + 1,
                matched_response_count=snapshot.matched_response_count,
                pending_request_count=len(self._pending_requests),
                last_message=snapshot.last_message,
                last_request=snapshot.last_request,
                last_response=snapshot.last_response,
                last_outbound_request=request,
                last_measurement=snapshot.last_measurement,
                best_measurement=snapshot.best_measurement,
                estimated_offset_ns=snapshot.estimated_offset_ns,
                estimated_rtt_ns=snapshot.estimated_rtt_ns,
                stable_offset_ns=snapshot.stable_offset_ns,
                stable_rtt_ns=snapshot.stable_rtt_ns,
                offset_jitter_ns=snapshot.offset_jitter_ns,
                sync_health=snapshot.sync_health,
                remote_supports_legacy_broadcast=snapshot.remote_supports_legacy_broadcast,
            )
        return request

    def handle_message(
        self,
        message: _TimesyncMessageLike | object,
        *,
        received_monotonic_ns: int | None = None,
        received_wall_ns: int | None = None,
    ) -> TimesyncEvent:
        event = parse_timesync_message(
            message,
            received_monotonic_ns=received_monotonic_ns,
            received_wall_ns=received_wall_ns,
        )
        self.record_event(event)
        return event

    def record_event(self, event: TimesyncEvent) -> None:
        with self._lock:
            measurement = self._measurement_for_response_locked(event)
            self._snapshot = self._snapshot_after(event, measurement)

    def snapshot(self) -> TimesyncSnapshot:
        with self._lock:
            return self._snapshot

    def _measurement_for_response_locked(self, event: TimesyncEvent) -> TimesyncMeasurement | None:
        if not event.is_response:
            return None

        request = self._pending_requests.get(event.ts1)
        if request is None or not self._response_matches_local_component(event):
            return None

        self._pending_requests.pop(event.ts1, None)
        legacy = event.target_system == 0 and event.target_component == 0
        return TimesyncMeasurement(
            ts1=event.ts1,
            remote_tc1=event.tc1,
            request_sent_monotonic_ns=request.sent_monotonic_ns,
            response_received_monotonic_ns=event.received_monotonic_ns,
            request_sent_wall_ns=request.sent_wall_ns,
            response_received_wall_ns=event.received_wall_ns,
            rtt_monotonic_ns=max(0, event.received_monotonic_ns - request.sent_monotonic_ns),
            rtt_wall_ns=max(0, event.received_wall_ns - request.sent_wall_ns),
            offset_ns=event.tc1 - ((request.sent_wall_ns + event.received_wall_ns) // 2),
            response_target_system=event.target_system,
            response_target_component=event.target_component,
            legacy_broadcast_response=legacy,
        )

    def _response_matches_local_component(self, event: TimesyncEvent) -> bool:
        if event.target_system == 0 and event.target_component == 0:
            return True
        if self._local_system is not None and event.target_system not in (0, self._local_system):
            return False
        if (
            self._local_component is not None
            and event.target_component not in (0, self._local_component)
        ):
            return False
        return True

    def _snapshot_after(
        self,
        event: TimesyncEvent,
        measurement: TimesyncMeasurement | None,
    ) -> TimesyncSnapshot:
        snapshot = self._snapshot
        best_measurement = snapshot.best_measurement
        if measurement is not None and (
            best_measurement is None or measurement.rtt_wall_ns < best_measurement.rtt_wall_ns
        ):
            best_measurement = measurement

        if measurement is not None:
            self._recent_measurements.append(measurement)
            self._prune_recent_measurements_locked()

        last_measurement = measurement or snapshot.last_measurement
        stable_offset_ns, stable_rtt_ns, offset_jitter_ns, stable_sample_count = (
            self._stable_estimate_locked()
        )
        sync_health = self._sync_health_for(
            matched_response_count=snapshot.matched_response_count + int(measurement is not None),
            pending_request_count=len(self._pending_requests),
            stable_sample_count=stable_sample_count,
            stable_offset_ns=stable_offset_ns,
            stable_rtt_ns=stable_rtt_ns,
            offset_jitter_ns=offset_jitter_ns,
        )
        remote_supports_legacy_broadcast = snapshot.remote_supports_legacy_broadcast or bool(
            measurement is not None and measurement.legacy_broadcast_response
        )

        return TimesyncSnapshot(
            message_count=snapshot.message_count + 1,
            request_count=snapshot.request_count + int(event.is_request),
            response_count=snapshot.response_count + int(event.is_response),
            outbound_request_count=snapshot.outbound_request_count,
            matched_response_count=snapshot.matched_response_count + int(measurement is not None),
            pending_request_count=len(self._pending_requests),
            last_message=event,
            last_request=event if event.is_request else snapshot.last_request,
            last_response=event if event.is_response else snapshot.last_response,
            last_outbound_request=snapshot.last_outbound_request,
            last_measurement=last_measurement,
            best_measurement=best_measurement,
            estimated_offset_ns=(
                best_measurement.offset_ns if best_measurement is not None else None
            ),
            estimated_rtt_ns=(
                best_measurement.rtt_wall_ns if best_measurement is not None else None
            ),
            stable_offset_ns=stable_offset_ns,
            stable_rtt_ns=stable_rtt_ns,
            offset_jitter_ns=offset_jitter_ns,
            sync_health=sync_health,
            remote_supports_legacy_broadcast=remote_supports_legacy_broadcast,
        )

    def _prune_pending_requests_locked(self) -> None:
        while len(self._pending_requests) > self._pending_request_limit:
            self._pending_requests.popitem(last=False)

    def _prune_recent_measurements_locked(self) -> None:
        if len(self._recent_measurements) > self._stable_window_size:
            self._recent_measurements = self._recent_measurements[-self._stable_window_size :]

    def _stable_estimate_locked(self) -> tuple[int | None, int | None, int | None, int]:
        if not self._recent_measurements:
            return None, None, None, 0

        chosen = sorted(self._recent_measurements, key=lambda item: item.rtt_wall_ns)[
            : self._stable_best_subset_size
        ]
        offsets = sorted(measurement.offset_ns for measurement in chosen)
        rtts = sorted(measurement.rtt_wall_ns for measurement in chosen)
        stable_offset_ns = int(median(offsets))
        stable_rtt_ns = int(median(rtts))
        offset_jitter_ns = max(offsets) - min(offsets)
        return stable_offset_ns, stable_rtt_ns, offset_jitter_ns, len(chosen)

    def _sync_health_for(
        self,
        *,
        matched_response_count: int,
        pending_request_count: int,
        stable_sample_count: int,
        stable_offset_ns: int | None,
        stable_rtt_ns: int | None,
        offset_jitter_ns: int | None,
    ) -> TimesyncHealth:
        if matched_response_count == 0 or stable_offset_ns is None or stable_rtt_ns is None:
            return TimesyncHealth(
                status="unsynced",
                sample_count=matched_response_count,
                stable_sample_count=stable_sample_count,
                stable_offset_ns=stable_offset_ns,
                stable_rtt_ns=stable_rtt_ns,
                offset_jitter_ns=offset_jitter_ns,
                reason="no stable matched TIMESYNC responses yet",
            )
        if stable_sample_count < self._min_stable_samples:
            return TimesyncHealth(
                status="warming_up",
                sample_count=matched_response_count,
                stable_sample_count=stable_sample_count,
                stable_offset_ns=stable_offset_ns,
                stable_rtt_ns=stable_rtt_ns,
                offset_jitter_ns=offset_jitter_ns,
                reason="collecting stable TIMESYNC samples",
            )
        degraded_reasons: list[str] = []
        if stable_rtt_ns > self._max_stable_rtt_ns:
            degraded_reasons.append("RTT too high")
        if offset_jitter_ns is not None and offset_jitter_ns > self._max_offset_jitter_ns:
            degraded_reasons.append("offset jitter too high")
        if pending_request_count > max(2, self._pending_request_limit // 4):
            degraded_reasons.append("too many pending requests")
        if degraded_reasons:
            return TimesyncHealth(
                status="degraded",
                sample_count=matched_response_count,
                stable_sample_count=stable_sample_count,
                stable_offset_ns=stable_offset_ns,
                stable_rtt_ns=stable_rtt_ns,
                offset_jitter_ns=offset_jitter_ns,
                reason=", ".join(degraded_reasons),
            )
        return TimesyncHealth(
            status="stable",
            sample_count=matched_response_count,
            stable_sample_count=stable_sample_count,
            stable_offset_ns=stable_offset_ns,
            stable_rtt_ns=stable_rtt_ns,
            offset_jitter_ns=offset_jitter_ns,
            reason="stable RTT and offset jitter",
        )


def _required_attr(message: object, attr_name: str) -> object:
    if not hasattr(message, attr_name):
        raise ValueError(f"TIMESYNC message is missing required field {attr_name!r}.")
    return getattr(message, attr_name)


def _receive_timestamps(
    *,
    received_monotonic_ns: int | None,
    received_wall_ns: int | None,
) -> tuple[int, int]:
    return (
        time.monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns,
        time.time_ns() if received_wall_ns is None else received_wall_ns,
    )


def _optional_source_id(message: object, getter_name: str) -> int | None:
    getter = getattr(message, getter_name, None)
    if not callable(getter):
        return None
    value = getter()
    if value is None:
        return None
    return int(value)
