from __future__ import annotations

import threading
import time
from dataclasses import dataclass

__all__ = [
    "TimesyncEvent",
    "TimesyncSnapshot",
    "TimesyncStore",
    "parse_timesync_message",
]

_TIMESYNC_MESSAGE_TYPE = "TIMESYNC"


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
class TimesyncSnapshot:
    message_count: int
    last_message: TimesyncEvent | None
    last_request: TimesyncEvent | None
    last_response: TimesyncEvent | None


def parse_timesync_message(
    message: object,
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

    return TimesyncEvent(
        tc1=tc1,
        ts1=ts1,
        target_system=target_system,
        target_component=target_component,
        source_system=source_system,
        source_component=source_component,
        received_monotonic_ns=(
            time.monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns
        ),
        received_wall_ns=(time.time_ns() if received_wall_ns is None else received_wall_ns),
    )


class TimesyncStore:
    """Thread-safe store for inbound MAVLink TIMESYNC messages."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._message_count = 0
        self._last_message: TimesyncEvent | None = None
        self._last_request: TimesyncEvent | None = None
        self._last_response: TimesyncEvent | None = None

    def handle_message(
        self,
        message: object,
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
            self._record_event_locked(event)

    def snapshot(self) -> TimesyncSnapshot:
        with self._lock:
            return TimesyncSnapshot(
                message_count=self._message_count,
                last_message=self._last_message,
                last_request=self._last_request,
                last_response=self._last_response,
            )

    def _record_event_locked(self, event: TimesyncEvent) -> None:
        self._message_count += 1
        self._last_message = event
        if event.is_request:
            self._last_request = event
        else:
            self._last_response = event


def _required_attr(message: object, attr_name: str) -> object:
    if not hasattr(message, attr_name):
        raise ValueError(f"TIMESYNC message is missing required field {attr_name!r}.")
    return getattr(message, attr_name)


def _optional_source_id(message: object, getter_name: str) -> int | None:
    getter = getattr(message, getter_name, None)
    if not callable(getter):
        return None
    value = getter()
    if value is None:
        return None
    return int(value)
