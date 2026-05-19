"""Shared MAVLink test doubles for unit tests (fake connection + mav sender)."""

from __future__ import annotations

import queue
import time
from contextlib import contextmanager
from typing import Any
from unittest.mock import patch


class FakeMessage:
    def __init__(self, message_type: str, **fields: Any) -> None:
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


class FakeMavSender:
    def __init__(self) -> None:
        self.command_long_calls: list[tuple[Any, ...]] = []
        self.message_interval_calls: list[tuple[Any, ...]] = []
        self.position_target_calls: list[tuple[Any, ...]] = []
        self.attitude_target_calls: list[tuple[Any, ...]] = []
        self.timesync_calls: list[tuple[int, int]] = []

    def command_long_send(self, *args: Any) -> None:
        self.command_long_calls.append(args)

    def message_interval_send(self, *args: Any) -> None:
        self.message_interval_calls.append(args)

    def set_position_target_local_ned_send(self, *args: Any) -> None:
        self.position_target_calls.append(args)

    def set_attitude_target_send(self, *args: Any) -> None:
        self.attitude_target_calls.append(args)

    def timesync_send(self, tc1: int, ts1: int) -> None:
        self.timesync_calls.append((tc1, ts1))


class FakeMavConnection:
    def __init__(self, heartbeat: FakeMessage, queued_messages: list[FakeMessage]) -> None:
        self._heartbeat = heartbeat
        self._queue: queue.Queue[FakeMessage] = queue.Queue()
        for message in queued_messages:
            self.push_message(message)
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self.mav = FakeMavSender()
        self.closed = False

    def wait_heartbeat(self, timeout: float | None = None) -> FakeMessage:
        _ = timeout
        return self._heartbeat

    def recv_match(
        self,
        type: str | list[str] | None = None,
        condition: str | None = None,
        blocking: bool = True,
        timeout: float | None = None,
    ) -> FakeMessage | None:
        _ = condition
        if type is None:
            types_filter: list[str] | None = None
        elif isinstance(type, str):
            types_filter = [type]
        else:
            types_filter = list(type)
        if not blocking:
            while True:
                try:
                    message = self._queue.get_nowait()
                except queue.Empty:
                    return None
                if types_filter is None or message.get_type() in types_filter:
                    return message

        deadline = time.time() + (timeout or 0.0)
        while True:
            remaining = max(0.0, deadline - time.time()) if timeout is not None else None
            try:
                message = self._queue.get(timeout=remaining)
            except queue.Empty:
                return None
            if types_filter is None or message.get_type() in types_filter:
                return message

    def close(self) -> None:
        self.closed = True

    def push_message(self, message: FakeMessage) -> None:
        self._queue.put(message)


@contextmanager
def fake_mavlink_monotonic_sleep():
    clock = {"t": 1000.0}

    def monotonic() -> float:
        return clock["t"]

    def sleep(dt: float) -> None:
        clock["t"] += max(float(dt), 0.0)

    with patch("src.control.mavlink_client.time.monotonic", monotonic), patch(
        "src.control.mavlink_client.time.sleep", sleep
    ):
        yield clock


class FakeMav:
    def __init__(self) -> None:
        self.message_interval_calls: list[tuple[int, int]] = []

    def message_interval_send(self, message_id: int, interval_us: int) -> None:
        self.message_interval_calls.append((int(message_id), int(interval_us)))
