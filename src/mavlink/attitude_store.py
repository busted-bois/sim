"""Thread-safe store for latest MAVLink ATTITUDE samples."""

from __future__ import annotations

import threading
import time
from typing import Any

from src.mavlink.attitude import AttitudeHealth, AttitudeSample


class AttitudeStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sample: AttitudeSample | None = None
        self._sample_count = 0
        self._first_monotonic_ns: int | None = None
        self._last_monotonic_ns: int | None = None

    def update(self, sample: AttitudeSample) -> None:
        received_ns = (
            sample.local_received_monotonic_ns
            if sample.local_received_monotonic_ns
            else time.monotonic_ns()
        )
        with self._lock:
            self._sample = sample
            self._sample_count += 1
            if self._first_monotonic_ns is None:
                self._first_monotonic_ns = received_ns
            self._last_monotonic_ns = received_ns

    def get(self) -> AttitudeSample | None:
        with self._lock:
            return self._sample

    def sample_count(self) -> int:
        with self._lock:
            return self._sample_count

    def get_health(
        self,
        *,
        enabled: bool,
        max_staleness_ms: float,
        expected_rate_hz: float | None = None,
    ) -> AttitudeHealth:
        with self._lock:
            sample = self._sample
            sample_count = self._sample_count
            first_ns = self._first_monotonic_ns
            last_ns = self._last_monotonic_ns

        if not enabled:
            return AttitudeHealth(
                status="disabled",
                reason="ATTITUDE stream disabled in config",
                enabled=False,
                sample_count=sample_count,
                stream_rate_hz=None,
                update_age_ms=None,
                max_staleness_ms=max_staleness_ms,
                expected_rate_hz=expected_rate_hz,
            )
        if sample is None or last_ns is None:
            return AttitudeHealth(
                status="missing",
                reason="no ATTITUDE samples received yet",
                enabled=True,
                sample_count=sample_count,
                stream_rate_hz=None,
                update_age_ms=None,
                max_staleness_ms=max_staleness_ms,
                expected_rate_hz=expected_rate_hz,
            )

        now_ns = time.monotonic_ns()
        update_age_ms = (now_ns - last_ns) / 1_000_000.0
        stream_rate_hz = None
        if sample_count >= 2 and first_ns is not None and last_ns > first_ns:
            stream_rate_hz = (sample_count - 1) / ((last_ns - first_ns) / 1_000_000_000.0)

        status = "ok"
        reason = "ATTITUDE samples are fresh"
        if update_age_ms > max_staleness_ms:
            status = "stale"
            reason = (
                f"latest ATTITUDE sample age {update_age_ms:.1f} ms exceeds "
                f"{max_staleness_ms:.1f} ms"
            )

        return AttitudeHealth(
            status=status,
            reason=reason,
            enabled=True,
            sample_count=sample_count,
            stream_rate_hz=stream_rate_hz,
            update_age_ms=update_age_ms,
            max_staleness_ms=max_staleness_ms,
            expected_rate_hz=expected_rate_hz,
        )


def message_source_id(message: Any, method_name: str) -> int | None:
    getter = getattr(message, method_name, None)
    if not callable(getter):
        return None
    value = getter()
    return int(value) if value is not None else None
