from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass


def normalize_command_rate_hz(raw_rate_hz: float, *, default: float = 50.0) -> float:
    try:
        rate_hz = float(raw_rate_hz)
    except (TypeError, ValueError):
        rate_hz = float(default)
    return max(1.0, min(99.0, rate_hz))


@dataclass(frozen=True, slots=True)
class CommandRateGateStats:
    rate_hz: float
    period_s: float
    allowed_count: int
    dropped_count: int
    attempted_count: int
    last_allowed_monotonic_s: float | None
    last_dropped_monotonic_s: float | None


class SkippedAsyncResult:
    """No-op future returned when a motion command is intentionally dropped."""

    dropped = True

    def join(self, timeout: float | None = None) -> None:
        _ = timeout
        return


class CommandRateGate:
    """Drop commands that arrive faster than the configured rate."""

    def __init__(
        self,
        rate_hz: float,
        *,
        label: str = "motion commands",
        reporter: Callable[[str], None] | None = print,
        report_interval_s: float = 5.0,
    ) -> None:
        self._rate_hz = normalize_command_rate_hz(rate_hz)
        self._period_s = 1.0 / self._rate_hz
        self._label = str(label).strip() or "motion commands"
        self._reporter = reporter
        self._report_interval_s = max(0.1, float(report_interval_s))
        self._lock = threading.Lock()
        self._next_allowed_s = 0.0
        self._allowed_count = 0
        self._dropped_count = 0
        self._last_allowed_monotonic_s: float | None = None
        self._last_dropped_monotonic_s: float | None = None
        self._last_report_monotonic_s = 0.0
        self._last_reported_drop_count = 0

    @property
    def period_s(self) -> float:
        return self._period_s

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    def stats(self) -> CommandRateGateStats:
        with self._lock:
            return CommandRateGateStats(
                rate_hz=self._rate_hz,
                period_s=self._period_s,
                allowed_count=self._allowed_count,
                dropped_count=self._dropped_count,
                attempted_count=self._allowed_count + self._dropped_count,
                last_allowed_monotonic_s=self._last_allowed_monotonic_s,
                last_dropped_monotonic_s=self._last_dropped_monotonic_s,
            )

    def _maybe_report_drop_locked(self, now_s: float) -> str | None:
        if self._reporter is None or self._dropped_count <= self._last_reported_drop_count:
            return None
        should_report = self._dropped_count == 1 or (
            now_s - self._last_report_monotonic_s >= self._report_interval_s
        )
        if not should_report:
            return None
        recent_drop_count = self._dropped_count - self._last_reported_drop_count
        self._last_reported_drop_count = self._dropped_count
        self._last_report_monotonic_s = now_s
        attempted_count = self._allowed_count + self._dropped_count
        return (
            f"[command-rate] {self._label}: dropped {recent_drop_count} command(s); "
            f"total dropped={self._dropped_count}, allowed={self._allowed_count}, "
            f"attempted={attempted_count}, limit={self._rate_hz:.1f} Hz"
        )

    def allow(self, now_s: float | None = None) -> bool:
        if now_s is None:
            now_s = time.monotonic()
        report_message: str | None = None
        with self._lock:
            if now_s + 1e-9 < self._next_allowed_s:
                self._dropped_count += 1
                self._last_dropped_monotonic_s = now_s
                report_message = self._maybe_report_drop_locked(now_s)
                allowed = False
            else:
                self._next_allowed_s = now_s + self._period_s
                self._allowed_count += 1
                self._last_allowed_monotonic_s = now_s
                allowed = True
        if report_message is not None and self._reporter is not None:
            self._reporter(report_message)
        return allowed
