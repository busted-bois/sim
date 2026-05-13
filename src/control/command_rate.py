from __future__ import annotations

import threading
import time


def normalize_command_rate_hz(raw_rate_hz: float, *, default: float = 50.0) -> float:
    try:
        rate_hz = float(raw_rate_hz)
    except (TypeError, ValueError):
        rate_hz = float(default)
    return max(1.0, min(99.0, rate_hz))


class SkippedAsyncResult:
    """No-op future returned when a motion command is intentionally dropped."""

    dropped = True

    def join(self, timeout: float | None = None) -> None:
        _ = timeout
        return


class CommandRateGate:
    """Drop commands that arrive faster than the configured rate."""

    def __init__(self, rate_hz: float) -> None:
        self._rate_hz = normalize_command_rate_hz(rate_hz)
        self._period_s = 1.0 / self._rate_hz
        self._lock = threading.Lock()
        self._next_allowed_s = 0.0

    @property
    def period_s(self) -> float:
        return self._period_s

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    def allow(self, now_s: float | None = None) -> bool:
        if now_s is None:
            now_s = time.monotonic()
        with self._lock:
            if now_s + 1e-9 < self._next_allowed_s:
                return False
            self._next_allowed_s = now_s + self._period_s
            return True
