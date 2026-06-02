"""Align propagated states to video sim_time_ns."""

from __future__ import annotations

import bisect
import threading
from collections import deque

from src.tracking.state import TrackingState


class SimTimeHistory:
    """Ring buffer of (sim_time_ns, TrackingState) for interpolation."""

    def __init__(self, *, max_samples: int = 500) -> None:
        self._max_samples = max(10, int(max_samples))
        self._lock = threading.Lock()
        self._times: deque[int] = deque(maxlen=self._max_samples)
        self._states: deque[TrackingState] = deque(maxlen=self._max_samples)

    def append(self, state: TrackingState) -> None:
        with self._lock:
            if self._times and state.sim_time_ns < self._times[-1]:
                return
            self._times.append(state.sim_time_ns)
            self._states.append(state)

    def nearest(self, sim_time_ns: int) -> TrackingState | None:
        with self._lock:
            if not self._times:
                return None
            times = list(self._times)
            states = list(self._states)
        idx = bisect.bisect_left(times, sim_time_ns)
        if idx <= 0:
            return states[0]
        if idx >= len(times):
            return states[-1]
        before = times[idx - 1]
        after = times[idx]
        if abs(sim_time_ns - before) <= abs(after - sim_time_ns):
            return states[idx - 1]
        return states[idx]
