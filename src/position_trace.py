"""MAVLink LOCAL_POSITION_NED trace — estimator fixes only, no velocity integration.

Each accepted sample stores x,y,z from the vehicle estimator (relative to a one-shot
anchor). Samples are validated to reject duplicates, time regression, non-finite values,
and implausible position jumps that would corrupt the map with ghost drift.
"""

from __future__ import annotations

import csv
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from src.log_paths import resolve_log_csv_path

PositionRow = tuple[float, int, float, float, float, float, float, float, float]

CSV_HEADER: tuple[str, ...] = (
    "t_s",
    "time_boot_ms",
    "x_m",
    "y_m",
    "z_m",
    "vx_ms",
    "vy_ms",
    "vz_ms",
    "altitude_m",
)


def _is_finite(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


@dataclass(frozen=True, slots=True)
class PositionTraceHealth:
    accepted_count: int
    rejected_count: int
    stream_rate_hz: float | None
    first_t_s: float | None
    last_t_s: float | None
    max_step_m: float | None
    last_reject_reason: str | None


@dataclass(frozen=True, slots=True)
class PositionTraceSnapshot:
    health: PositionTraceHealth
    latest: PositionRow | None


def resolve_position_trace_path(raw_path: str, project_root: Path) -> Path:
    return resolve_log_csv_path(raw_path, project_root, default="logs/position_trace.csv")


def position_trace_store_from_config(
    config: dict[str, Any],
    project_root: Path,
) -> PositionTraceStore | None:
    control_cfg = config.get("control", {})
    mav_cfg = control_cfg.get("mavlink", {})
    trace_cfg = mav_cfg.get("position_trace", {})
    hud_cfg = mav_cfg.get("position_hud", {})
    if not bool(trace_cfg.get("enabled", False)) and not bool(hud_cfg.get("enabled", False)):
        return None
    max_speed_ms = float(trace_cfg.get("max_speed_ms", control_cfg.get("max_speed_ms", 10.0)))
    return PositionTraceStore(
        out_path=resolve_position_trace_path(
            str(trace_cfg.get("path", "logs/position_trace.csv")),
            project_root,
        ),
        relative_origin=bool(trace_cfg.get("relative_origin", True)),
        max_samples=int(trace_cfg.get("max_samples", 0)),
        max_speed_ms=max(0.1, max_speed_ms),
        plausible_step_slack=max(1.0, float(trace_cfg.get("plausible_step_slack", 1.5))),
        allow_discontinuities=bool(trace_cfg.get("allow_discontinuities", False)),
    )


_EMPTY_TRACE_HEALTH = PositionTraceHealth(
    accepted_count=0,
    rejected_count=0,
    stream_rate_hz=None,
    first_t_s=None,
    last_t_s=None,
    max_step_m=None,
    last_reject_reason=None,
)


class RpcPositionSnapshotProvider:
    """HUD snapshot from ``getMultirotorState`` when transport is AirSim RPC (not MAVLink)."""

    def __init__(self, client: Any) -> None:
        self._client = client
        self._t0: float | None = None
        self._accepted_count = 0

    def __call__(self) -> PositionTraceSnapshot:
        try:
            state = self._client.getMultirotorState()
            k = state.kinematics_estimated
            p = k.position
            v = k.linear_velocity
            x_m = float(p.x_val)
            y_m = float(p.y_val)
            z_m = float(p.z_val)
            if not _is_finite(x_m, y_m, z_m):
                return PositionTraceSnapshot(health=_EMPTY_TRACE_HEALTH, latest=None)
            now = time.monotonic()
            if self._t0 is None:
                self._t0 = now
            t_s = now - self._t0
            boot_ms = int(getattr(state, "timestamp", 0) or 0)
            row: PositionRow = (
                t_s,
                boot_ms,
                x_m,
                y_m,
                z_m,
                float(v.x_val),
                float(v.y_val),
                float(v.z_val),
                max(0.0, -z_m),
            )
            self._accepted_count += 1
            health = PositionTraceHealth(
                accepted_count=self._accepted_count,
                rejected_count=0,
                stream_rate_hz=None,
                first_t_s=0.0,
                last_t_s=t_s,
                max_step_m=None,
                last_reject_reason=None,
            )
            return PositionTraceSnapshot(health=health, latest=row)
        except Exception:
            return PositionTraceSnapshot(health=_EMPTY_TRACE_HEALTH, latest=None)


class PositionTraceStore:
    """Thread-safe LOCAL_POSITION_NED history with drift-resistant validation."""

    def __init__(
        self,
        out_path: Path,
        *,
        relative_origin: bool = True,
        max_samples: int = 0,
        max_speed_ms: float = 10.0,
        plausible_step_slack: float = 1.5,
        allow_discontinuities: bool = False,
    ) -> None:
        self.out_path = out_path
        self._relative_origin = relative_origin
        self._max_samples = max(0, int(max_samples))
        self._max_speed_ms = max(0.1, float(max_speed_ms))
        self._plausible_step_slack = max(1.0, float(plausible_step_slack))
        self._allow_discontinuities = allow_discontinuities

        self._lock = threading.Lock()
        self._rows: list[PositionRow] = []
        self._deque: deque[PositionRow] | None = (
            deque(maxlen=self._max_samples) if self._max_samples else None
        )

        self._anchor_set = False
        self._x0 = 0.0
        self._y0 = 0.0
        self._z0 = 0.0
        self._t0_boot_ms = 0

        self._last_time_boot_ms: int | None = None
        self._last_raw_x = 0.0
        self._last_raw_y = 0.0
        self._last_raw_z = 0.0

        self._accepted_count = 0
        self._rejected_count = 0
        self._last_reject_reason: str | None = None
        self._max_step_m = 0.0
        self._first_t_s: float | None = None
        self._last_t_s: float | None = None
        self._latest: PositionRow | None = None
        self._flushed = False

    def record(
        self,
        time_boot_ms: int,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
        received_monotonic_ns: int,
    ) -> bool:
        _ = received_monotonic_ns
        boot_ms = int(time_boot_ms)
        xf, yf, zf = float(x), float(y), float(z)
        vxf, vyf, vzf = float(vx), float(vy), float(vz)

        with self._lock:
            if not _is_finite(xf, yf, zf, vxf, vyf, vzf):
                self._reject("non-finite position or velocity")
                return False
            if self._last_time_boot_ms is not None:
                if boot_ms == self._last_time_boot_ms:
                    self._reject("duplicate time_boot_ms")
                    return False
                if boot_ms < self._last_time_boot_ms:
                    self._reject("time_boot_ms regression")
                    return False

                dt_s = (boot_ms - self._last_time_boot_ms) / 1000.0
                step_m = _norm3(xf - self._last_raw_x, yf - self._last_raw_y, zf - self._last_raw_z)
                self._max_step_m = max(self._max_step_m, step_m)
                max_plausible = self._max_speed_ms * dt_s * self._plausible_step_slack
                if step_m > max_plausible and not self._allow_discontinuities:
                    self._reject(
                        f"position step {step_m:.3f} m exceeds plausible "
                        f"{max_plausible:.3f} m (dt={dt_s:.3f} s)"
                    )
                    return False

            if not self._anchor_set:
                self._anchor_set = True
                self._x0 = xf
                self._y0 = yf
                self._z0 = zf
                self._t0_boot_ms = boot_ms

            t_s = (boot_ms - self._t0_boot_ms) / 1000.0
            if self._relative_origin:
                x_out = xf - self._x0
                y_out = yf - self._y0
                z_out = zf - self._z0
            else:
                x_out, y_out, z_out = xf, yf, zf

            altitude_m = max(0.0, -z_out)
            row: PositionRow = (
                t_s,
                boot_ms,
                x_out,
                y_out,
                z_out,
                vxf,
                vyf,
                vzf,
                altitude_m,
            )
            self._append_row(row)
            self._accepted_count += 1
            self._last_time_boot_ms = boot_ms
            self._last_raw_x = xf
            self._last_raw_y = yf
            self._last_raw_z = zf
            self._latest = row
            if self._first_t_s is None:
                self._first_t_s = t_s
            self._last_t_s = t_s
            return True

    def _reject(self, reason: str) -> None:
        self._rejected_count += 1
        self._last_reject_reason = reason

    def _append_row(self, row: PositionRow) -> None:
        if self._deque is not None:
            self._deque.append(row)
            self._rows = list(self._deque)
        else:
            self._rows.append(row)

    def accepted_count(self) -> int:
        with self._lock:
            return self._accepted_count

    def health(self) -> PositionTraceHealth:
        with self._lock:
            return self._health_unlocked()

    def _health_unlocked(self) -> PositionTraceHealth:
        stream_rate_hz = None
        if (
            self._accepted_count >= 2
            and self._first_t_s is not None
            and self._last_t_s is not None
        ):
            span = self._last_t_s - self._first_t_s
            if span > 0.0:
                stream_rate_hz = (self._accepted_count - 1) / span
        max_step = self._max_step_m if self._accepted_count >= 2 else None
        return PositionTraceHealth(
            accepted_count=self._accepted_count,
            rejected_count=self._rejected_count,
            stream_rate_hz=stream_rate_hz,
            first_t_s=self._first_t_s,
            last_t_s=self._last_t_s,
            max_step_m=max_step,
            last_reject_reason=self._last_reject_reason,
        )

    def snapshot(self) -> PositionTraceSnapshot:
        with self._lock:
            return PositionTraceSnapshot(health=self._health_unlocked(), latest=self._latest)

    def rows(self) -> list[PositionRow]:
        with self._lock:
            return list(self._rows)

    def flush(self) -> None:
        with self._lock:
            if self._flushed:
                return
            self._flushed = True
            rows = list(self._rows)
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        with self.out_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(CSV_HEADER)
            writer.writerows(rows)
