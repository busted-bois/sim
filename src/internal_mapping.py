"""NED pose / velocity CSV from fused tracking or flight client RPC fallback.

Separate from ``exploration.slam`` map export. Samples on a background thread and
flushes each row so Ctrl+C still leaves a usable trace.
"""

from __future__ import annotations

import csv
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, TextIO

from src.control.utils import _yaw_from_orientation
from src.log_paths import resolve_log_csv_path

if TYPE_CHECKING:
    from src.control.flight_client import FlightClient

DataSource = Literal["tracking", "rpc"]

CSV_HEADER = (
    "session_t_s",
    "airsim_timestamp",
    "pos_n_m",
    "pos_e_m",
    "pos_d_m",
    "yaw_rad",
    "vn_ms",
    "ve_ms",
    "vd_ms",
    "roll_rad",
    "pitch_rad",
    "tracking_status",
    "imu_rate_hz",
    "imu_time_usec",
    "xacc",
    "yacc",
    "zacc",
    "xgyro",
    "ygyro",
    "zgyro",
)


def internal_mapping_config(config: dict[str, Any]) -> dict[str, Any]:
    """``autonomous_explore.exploration.internal_mapping`` or legacy top-level key."""
    explore = config.get("autonomous_explore", {}).get("exploration", {}).get(
        "internal_mapping", {}
    )
    if explore:
        return dict(explore)
    return dict(config.get("internal_mapping", {}))


def resolve_internal_mapping_data_source(
    config: dict[str, Any],
    *,
    explicit: str | None = None,
) -> DataSource:
    if explicit in ("tracking", "rpc"):
        return explicit  # type: ignore[return-value]
    mav_cfg = config.get("control", {}).get("mavlink", {})
    tracking_enabled = bool(mav_cfg.get("tracking", {}).get("enabled", False))
    transport = str(config.get("control", {}).get("transport", "")).lower()
    if tracking_enabled and transport == "mavlink":
        return "tracking"
    return "rpc"


def internal_mapping_logger_from_config(
    config: dict[str, Any],
    project_root: Path,
    client: FlightClient,
) -> InternalMappingLogger | None:
    cfg = internal_mapping_config(config)
    if not bool(cfg.get("enabled", False)):
        return None
    out_path = resolve_log_csv_path(
        str(cfg.get("path", "logs/internal_mapping_{timestamp}.csv")),
        project_root,
        default="logs/internal_mapping_{timestamp}.csv",
    )
    data_source = resolve_internal_mapping_data_source(
        config,
        explicit=str(cfg.get("data_source", "")).strip().lower() or None,
    )
    return InternalMappingLogger(
        client,
        out_path=out_path,
        sample_hz=float(cfg.get("sample_hz", 20.0)),
        data_source=data_source,
        log_imu=bool(cfg.get("log_imu", False)),
    )


class InternalMappingLogger:
    """Threaded CSV writer; NED frame (x North, y East, z Down)."""

    def __init__(
        self,
        client: FlightClient,
        *,
        out_path: Path,
        sample_hz: float = 20.0,
        data_source: DataSource = "rpc",
        log_imu: bool = False,
    ) -> None:
        self._client = client
        self.out_path = out_path
        self._period = 1.0 / max(1.0, float(sample_hz))
        self._data_source = data_source
        self._log_imu = log_imu
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fh: TextIO | None = None
        self._writer: csv.writer | None = None
        self._t0 = 0.0
        self._row_count = 0

    @property
    def row_count(self) -> int:
        return self._row_count

    def start(self) -> None:
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self._t0 = time.monotonic()
        self._fh = self.out_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(CSV_HEADER)
        self._fh.flush()

        def loop() -> None:
            while not self._stop.is_set():
                self._sample()
                if self._stop.wait(self._period):
                    break
            self._sample()

        self._thread = threading.Thread(target=loop, name="internal_mapping", daemon=True)
        self._thread.start()

    def _sample(self) -> None:
        if self._writer is None or self._fh is None:
            return
        session_t = time.monotonic() - self._t0
        try:
            row = self._build_row(session_t)
            self._writer.writerow(row)
            self._fh.flush()
            self._row_count += 1
        except Exception as exc:
            self._writer.writerow(self._error_row(session_t, exc))
            self._fh.flush()

    def _build_row(self, session_t: float) -> list[str]:
        if self._data_source == "tracking":
            tracking_row = self._sample_tracking(session_t)
            if tracking_row is not None:
                return tracking_row
            ned_row = self._sample_ned_environment(session_t)
            if ned_row is not None:
                return ned_row
        return self._sample_rpc(session_t)

    def _sample_tracking(self, session_t: float) -> list[str] | None:
        getter = getattr(self._client, "getTrackingSnapshot", None)
        if not callable(getter):
            return None
        snapshot = getter()
        if snapshot is None:
            return None
        health = snapshot.health
        if health.status not in ("ok", "degraded"):
            return None
        state = snapshot.state
        if state is not None:
            x, y, z = state.position_ned
            vx, vy, vz = state.velocity_ned
            roll, pitch, yaw = state.attitude_rpy
            ts = state.sim_time_ns
        else:
            x, y, z = snapshot.position_ned
            vx, vy, vz = snapshot.velocity_ned
            roll, pitch, yaw = snapshot.attitude_rpy
            ts = snapshot.sim_time_ns
        return self._format_row(
            session_t=session_t,
            ts=ts,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            vx=vx,
            vy=vy,
            vz=vz,
            roll=roll,
            pitch=pitch,
            tracking_status=health.status,
            imu_rate_hz=health.imu_rate_hz,
        )

    def _sample_ned_environment(self, session_t: float) -> list[str] | None:
        getter = getattr(self._client, "get_ned_environment", None)
        if not callable(getter):
            return None
        ned = getter()
        if ned is None:
            return None
        snap = ned.snapshot()
        if not snap.has_position:
            return None
        att = snap.attitude
        roll = att.roll if att is not None else 0.0
        pitch = att.pitch if att is not None else 0.0
        yaw = att.yaw if att is not None else 0.0
        ts = snap.position_time_boot_ms or 0
        return self._format_row(
            session_t=session_t,
            ts=ts,
            x=snap.position.x,
            y=snap.position.y,
            z=snap.position.z,
            yaw=yaw,
            vx=snap.velocity.vx,
            vy=snap.velocity.vy,
            vz=snap.velocity.vz,
            roll=roll,
            pitch=pitch,
            tracking_status="ned_fallback",
            imu_rate_hz=None,
        )

    def _sample_rpc(self, session_t: float) -> list[str]:
        state = self._client.getMultirotorState()
        k = state.kinematics_estimated
        p = k.position
        v = k.linear_velocity
        yaw = _yaw_from_orientation(k.orientation)
        return self._format_row(
            session_t=session_t,
            ts=int(state.timestamp),
            x=float(p.x_val),
            y=float(p.y_val),
            z=float(p.z_val),
            yaw=yaw,
            vx=float(v.x_val),
            vy=float(v.y_val),
            vz=float(v.z_val),
            roll=0.0,
            pitch=0.0,
            tracking_status="rpc",
            imu_rate_hz=None,
        )

    def _format_row(
        self,
        *,
        session_t: float,
        ts: int,
        x: float,
        y: float,
        z: float,
        yaw: float,
        vx: float,
        vy: float,
        vz: float,
        roll: float,
        pitch: float,
        tracking_status: str,
        imu_rate_hz: float | None,
    ) -> list[str]:
        imu_cols = self._imu_columns()
        imu_rate = "" if imu_rate_hz is None else f"{imu_rate_hz:.1f}"
        return [
            f"{session_t:.6f}",
            str(int(ts)),
            f"{x:.6f}",
            f"{y:.6f}",
            f"{z:.6f}",
            f"{yaw:.6f}",
            f"{vx:.6f}",
            f"{vy:.6f}",
            f"{vz:.6f}",
            f"{roll:.6f}",
            f"{pitch:.6f}",
            tracking_status,
            imu_rate,
            *imu_cols,
        ]

    def _imu_columns(self) -> list[str]:
        if not self._log_imu:
            return ["", "", "", "", "", "", ""]
        sample = self._client.getHighresImu()
        if sample is None:
            return ["", "", "", "", "", "", ""]
        return [
            str(int(sample.time_usec)),
            f"{sample.xacc:.6f}",
            f"{sample.yacc:.6f}",
            f"{sample.zacc:.6f}",
            f"{sample.xgyro:.6f}",
            f"{sample.ygyro:.6f}",
            f"{sample.zgyro:.6f}",
        ]

    def _error_row(self, session_t: float, exc: Exception) -> list[str]:
        return [
            f"{session_t:.6f}",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            f"error:{exc}",
            "",
            "",
            "error",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
        ]

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        self._writer = None
