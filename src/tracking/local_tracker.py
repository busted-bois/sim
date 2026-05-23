"""Orchestrates origin, IMU propagation, estimator blend, vision correction, CSV log."""

from __future__ import annotations

import csv
import json
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from src.control.highres_imu import HighresImuSample
from src.log_paths import resolve_log_csv_path
from src.tracking.imu_propagator import ImuPropagator
from src.tracking.origin import OriginManager
from src.tracking.snapshot import TrackingSnapshot
from src.tracking.state import TrackingHealth, TrackingState
from src.tracking.sync import SimTimeHistory
from src.tracking.vision_correction import VisionCorrector

TRACKING_CSV_HEADER = (
    "sim_time_ns",
    "t_s",
    "time_boot_ms",
    "x_m",
    "y_m",
    "z_m",
    "vx_ms",
    "vy_ms",
    "vz_ms",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "armed",
)


@dataclass(frozen=True, slots=True)
class TrackingConfig:
    enabled: bool = True
    origin_on_arm: bool = True
    imu_propagation: bool = True
    local_position_blend: float = 0.2
    vision_correction: bool = True
    log_csv: bool = True
    csv_path: Path = Path("logs/tracking_state.csv")
    max_speed_ms: float = 10.0
    pitch_up_degrees: float = 20.0
    gate_width_m: float = 1.5


def resolve_tracking_csv_path(raw_path: str, project_root: Path) -> Path:
    return resolve_log_csv_path(raw_path, project_root, default="logs/tracking_state.csv")


def local_tracker_from_config(config: dict[str, Any], project_root: Path) -> LocalTracker | None:
    mav_cfg = config.get("control", {}).get("mavlink", {})
    trace_cfg = mav_cfg.get("tracking", {})
    if not bool(trace_cfg.get("enabled", False)):
        return None
    control_cfg = config.get("control", {})
    camera_cfg = config.get("camera", {})
    gate_dims = config.get("simulator", {})
    gate_w = 1.5
    spec_path = gate_dims.get("specification_path")
    if spec_path:
        try:
            spec_file = Path(spec_path)
            if not spec_file.is_absolute():
                spec_file = project_root / spec_file
            if spec_file.is_file():
                data = json.loads(spec_file.read_text(encoding="utf-8"))
                ref = data.get("gate_reference", {}).get("dimensions_m", [1.5, 1.5, 0.5])
                if isinstance(ref, list) and ref:
                    gate_w = float(ref[0])
        except (OSError, json.JSONDecodeError, TypeError, ValueError):
            pass

    return LocalTracker(
        TrackingConfig(
            enabled=True,
            origin_on_arm=bool(trace_cfg.get("origin_on_arm", True)),
            imu_propagation=bool(trace_cfg.get("imu_propagation", True)),
            local_position_blend=float(trace_cfg.get("local_position_blend", 0.2)),
            vision_correction=bool(trace_cfg.get("vision_correction", True)),
            log_csv=bool(trace_cfg.get("log_csv", True)),
            csv_path=resolve_tracking_csv_path(str(trace_cfg.get("path", "")), project_root),
            max_speed_ms=float(
                trace_cfg.get("max_speed_ms", control_cfg.get("max_speed_ms", 10.0))
            ),
            pitch_up_degrees=float(camera_cfg.get("pitch_up_degrees", 20.0)),
            gate_width_m=float(trace_cfg.get("gate_width_m", gate_w)),
        )
    )


class LocalTracker:
    def __init__(self, config: TrackingConfig) -> None:
        self._config = config
        self._lock = threading.Lock()
        self._origin = OriginManager(enabled=config.origin_on_arm)
        self._propagator = ImuPropagator(
            max_speed_ms=config.max_speed_ms,
        )
        self._vision = VisionCorrector()
        self._history = SimTimeHistory()
        self._armed = False
        self._imu_count = 0
        self._imu_first_ns: int | None = None
        self._imu_last_ns: int | None = None
        self._vision_corrections = 0
        self._latest_image: np.ndarray | None = None
        self._latest_sim_time_ns = 0
        self._rows: list[tuple] = []
        self._csv_flushed = False

    @property
    def csv_path(self) -> Path:
        return self._config.csv_path

    def on_heartbeat_armed(self, armed: bool) -> None:
        with self._lock:
            self._armed = armed

    def on_local_position(
        self,
        *,
        time_boot_ms: int,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
        armed: bool,
    ) -> None:
        with self._lock:
            self._armed = armed
            sim_ns = self._origin.sim_time_ns_from_boot_ms(time_boot_ms)
            if self._origin.on_arm_transition(
                armed,
                raw_x=x,
                raw_y=y,
                raw_z=z,
                time_boot_ms=time_boot_ms,
                sim_time_ns=sim_ns,
            ):
                self._propagator.reset()
                self._imu_count = 0

            rx, ry, rz, t_s = self._origin.to_relative(x, y, z, time_boot_ms)
            blend = max(0.0, min(1.0, self._config.local_position_blend))
            if blend > 0.0 and self._origin.origin.set:
                st = self._propagator.state
                st.x = (1.0 - blend) * st.x + blend * rx
                st.y = (1.0 - blend) * st.y + blend * ry
                st.z = (1.0 - blend) * st.z + blend * rz
                st.vx = (1.0 - blend) * st.vx + blend * float(vx)
                st.vy = (1.0 - blend) * st.vy + blend * float(vy)
                st.vz = (1.0 - blend) * st.vz + blend * float(vz)

            state = self._build_state(sim_ns, t_s, time_boot_ms)
            self._latest_sim_time_ns = sim_ns
            self._history.append(state)
            self._maybe_log_row(state, time_boot_ms)

    def on_highres_imu(self, sample: HighresImuSample) -> None:
        if not self._config.imu_propagation:
            return
        with self._lock:
            self._imu_count += 1
            now_ns = sample.local_received_monotonic_ns
            if self._imu_first_ns is None:
                self._imu_first_ns = now_ns
            self._imu_last_ns = now_ns

            if not self._origin.origin.set:
                return

            self._propagator.step(
                sample.time_usec,
                sample.xacc,
                sample.yacc,
                sample.zacc,
                sample.xgyro,
                sample.ygyro,
                sample.zgyro,
            )
            boot_ms = int(sample.time_usec // 1000)
            _, _, _, t_s = self._origin.to_relative(0.0, 0.0, 0.0, boot_ms)
            sim_ns = self._origin.sim_time_ns_from_boot_ms(boot_ms)
            st = self._propagator.state
            state = TrackingState(
                sim_time_ns=sim_ns,
                t_s=t_s,
                position_ned=(st.x, st.y, st.z),
                velocity_ned=(st.vx, st.vy, st.vz),
                attitude_rpy=(st.roll, st.pitch, st.yaw),
                armed=self._armed,
                origin_set=self._origin.origin.set,
            )
            self._history.append(state)

    def on_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        with self._lock:
            if self._config.imu_propagation:
                self._propagator.set_attitude(roll, pitch, yaw)

    def on_video_frame(self, image_rgb: np.ndarray, sim_time_ns: int) -> None:
        if not self._config.vision_correction:
            with self._lock:
                self._latest_image = image_rgb.copy()
                self._latest_sim_time_ns = int(sim_time_ns)
            return
        with self._lock:
            self._latest_image = image_rgb.copy()
            self._latest_sim_time_ns = int(sim_time_ns)
            base = self._history.nearest(sim_time_ns)
            if base is None:
                return
            result = self._vision.correct(image_rgb, self._propagator)
            if result.applied:
                self._vision_corrections += 1
            st = self._propagator.state
            state = TrackingState(
                sim_time_ns=sim_time_ns,
                t_s=base.t_s,
                position_ned=(st.x, st.y, st.z),
                velocity_ned=(st.vx, st.vy, st.vz),
                attitude_rpy=(st.roll, st.pitch, st.yaw),
                armed=self._armed,
                origin_set=self._origin.origin.set,
            )
            self._history.append(state)

    def _build_state(self, sim_time_ns: int, t_s: float, time_boot_ms: int) -> TrackingState:
        st = self._propagator.state
        return TrackingState(
            sim_time_ns=sim_time_ns,
            t_s=t_s,
            position_ned=(st.x, st.y, st.z),
            velocity_ned=(st.vx, st.vy, st.vz),
            attitude_rpy=(st.roll, st.pitch, st.yaw),
            armed=self._armed,
            origin_set=self._origin.origin.set,
        )

    def _maybe_log_row(self, state: TrackingState, time_boot_ms: int) -> None:
        if not self._config.log_csv:
            return
        st = self._propagator.state
        self._rows.append(
            (
                state.sim_time_ns,
                state.t_s,
                time_boot_ms,
                state.position_ned[0],
                state.position_ned[1],
                state.position_ned[2],
                state.velocity_ned[0],
                state.velocity_ned[1],
                state.velocity_ned[2],
                st.roll,
                st.pitch,
                st.yaw,
                int(state.armed),
            )
        )

    def health(self) -> TrackingHealth:
        with self._lock:
            return self._health_unlocked()

    def _health_unlocked(self) -> TrackingHealth:
        rate = None
        if (
            self._imu_count >= 2
            and self._imu_first_ns is not None
            and self._imu_last_ns is not None
            and self._imu_last_ns > self._imu_first_ns
        ):
            span = (self._imu_last_ns - self._imu_first_ns) / 1e9
            if span > 0:
                rate = (self._imu_count - 1) / span
        status = "ok" if self._origin.origin.set else "waiting_origin"
        reason = "tracking active" if self._origin.origin.set else "awaiting arm for origin"
        return TrackingHealth(
            status=status,
            reason=reason,
            imu_sample_count=self._imu_count,
            imu_rate_hz=rate,
            vision_correction_count=self._vision_corrections,
            origin_set=self._origin.origin.set,
            armed=self._armed,
        )

    def latest_state(self) -> TrackingState | None:
        with self._lock:
            if not self._latest_sim_time_ns:
                return None
            return self._history.nearest(self._latest_sim_time_ns)

    def latest_snapshot(self) -> TrackingSnapshot:
        with self._lock:
            health = self._health_unlocked()
            sim_ns = self._latest_sim_time_ns
            state = self._history.nearest(sim_ns) if sim_ns else None
            if state is None:
                state = TrackingState(
                    sim_time_ns=0,
                    t_s=0.0,
                    position_ned=(0.0, 0.0, 0.0),
                    velocity_ned=(0.0, 0.0, 0.0),
                    attitude_rpy=(0.0, 0.0, 0.0),
                    armed=self._armed,
                    origin_set=self._origin.origin.set,
                )
            img = self._latest_image.copy() if self._latest_image is not None else None
            return TrackingSnapshot(
                sim_time_ns=state.sim_time_ns,
                image_rgb=img,
                position_ned=state.position_ned,
                velocity_ned=state.velocity_ned,
                attitude_rpy=state.attitude_rpy,
                health=health,
                state=state,
            )

    def flush(self) -> None:
        with self._lock:
            if self._csv_flushed or not self._config.log_csv:
                return
            self._csv_flushed = True
            rows = list(self._rows)
        self._config.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self._config.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(TRACKING_CSV_HEADER)
            writer.writerows(rows)
