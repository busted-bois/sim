"""MAVLink NED frame mapping: LOCAL_NED (arm origin) and BODY_NED (vehicle)."""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from enum import IntEnum
from pathlib import Path
from typing import Any

# MAV_FRAME_LOCAL_NED / MAV_FRAME_BODY_NED per mavlink.io common messages.
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_NED = 8


class MavlinkNedFrame(IntEnum):
    LOCAL_NED = MAV_FRAME_LOCAL_NED
    BODY_NED = MAV_FRAME_BODY_NED


class NedTransformError(RuntimeError):
    """Raised when a body/local transform requires attitude that is not available."""


@dataclass(frozen=True, slots=True)
class NedVector3:
    x: float
    y: float
    z: float


@dataclass(frozen=True, slots=True)
class NedVelocity:
    vx: float
    vy: float
    vz: float


@dataclass(frozen=True, slots=True)
class NedAttitude:
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True, slots=True)
class VehicleNedSnapshot:
    position: NedVector3
    velocity: NedVelocity
    attitude: NedAttitude | None
    position_time_boot_ms: int | None
    attitude_time_boot_ms: int | None
    has_position: bool
    has_attitude: bool
    spawn_relative_xy: tuple[float, float] | None
    transform_ready: bool


@dataclass(frozen=True, slots=True)
class NedEnvironmentSettings:
    enabled: bool = True
    use_full_attitude: bool = False
    spawn_relative_enabled: bool = True
    max_position_staleness_ms: float = 500.0
    max_attitude_staleness_ms: float = 500.0


@dataclass(frozen=True, slots=True)
class NedEnvironmentHealth:
    status: str
    reason: str
    has_position: bool
    has_attitude: bool
    transform_ready: bool
    position_age_ms: float | None
    attitude_age_ms: float | None

    def is_healthy(self) -> bool:
        return self.status == "ok"

    def is_stale(self) -> bool:
        return self.status == "stale"


@dataclass(frozen=True, slots=True)
class BodyVelocitySetpoint:
    """Body-frame command with precomputed LOCAL_NED equivalent."""

    body: NedVelocity
    local: NedVelocity


def parse_ned_environment_settings(raw: dict[str, Any] | None) -> NedEnvironmentSettings:
    cfg = raw or {}
    return NedEnvironmentSettings(
        enabled=bool(cfg.get("enabled", True)),
        use_full_attitude=bool(cfg.get("use_full_attitude", False)),
        spawn_relative_enabled=bool(cfg.get("spawn_relative_enabled", True)),
        max_position_staleness_ms=max(1.0, float(cfg.get("max_position_staleness_ms", 500.0))),
        max_attitude_staleness_ms=max(1.0, float(cfg.get("max_attitude_staleness_ms", 500.0))),
    )


def load_ned_environment_config(config: dict[str, Any]) -> NedEnvironmentSettings:
    control = config.get("control")
    if not isinstance(control, dict):
        return NedEnvironmentSettings()
    mavlink = control.get("mavlink")
    if not isinstance(mavlink, dict):
        return NedEnvironmentSettings()
    ned = mavlink.get("ned_environment")
    if not isinstance(ned, dict):
        return NedEnvironmentSettings()
    return parse_ned_environment_settings(ned)


def _rotate_yaw_only(vec: NedVector3, yaw_rad: float, *, inverse: bool) -> NedVector3:
    sign = -1.0 if inverse else 1.0
    psi = sign * yaw_rad
    c = math.cos(psi)
    s = math.sin(psi)
    x = vec.x * c - vec.y * s
    y = vec.x * s + vec.y * c
    return NedVector3(x, y, vec.z)


def _rotation_matrix_ned(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, ...], ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat_vec3(matrix: tuple[tuple[float, ...], ...], vec: NedVector3) -> NedVector3:
    return NedVector3(
        matrix[0][0] * vec.x + matrix[0][1] * vec.y + matrix[0][2] * vec.z,
        matrix[1][0] * vec.x + matrix[1][1] * vec.y + matrix[1][2] * vec.z,
        matrix[2][0] * vec.x + matrix[2][1] * vec.y + matrix[2][2] * vec.z,
    )


def _transpose3(matrix: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


class NedEnvironmentMap:
    """Canonical vehicle state in MAV_FRAME_LOCAL_NED with BODY_NED transforms."""

    def __init__(self, settings: NedEnvironmentSettings | None = None) -> None:
        self._settings = settings or NedEnvironmentSettings()
        self._lock = threading.Lock()
        self._position: NedVector3 | None = None
        self._velocity: NedVelocity | None = None
        self._attitude: NedAttitude | None = None
        self._position_time_boot_ms: int | None = None
        self._attitude_time_boot_ms: int | None = None
        self._position_monotonic_ns: int | None = None
        self._attitude_monotonic_ns: int | None = None
        self._spawn_origin: NedVector3 | None = None

    @property
    def settings(self) -> NedEnvironmentSettings:
        return self._settings

    @property
    def heading_yaw_rad(self) -> float | None:
        with self._lock:
            if self._attitude is None:
                return None
            return self._attitude.yaw

    def update_local_position_ned(
        self,
        *,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
        time_boot_ms: int | None = None,
    ) -> None:
        if not self._settings.enabled:
            return
        with self._lock:
            self._position = NedVector3(float(x), float(y), float(z))
            self._velocity = NedVelocity(float(vx), float(vy), float(vz))
            self._position_monotonic_ns = time.monotonic_ns()
            if time_boot_ms is not None:
                self._position_time_boot_ms = int(time_boot_ms)

    def update_attitude(
        self,
        *,
        roll: float,
        pitch: float,
        yaw: float,
        time_boot_ms: int | None = None,
    ) -> None:
        if not self._settings.enabled:
            return
        with self._lock:
            self._attitude = NedAttitude(float(roll), float(pitch), float(yaw))
            self._attitude_monotonic_ns = time.monotonic_ns()
            if time_boot_ms is not None:
                self._attitude_time_boot_ms = int(time_boot_ms)

    def set_spawn_origin(
        self,
        x_m: float,
        y_m: float,
        z_m: float | None = None,
    ) -> None:
        with self._lock:
            z = float(z_m) if z_m is not None else (self._position.z if self._position else 0.0)
            self._spawn_origin = NedVector3(float(x_m), float(y_m), z)

    def refresh_from_multirotor_state(self, state: Any) -> None:
        """Update pose from AirSim RPC without clearing spawn origin."""
        kin = state.kinematics_estimated
        pos = kin.position
        vel = kin.linear_velocity
        self.update_local_position_ned(
            x=float(pos.x_val),
            y=float(pos.y_val),
            z=float(pos.z_val),
            vx=float(vel.x_val),
            vy=float(vel.y_val),
            vz=float(vel.z_val),
        )
        orientation = getattr(kin, "orientation", None)
        if orientation is not None:
            from src.control.utils import rpy_from_orientation

            roll, pitch, yaw = rpy_from_orientation(orientation)
            self.update_attitude(roll=roll, pitch=pitch, yaw=yaw)

    def spawn_relative_xy(self) -> tuple[float, float] | None:
        return self.snapshot().spawn_relative_xy

    def _spawn_relative_xy_locked(self) -> tuple[float, float] | None:
        if not self._settings.spawn_relative_enabled:
            return None
        if self._spawn_origin is None or self._position is None:
            return None
        return (
            self._position.x - self._spawn_origin.x,
            self._position.y - self._spawn_origin.y,
        )

    def snapshot(self) -> VehicleNedSnapshot:
        with self._lock:
            zero = NedVector3(0.0, 0.0, 0.0)
            zero_vel = NedVelocity(0.0, 0.0, 0.0)
            has_attitude = self._attitude is not None
            return VehicleNedSnapshot(
                position=self._position or zero,
                velocity=self._velocity or zero_vel,
                attitude=self._attitude,
                position_time_boot_ms=self._position_time_boot_ms,
                attitude_time_boot_ms=self._attitude_time_boot_ms,
                has_position=self._position is not None,
                has_attitude=has_attitude,
                spawn_relative_xy=self._spawn_relative_xy_locked(),
                transform_ready=has_attitude,
            )

    def get_health(self, now_monotonic_ns: int | None = None) -> NedEnvironmentHealth:
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
        with self._lock:
            has_position = self._position is not None
            has_attitude = self._attitude is not None
            position_age_ms = None
            attitude_age_ms = None
            if self._position_monotonic_ns is not None:
                position_age_ms = max(0.0, (now_ns - self._position_monotonic_ns) / 1_000_000.0)
            if self._attitude_monotonic_ns is not None:
                attitude_age_ms = max(0.0, (now_ns - self._attitude_monotonic_ns) / 1_000_000.0)

        if not self._settings.enabled:
            return NedEnvironmentHealth(
                status="disabled",
                reason="NED environment mapping disabled in config",
                has_position=has_position,
                has_attitude=has_attitude,
                transform_ready=False,
                position_age_ms=position_age_ms,
                attitude_age_ms=attitude_age_ms,
            )
        if not has_position:
            return NedEnvironmentHealth(
                status="missing",
                reason="no LOCAL_POSITION_NED sample received yet",
                has_position=False,
                has_attitude=has_attitude,
                transform_ready=False,
                position_age_ms=None,
                attitude_age_ms=attitude_age_ms,
            )
        stale_parts: list[str] = []
        if (
            position_age_ms is not None
            and position_age_ms > self._settings.max_position_staleness_ms
        ):
            stale_parts.append(f"position age {position_age_ms:.0f}ms")
        if not has_attitude:
            stale_parts.append("no ATTITUDE sample")
        elif (
            attitude_age_ms is not None
            and attitude_age_ms > self._settings.max_attitude_staleness_ms
        ):
            stale_parts.append(f"attitude age {attitude_age_ms:.0f}ms")
        if stale_parts:
            return NedEnvironmentHealth(
                status="stale",
                reason="; ".join(stale_parts),
                has_position=True,
                has_attitude=has_attitude,
                transform_ready=has_attitude,
                position_age_ms=position_age_ms,
                attitude_age_ms=attitude_age_ms,
            )
        return NedEnvironmentHealth(
            status="ok",
            reason="LOCAL_NED pose and ATTITUDE are fresh",
            has_position=True,
            has_attitude=has_attitude,
            transform_ready=has_attitude,
            position_age_ms=position_age_ms,
            attitude_age_ms=attitude_age_ms,
        )

    def export_snapshot_json(self, path: str | Path) -> Path:
        snap = self.snapshot()
        health = self.get_health()
        payload = {
            "snapshot": asdict(snap),
            "health": asdict(health),
        }
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return out

    def _require_attitude(self) -> NedAttitude:
        with self._lock:
            if self._attitude is None:
                raise NedTransformError(
                    "BODY_NED transform requires ATTITUDE; no attitude sample received yet."
                )
            return self._attitude

    def transform_vector(
        self,
        vec: NedVector3,
        *,
        from_frame: MavlinkNedFrame,
        to_frame: MavlinkNedFrame,
    ) -> NedVector3:
        if from_frame == to_frame:
            return vec
        att = self._require_attitude()
        body_to_local = (
            from_frame == MavlinkNedFrame.BODY_NED and to_frame == MavlinkNedFrame.LOCAL_NED
        )
        local_to_body = (
            from_frame == MavlinkNedFrame.LOCAL_NED and to_frame == MavlinkNedFrame.BODY_NED
        )
        if not body_to_local and not local_to_body:
            raise NedTransformError(f"Unsupported frame transform {from_frame!r} -> {to_frame!r}")

        if self._settings.use_full_attitude:
            rot = _rotation_matrix_ned(att.roll, att.pitch, att.yaw)
            if local_to_body:
                rot = _transpose3(rot)
            return _mat_vec3(rot, vec)

        return _rotate_yaw_only(vec, att.yaw, inverse=local_to_body)

    def body_velocity_to_local(self, vx: float, vy: float, vz: float) -> NedVelocity:
        out = self.transform_vector(
            NedVector3(vx, vy, vz),
            from_frame=MavlinkNedFrame.BODY_NED,
            to_frame=MavlinkNedFrame.LOCAL_NED,
        )
        return NedVelocity(out.x, out.y, out.z)

    def local_velocity_to_body(self, vx: float, vy: float, vz: float) -> NedVelocity:
        out = self.transform_vector(
            NedVector3(vx, vy, vz),
            from_frame=MavlinkNedFrame.LOCAL_NED,
            to_frame=MavlinkNedFrame.BODY_NED,
        )
        return NedVelocity(out.x, out.y, out.z)

    def body_forward_velocity_ms(self, speed_ms: float, vz: float = 0.0) -> NedVelocity:
        return self.body_velocity_to_local(speed_ms, 0.0, vz)

    def body_offset_to_local(self, x: float, y: float, z: float) -> NedVector3:
        return self.transform_vector(
            NedVector3(x, y, z),
            from_frame=MavlinkNedFrame.BODY_NED,
            to_frame=MavlinkNedFrame.LOCAL_NED,
        )

    @classmethod
    def from_multirotor_state(
        cls,
        state: Any,
        *,
        settings: NedEnvironmentSettings | None = None,
        spawn_xy: tuple[float, float] | None = None,
        spawn_z: float | None = None,
    ) -> NedEnvironmentMap:
        """Build mapper snapshot from AirSim-style getMultirotorState() kinematics."""
        mapper = cls(settings)
        mapper.refresh_from_multirotor_state(state)
        if spawn_xy is not None:
            mapper.set_spawn_origin(spawn_xy[0], spawn_xy[1], spawn_z)
        return mapper


def plan_body_velocity(
    mapper: NedEnvironmentMap,
    vx_body: float,
    vy_body: float,
    vz_body: float,
) -> BodyVelocitySetpoint:
    """Map a BODY_NED velocity command to its LOCAL_NED equivalent."""
    local = mapper.body_velocity_to_local(vx_body, vy_body, vz_body)
    return BodyVelocitySetpoint(
        body=NedVelocity(vx_body, vy_body, vz_body),
        local=local,
    )


class MavlinkNedIngest:
    """Feed pymavlink LOCAL_POSITION_NED and ATTITUDE into NedEnvironmentMap."""

    def __init__(self, mapper: NedEnvironmentMap) -> None:
        self._mapper = mapper

    def on_local_position_ned(self, message: Any) -> None:
        self._mapper.update_local_position_ned(
            x=float(getattr(message, "x", 0.0)),
            y=float(getattr(message, "y", 0.0)),
            z=float(getattr(message, "z", 0.0)),
            vx=float(getattr(message, "vx", 0.0)),
            vy=float(getattr(message, "vy", 0.0)),
            vz=float(getattr(message, "vz", 0.0)),
            time_boot_ms=int(getattr(message, "time_boot_ms", 0)),
        )

    def on_attitude(self, message: Any) -> None:
        self._mapper.update_attitude(
            roll=float(getattr(message, "roll", 0.0)),
            pitch=float(getattr(message, "pitch", 0.0)),
            yaw=float(getattr(message, "yaw", 0.0)),
            time_boot_ms=int(getattr(message, "time_boot_ms", 0)),
        )
