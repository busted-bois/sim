"""MAVLink SET_POSITION_TARGET_LOCAL_NED command builders."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from src.control.flight_client import (
    SET_POSITION_FRAME_LOCAL_NED,
    SetAttitudeTargetCommand,
    SetPositionTargetLocalNedCommand,
    build_attitude_only_type_mask,
    build_position_target_type_mask,
)

if TYPE_CHECKING:
    from src.control.flight_client import FlightClient


def euler_to_quaternion_wxyz(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> tuple[float, float, float, float]:
    cr = math.cos(roll_rad / 2.0)
    sr = math.sin(roll_rad / 2.0)
    cp = math.cos(pitch_rad / 2.0)
    sp = math.sin(pitch_rad / 2.0)
    cy = math.cos(yaw_rad / 2.0)
    sy = math.sin(yaw_rad / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def attitude_target_command(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    thrust: float,
    *,
    throttle_body_z: bool = False,
) -> SetAttitudeTargetCommand:
    return SetAttitudeTargetCommand(
        type_mask=build_attitude_only_type_mask(throttle_body_z=throttle_body_z),
        quaternion=euler_to_quaternion_wxyz(roll_rad, pitch_rad, yaw_rad),
        thrust=thrust,
    )


def velocity_ned_command(
    vx: float,
    vy: float,
    vz: float,
    *,
    yaw_rate_rad_s: float | None = None,
    force_set: bool = True,
) -> SetPositionTargetLocalNedCommand:
    """Velocity setpoint in LOCAL_NED with optional yaw rate (rad/s)."""
    use_yaw_rate = yaw_rate_rad_s is not None
    type_mask = build_position_target_type_mask(
        use_velocity=True,
        use_yaw_rate=use_yaw_rate,
        force_set=force_set,
    )
    return SetPositionTargetLocalNedCommand(
        frame=SET_POSITION_FRAME_LOCAL_NED,
        type_mask=type_mask,
        vx=vx,
        vy=vy,
        vz=vz,
        yaw_rate=float(yaw_rate_rad_s or 0.0),
    )


def velocity_ned_with_yaw_rate_dps(
    vx: float,
    vy: float,
    vz: float,
    yaw_rate_dps: float,
    *,
    force_set: bool = True,
) -> SetPositionTargetLocalNedCommand:
    return velocity_ned_command(
        vx,
        vy,
        vz,
        yaw_rate_rad_s=math.radians(yaw_rate_dps),
        force_set=force_set,
    )


def apply_velocity_ned(
    client: FlightClient,
    vx: float,
    vy: float,
    vz: float,
    duration_s: float,
    *,
    yaw_rate_dps: float = 0.0,
) -> None:
    if abs(yaw_rate_dps) > 1e-6:
        cmd = velocity_ned_with_yaw_rate_dps(vx, vy, vz, yaw_rate_dps)
    else:
        cmd = velocity_ned_command(vx, vy, vz)
    client.streamSetPositionTargetLocalNedAsync(cmd, duration_s).join()
