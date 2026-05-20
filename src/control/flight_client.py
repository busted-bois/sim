"""FlightClient protocol and MAVLink setpoint command types.

Motion units: NED m/s for velocity; radians for roll/pitch/yaw;
throttle in [0, 1]. rotateByYawRateAsync uses deg/s.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal, Protocol

from src.control.command_rate import CommandRateGateStats
from src.control.highres_imu import HighresImuHealth, HighresImuSample

SetPositionTargetFrame = Literal["local_ned", "body_ned"]
SET_POSITION_FRAME_LOCAL_NED: SetPositionTargetFrame = "local_ned"
SET_POSITION_FRAME_BODY_NED: SetPositionTargetFrame = "body_ned"


@dataclass(frozen=True, slots=True)
class SetAttitudeTargetCommand:
    type_mask: int
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    body_roll_rate: float = 0.0
    body_pitch_rate: float = 0.0
    body_yaw_rate: float = 0.0
    thrust: float = 0.5


def build_attitude_target_type_mask(
    *,
    ignore_attitude: bool = False,
    ignore_roll_rate: bool = True,
    ignore_pitch_rate: bool = True,
    ignore_yaw_rate: bool = True,
    ignore_thrust: bool = False,
    throttle_body_z: bool = False,
) -> int:
    from pymavlink import mavutil

    mask = 0
    if ignore_attitude:
        mask |= int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE)
    if ignore_roll_rate:
        mask |= int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE)
    if ignore_pitch_rate:
        mask |= int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE)
    if ignore_yaw_rate:
        mask |= int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE)
    if ignore_thrust:
        mask |= int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE)
    if throttle_body_z:
        mask |= int(getattr(mavutil.mavlink, "ATTITUDE_TARGET_TYPEMASK_THROTTLE_BODY_SET", 32))
    return int(mask)


def build_attitude_only_type_mask(*, throttle_body_z: bool = False) -> int:
    return build_attitude_target_type_mask(throttle_body_z=throttle_body_z)


def build_body_rate_type_mask(*, throttle_body_z: bool = False) -> int:
    return build_attitude_target_type_mask(
        ignore_attitude=True,
        ignore_roll_rate=False,
        ignore_pitch_rate=False,
        ignore_yaw_rate=False,
        throttle_body_z=throttle_body_z,
    )


@dataclass(frozen=True, slots=True)
class SetPositionTargetLocalNedCommand:
    frame: SetPositionTargetFrame
    type_mask: int
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    afx: float = 0.0
    afy: float = 0.0
    afz: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0


def build_position_target_type_mask(
    *,
    use_position: bool = False,
    use_velocity: bool = False,
    use_acceleration: bool = False,
    use_yaw: bool = False,
    use_yaw_rate: bool = False,
    force_set: bool = False,
) -> int:
    from pymavlink import mavutil

    mask = 0
    if not use_position:
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE)
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE)
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE)
    if not use_velocity:
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE)
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE)
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE)
    if not use_acceleration:
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE)
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE)
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
    if not use_yaw:
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
    if not use_yaw_rate:
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    if force_set:
        mask |= int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET)
    return int(mask)


def build_velocity_type_mask(*, force_set: bool = True) -> int:
    return build_position_target_type_mask(
        use_velocity=True,
        force_set=force_set,
    )


def build_position_type_mask() -> int:
    return build_position_target_type_mask(use_position=True)


class FlightClient(Protocol):

    def enableApiControl(self, enable: bool) -> None: ...
    def armDisarm(self, arm: bool) -> None: ...
    def takeoffAsync(self) -> Any: ...
    def landAsync(self) -> Any: ...
    def goHomeAsync(self) -> Any: ...
    def getMultirotorState(self) -> Any: ...
    def cancelLastTask(self) -> None: ...

    def moveByVelocityAsync(
        self, vx: float, vy: float, vz: float, duration: float, **kwargs
    ) -> Any: ...

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float) -> Any: ...
    def submitSetPositionTargetLocalNed(
        self, command: SetPositionTargetLocalNedCommand
    ) -> None: ...

    def submitVelocityLocalNed(self, vx: float, vy: float, vz: float) -> None: ...
    def submitVelocityBodyNed(self, vx: float, vy: float, vz: float) -> None: ...
    def submitPositionLocalNed(self, x: float, y: float, z: float) -> None: ...
    def streamSetPositionTargetLocalNedAsync(
        self, command: SetPositionTargetLocalNedCommand, duration: float
    ) -> Any: ...

    def submitSetAttitudeTarget(self, command: SetAttitudeTargetCommand) -> None: ...

    def streamSetAttitudeTargetAsync(
        self, command: SetAttitudeTargetCommand, duration: float
    ) -> Any: ...

    def moveByAngleThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> Any: ...

    def moveByAngleRateThrottleAsync(
        self, roll_rate: float, pitch_rate: float, yaw_rate: float, throttle: float, duration: float
    ) -> Any: ...

    def moveByRollPitchYawThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> Any: ...

    def rotateByYawRateAsync(self, yaw_rate: float, duration: float) -> Any: ...
    def hoverAsync(self) -> Any: ...
    def simSetTraceLine(
        self, color: list[float], thickness: float, vehicle_name: str = ""
    ) -> None: ...

    def confirmConnection(self) -> None: ...
    def getCommandRateStats(self) -> CommandRateGateStats | None: ...
    def getHighresImu(self) -> HighresImuSample | None: ...
    def getHighresImuHealth(self) -> HighresImuHealth | None: ...
