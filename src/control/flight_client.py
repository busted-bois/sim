"""FlightClient protocol and AirSim adapter. Units match AirSim Python: NED m/s, rpy and angle
rates in rad and rad/s, rotateByYawRateAsync in deg/s, throttle 0..1."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Literal, Protocol

from src.control.command_rate import CommandRateGate, CommandRateGateStats, SkippedAsyncResult
from src.control.highres_imu import HighresImuHealth, HighresImuSample

SetPositionTargetFrame = Literal["local_ned", "body_ned"]
SET_POSITION_FRAME_LOCAL_NED: SetPositionTargetFrame = "local_ned"
SET_POSITION_FRAME_BODY_NED: SetPositionTargetFrame = "body_ned"


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
    def reset(self) -> None: ...

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
    def simSetCameraPose(self, camera_name: str, pose: Any) -> None: ...
    def simSetVehiclePose(self, pose: Any, ignore_collision: bool = True) -> None: ...
    def simGetImages(self, requests: list[Any]) -> list[Any]: ...

    def simSetTraceLine(
        self, color: list[float], thickness: float, vehicle_name: str = ""
    ) -> None: ...

    def confirmConnection(self) -> None: ...
    def ping(self) -> bool: ...


class AirSimAdapter:
    def __init__(self, client: Any, *, command_rate_hz: float | None = None) -> None:
        self._client = client
        self._command_rate_gate = (
            None
            if command_rate_hz is None
            else CommandRateGate(command_rate_hz, label="AirSim motion commands")
        )

    def _motion_command_allowed(self) -> bool:
        if self._command_rate_gate is None:
            return True
        return self._command_rate_gate.allow()

    def _forward_optional(self, method: str, *args: Any) -> Any:
        fn = getattr(self._client, method, None)
        if not callable(fn):
            raise NotImplementedError(f"{method} is unavailable for AirSim")
        return fn(*args)

    def enableApiControl(self, enable: bool) -> None:
        return self._client.enableApiControl(enable)

    def armDisarm(self, arm: bool) -> None:
        return self._client.armDisarm(arm)

    def takeoffAsync(self) -> Any:
        return self._client.takeoffAsync()

    def landAsync(self) -> Any:
        return self._client.landAsync()

    def goHomeAsync(self) -> Any:
        return self._client.goHomeAsync()

    def getMultirotorState(self) -> Any:
        return self._client.getMultirotorState()

    def cancelLastTask(self) -> None:
        return self._client.cancelLastTask()

    def reset(self) -> None:
        return self._client.reset()

    def moveByVelocityAsync(
        self, vx: float, vy: float, vz: float, duration: float, **kwargs
    ) -> Any:
        if not self._motion_command_allowed():
            return SkippedAsyncResult()
        return self._client.moveByVelocityAsync(vx, vy, vz, duration, **kwargs)

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float) -> Any:
        if not self._motion_command_allowed():
            return SkippedAsyncResult()
        return self._client.moveByVelocityZAsync(vx, vy, z, duration)

    def submitSetPositionTargetLocalNed(self, command: SetPositionTargetLocalNedCommand) -> None:
        self._forward_optional("submitSetPositionTargetLocalNed", command)

    def submitVelocityLocalNed(self, vx: float, vy: float, vz: float) -> None:
        self._forward_optional("submitVelocityLocalNed", vx, vy, vz)

    def submitVelocityBodyNed(self, vx: float, vy: float, vz: float) -> None:
        self._forward_optional("submitVelocityBodyNed", vx, vy, vz)

    def submitPositionLocalNed(self, x: float, y: float, z: float) -> None:
        self._forward_optional("submitPositionLocalNed", x, y, z)

    def streamSetPositionTargetLocalNedAsync(
        self, command: SetPositionTargetLocalNedCommand, duration: float
    ) -> Any:
        return self._forward_optional("streamSetPositionTargetLocalNedAsync", command, duration)

    def moveByAngleThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> Any:
        if not self._motion_command_allowed():
            return SkippedAsyncResult()
        return self._client.moveByAngleThrottleAsync(roll, pitch, yaw, throttle, duration)

    def moveByAngleRateThrottleAsync(
        self, roll_rate: float, pitch_rate: float, yaw_rate: float, throttle: float, duration: float
    ) -> Any:
        if not self._motion_command_allowed():
            return SkippedAsyncResult()
        return self._client.moveByAngleRateThrottleAsync(
            roll_rate, pitch_rate, yaw_rate, throttle, duration
        )

    def moveByRollPitchYawThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> Any:
        if not self._motion_command_allowed():
            return SkippedAsyncResult()
        return self._client.moveByRollPitchYawThrottleAsync(
            roll, pitch, yaw, throttle, duration
        )

    def rotateByYawRateAsync(self, yaw_rate: float, duration: float) -> Any:
        if not self._motion_command_allowed():
            return SkippedAsyncResult()
        return self._client.rotateByYawRateAsync(yaw_rate, duration)

    def hoverAsync(self) -> Any:
        if not self._motion_command_allowed():
            return SkippedAsyncResult()
        return self._client.hoverAsync()

    def simSetCameraPose(self, camera_name: str, pose: Any) -> None:
        return self._client.simSetCameraPose(camera_name, pose)

    def simSetVehiclePose(self, pose: Any, ignore_collision: bool = True) -> None:
        return self._client.simSetVehiclePose(pose, ignore_collision)

    def simGetImages(self, requests: list[Any]) -> list[Any]:
        return self._client.simGetImages(requests)

    def simSetTraceLine(
        self, color: list[float], thickness: float, vehicle_name: str = ""
    ) -> None:
        return self._client.simSetTraceLine(color, thickness, vehicle_name)

    def confirmConnection(self) -> None:
        return self._client.confirmConnection()

    def ping(self) -> bool:
        return bool(self._client.ping())
