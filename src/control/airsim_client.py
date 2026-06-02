"""AirSim RPC flight client implementing the FlightClient surface for SimpleFlight."""

from __future__ import annotations

from typing import Any

from src.control.command_rate import CommandRateGateStats
from src.control.highres_imu import HighresImuHealth, HighresImuSample
from src.position_trace import PositionTraceSnapshot


class AirSimFlightClient:
    """Thin wrapper around airsim.MultirotorClient for autonomous sim runs."""

    def __init__(self, host: str, port: int, *, timeout_value: float = 30.0) -> None:
        import airsim

        self._host = host
        self._port = port
        self._client = airsim.MultirotorClient(ip=host, port=port, timeout_value=timeout_value)

    def confirmConnection(self) -> None:
        self._client.confirmConnection()

    def enableApiControl(self, enable: bool) -> None:
        self._client.enableApiControl(enable)

    def armDisarm(self, arm: bool) -> None:
        self._client.armDisarm(arm)

    def takeoffAsync(self) -> Any:
        return self._client.takeoffAsync()

    def landAsync(self) -> Any:
        return self._client.landAsync()

    def goHomeAsync(self) -> Any:
        return self._client.goHomeAsync()

    def getMultirotorState(self) -> Any:
        return self._client.getMultirotorState()

    def cancelLastTask(self) -> None:
        self._client.cancelLastTask()

    def reset(self) -> None:
        self._client.reset()

    def moveByVelocityAsync(
        self, vx: float, vy: float, vz: float, duration: float, **kwargs
    ) -> Any:
        return self._client.moveByVelocityAsync(vx, vy, vz, duration, **kwargs)

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float) -> Any:
        return self._client.moveByVelocityZAsync(vx, vy, z, duration)

    def hoverAsync(self) -> Any:
        return self._client.hoverAsync()

    def rotateByYawRateAsync(self, yaw_rate: float, duration: float) -> Any:
        return self._client.rotateByYawRateAsync(yaw_rate, duration)

    def moveByRollPitchYawThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> Any:
        return self._client.moveByRollPitchYawThrottleAsync(roll, pitch, yaw, throttle, duration)

    def moveByAngleThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> Any:
        return self._client.moveByAngleThrottleAsync(roll, pitch, yaw, throttle, duration)

    def moveByAngleRateThrottleAsync(
        self, roll_rate: float, pitch_rate: float, yaw_rate: float, throttle: float, duration: float
    ) -> Any:
        return self._client.moveByAngleRateThrottleAsync(
            roll_rate, pitch_rate, yaw_rate, throttle, duration
        )

    def simSetCameraPose(self, camera_name: str, pose: Any) -> None:
        self._client.simSetCameraPose(camera_name, pose)

    def simSetVehiclePose(self, pose: Any, ignore_collision: bool = True) -> None:
        self._client.simSetVehiclePose(pose, ignore_collision)

    def simGetImages(self, requests: list[Any]) -> list[Any]:
        return self._client.simGetImages(requests)

    def simSetTraceLine(
        self, color: list[float], thickness: float, vehicle_name: str = ""
    ) -> None:
        self._client.simSetTraceLine(color, thickness, vehicle_name)

    def getCommandRateStats(self) -> CommandRateGateStats | None:
        return None

    def getHighresImu(self) -> HighresImuSample | None:
        return None

    def getHighresImuHealth(self) -> HighresImuHealth | None:
        return None

    def getPositionTraceSnapshot(self) -> PositionTraceSnapshot | None:
        return None

    def submitSetPositionTargetLocalNed(self, command: Any) -> None:
        raise NotImplementedError("SET_POSITION_TARGET requires MAVLink transport.")

    def submitVelocityLocalNed(self, vx: float, vy: float, vz: float) -> None:
        raise NotImplementedError("Velocity targets require MAVLink transport.")

    def submitVelocityBodyNed(self, vx: float, vy: float, vz: float) -> None:
        raise NotImplementedError("Velocity targets require MAVLink transport.")

    def submitPositionLocalNed(self, x: float, y: float, z: float) -> None:
        raise NotImplementedError("Position targets require MAVLink transport.")

    def streamSetPositionTargetLocalNedAsync(self, command: Any, duration: float) -> Any:
        raise NotImplementedError("SET_POSITION_TARGET requires MAVLink transport.")

    def submitSetAttitudeTarget(self, command: Any) -> None:
        raise NotImplementedError("SET_ATTITUDE_TARGET requires MAVLink transport.")

    def streamSetAttitudeTargetAsync(self, command: Any, duration: float) -> Any:
        raise NotImplementedError("SET_ATTITUDE_TARGET requires MAVLink transport.")

    def close(self) -> None:
        pass
