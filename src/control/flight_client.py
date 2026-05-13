"""FlightClient protocol and AirSim adapter."""

from __future__ import annotations

from typing import Any, Protocol


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
    def __init__(self, client: Any) -> None:
        self._client = client

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
        return self._client.moveByVelocityAsync(vx, vy, vz, duration, **kwargs)

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float) -> Any:
        return self._client.moveByVelocityZAsync(vx, vy, z, duration)

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

    def moveByRollPitchYawThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> Any:
        return self._client.moveByRollPitchYawThrottleAsync(
            roll, pitch, yaw, throttle, duration
        )

    def rotateByYawRateAsync(self, yaw_rate: float, duration: float) -> Any:
        return self._client.rotateByYawRateAsync(yaw_rate, duration)

    def hoverAsync(self) -> Any:
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
