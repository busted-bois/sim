"""FlightClient Protocol and AirSimAdapter for drone control abstraction."""

from __future__ import annotations

from typing import Any, Protocol


class FlightClient(Protocol):
    """Protocol for drone flight control operations.

    Defines the interface for controlling a drone, enabling algorithms to be
    agnostic to the underlying RPC client implementation. The AirSimAdapter
    provides the concrete implementation for AirSim's MultirotorClient.
    """

    def enableApiControl(self, enable: bool) -> None:
        """Enable or disable API control of the vehicle."""

    def armDisarm(self, arm: bool) -> None:
        """Arm or disarm the vehicle motors."""

    def takeoffAsync(self) -> Any:
        """Initiate takeoff. Returns a Future that completes when airborne."""

    def landAsync(self) -> Any:
        """Initiate landing. Returns a Future that completes when landed."""

    def goHomeAsync(self) -> Any:
        """Return vehicle to home position. Returns a Future."""

    def getMultirotorState(self) -> Any:
        """Get current multirotor state (position, velocity, orientation, etc.)."""

    def cancelLastTask(self) -> None:
        """Cancel the last executed task."""

    def reset(self) -> None:
        """Reset the simulation vehicle to initial state."""

    def moveByVelocityAsync(
        self, vx: float, vy: float, vz: float, duration: float, **kwargs
    ) -> Any:
        """Move by world-frame velocity for duration. Returns a Future."""

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float) -> Any:
        """Move by velocity while maintaining altitude. Returns a Future."""

    def moveByAngleThrottleAsync(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        throttle: float,
        duration: float,
    ) -> Any:
        """Move by attitude and throttle. Returns a Future."""

    def moveByAngleRateThrottleAsync(
        self,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
        throttle: float,
        duration: float,
    ) -> Any:
        """Move by angular rates and throttle. Returns a Future."""

    def moveByRollPitchYawThrottleAsync(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        throttle: float,
        duration: float,
    ) -> Any:
        """Move by roll, pitch, yaw, and throttle. Returns a Future."""

    def rotateByYawRateAsync(self, yaw_rate: float, duration: float) -> Any:
        """Rotate by yaw rate for duration. Returns a Future."""

    def hoverAsync(self) -> Any:
        """Hold current position. Returns a Future."""

    def simSetCameraPose(self, camera_name: str, pose: Any) -> None:
        """Set camera pose in simulation."""

    def simSetVehiclePose(self, pose: Any, ignore_collision: bool = True) -> None:
        """Set vehicle pose in simulation."""

    def simGetImages(self, requests: list[Any]) -> list[Any]:
        """Get images from simulation cameras."""

    def simSetTraceLine(self, color: list[float], thickness: float, vehicle_name: str = "") -> None:
        """Set trace line style for vehicle trajectory."""

    def confirmConnection(self) -> None:
        """Confirm connection to the RPC server."""


class AirSimAdapter:
    """Adapter wrapping AirSim's MultirotorClient to implement FlightClient.

    This adapter delegates all FlightClient protocol methods to the underlying
    AirSim client, preserving exception types and behavior without wrapping.
    """

    def __init__(self, client: Any) -> None:
        """Initialize adapter with an AirSim MultirotorClient instance."""
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
        self,
        roll: float,
        pitch: float,
        yaw: float,
        throttle: float,
        duration: float,
    ) -> Any:
        return self._client.moveByAngleThrottleAsync(
            roll,
            pitch,
            yaw,
            throttle,
            duration,
        )

    def moveByAngleRateThrottleAsync(
        self,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
        throttle: float,
        duration: float,
    ) -> Any:
        return self._client.moveByAngleRateThrottleAsync(
            roll_rate,
            pitch_rate,
            yaw_rate,
            throttle,
            duration,
        )

    def moveByRollPitchYawThrottleAsync(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        throttle: float,
        duration: float,
    ) -> Any:
        return self._client.moveByRollPitchYawThrottleAsync(
            roll,
            pitch,
            yaw,
            throttle,
            duration,
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

    def simSetTraceLine(self, color: list[float], thickness: float, vehicle_name: str = "") -> None:
        return self._client.simSetTraceLine(color, thickness, vehicle_name)

    def confirmConnection(self) -> None:
        return self._client.confirmConnection()
