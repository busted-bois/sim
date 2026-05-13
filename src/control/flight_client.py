"""FlightClient protocol and AirSim adapter."""

from __future__ import annotations

import time
from typing import Any, Protocol

from src.control.command_rate import CommandRateGate, SkippedAsyncResult
from src.control.highres_imu import HighresImuHealth, HighresImuSample, SensorSnapshot


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
    def getHighresImu(self) -> HighresImuSample | None: ...
    def getHighresImuHealth(self) -> HighresImuHealth | None: ...
    def getHighresImuBySensorId(self, sensor_id: int) -> HighresImuSample | None: ...
    def getHighresImuSensors(self) -> dict[int, HighresImuSample]: ...
    def getSensorSnapshot(self) -> SensorSnapshot: ...


class AirSimAdapter:
    def __init__(self, client: Any, *, command_rate_hz: float | None = None) -> None:
        self._client = client
        self._command_rate_gate = (
            None if command_rate_hz is None else CommandRateGate(command_rate_hz)
        )

    def _motion_command_allowed(self) -> bool:
        if self._command_rate_gate is None:
            return True
        return self._command_rate_gate.allow()

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

    def _current_highres_imu_sample(self) -> HighresImuSample | None:
        imu = self._client.getImuData()
        mag = self._client.getMagnetometerData()
        baro = self._client.getBarometerData()
        received_ns = time.monotonic_ns()
        return HighresImuSample(
            time_usec=int(getattr(imu, "time_stamp", 0)),
            xacc=float(imu.linear_acceleration.x_val),
            yacc=float(imu.linear_acceleration.y_val),
            zacc=float(imu.linear_acceleration.z_val),
            xgyro=float(imu.angular_velocity.x_val),
            ygyro=float(imu.angular_velocity.y_val),
            zgyro=float(imu.angular_velocity.z_val),
            xmag=float(mag.magnetic_field_body.x_val),
            ymag=float(mag.magnetic_field_body.y_val),
            zmag=float(mag.magnetic_field_body.z_val),
            abs_pressure=float(getattr(baro, "pressure", 0.0)),
            diff_pressure=None,
            pressure_alt=float(getattr(baro, "altitude", 0.0)),
            temperature=None,
            fields_updated=0,
            sensor_id=0,
            source_system=None,
            source_component=None,
            local_received_monotonic_ns=received_ns,
            transport="airsim",
        )

    def getHighresImu(self) -> HighresImuSample | None:
        return self._current_highres_imu_sample()

    def getHighresImuHealth(self) -> HighresImuHealth | None:
        try:
            sample = self._current_highres_imu_sample()
        except Exception as exc:
            return HighresImuHealth(
                status="error",
                reason=f"AirSim IMU fetch failed: {exc}",
                enabled=True,
                sample_count=0,
                stream_rate_hz=None,
                update_age_ms=None,
                max_staleness_ms=1000.0,
                sensor_count=0,
                active_sensor_ids=(),
            )
        if sample is None:
            return HighresImuHealth(
                status="missing",
                reason="AirSim IMU fetch returned no sample",
                enabled=True,
                sample_count=0,
                stream_rate_hz=None,
                update_age_ms=None,
                max_staleness_ms=1000.0,
                sensor_count=0,
                active_sensor_ids=(),
            )
        return HighresImuHealth(
            status="ok",
            reason="AirSim IMU RPC fetch available",
            enabled=True,
            sample_count=1,
            stream_rate_hz=None,
            update_age_ms=0.0,
            max_staleness_ms=1000.0,
            sensor_count=1,
            active_sensor_ids=(sample.sensor_id,),
        )

    def getHighresImuBySensorId(self, sensor_id: int) -> HighresImuSample | None:
        sample = self._current_highres_imu_sample()
        if sample is None or sample.sensor_id != int(sensor_id):
            return None
        return sample

    def getHighresImuSensors(self) -> dict[int, HighresImuSample]:
        sample = self._current_highres_imu_sample()
        if sample is None:
            return {}
        return {sample.sensor_id: sample}

    def getSensorSnapshot(self) -> SensorSnapshot:
        captured_ns = time.monotonic_ns()
        sample = self._current_highres_imu_sample()
        return SensorSnapshot(
            state=self.getMultirotorState(),
            highres_imu=sample,
            highres_imu_health=self.getHighresImuHealth(),
            captured_monotonic_ns=captured_ns,
            transport="airsim",
        )
