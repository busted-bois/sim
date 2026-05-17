from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Final

from pymavlink import mavutil

from src.control.command_rate import (
    CommandRateGate,
    CommandRateGateStats,
    normalize_command_rate_hz,
)
from src.control.flight_client import (
    SET_POSITION_FRAME_BODY_NED,
    SET_POSITION_FRAME_LOCAL_NED,
    SetPositionTargetLocalNedCommand,
    build_position_type_mask,
    build_velocity_type_mask,
)
from src.control.highres_imu import (
    HighresImuHealth,
    HighresImuSample,
    format_highres_imu_health,
    merge_highres_imu_sample,
)
from src.control.mavlink_timesync import TimesyncOutboundRequest, TimesyncSnapshot, TimesyncStore

_logger = logging.getLogger(__name__)


def _parse_udp_endpoint(endpoint: str) -> tuple[str, int]:
    text = endpoint.strip()
    prefixes = ("udp:", "udpin:")
    if not text.startswith(prefixes):
        raise ValueError(
            f"Unsupported MAVLink endpoint (expected udp:... or udpin:...): {endpoint!r}"
        )
    prefix = "udpin:" if text.startswith("udpin:") else "udp:"
    host_port = text.split(prefix, 1)[1]
    if host_port.count(":") != 1:
        raise ValueError(f"Invalid udp endpoint (expected udp:host:port): {endpoint!r}")
    host, port_s = host_port.split(":", 1)
    return host, int(port_s)


@dataclass(frozen=True, slots=True)
class _Vector3r:
    x_val: float
    y_val: float
    z_val: float


@dataclass(frozen=True, slots=True)
class _KinematicsEstimated:
    position: _Vector3r
    linear_velocity: _Vector3r


@dataclass(frozen=True, slots=True)
class _MultirotorState:
    kinematics_estimated: _KinematicsEstimated


@dataclass(frozen=True, slots=True)
class _Telemetry:
    state: _MultirotorState
    armed: bool


class _Joinable:
    def __init__(self, fn: Callable[[], None]) -> None:
        self._fn = fn
        self._ran = False
        self._exc: BaseException | None = None

    def join(self, timeout: float | None = None) -> None:
        _ = timeout
        if self._ran:
            if self._exc is not None:
                raise self._exc
            return

        self._ran = True
        try:
            self._fn()
        except BaseException as exc:
            self._exc = exc
            raise


class PymavlinkFlightClient:
    """MAVLink runtime client with TIMESYNC observability.

    Motion setpoints use NED Euler/sign conventions for SET_POSITION /
    SET_ATTITUDE_TARGET sends.
    """

    _DEFAULT_ENDPOINT: Final[str] = "udpin:0.0.0.0:14550"
    _GUIDED_MODE_MIN_RESEND_INTERVAL_S: Final[float] = 1.0
    _POSITION_TARGET_FRAME_MAP: Final[dict[str, int]] = {
        "local_ned": int(mavutil.mavlink.MAV_FRAME_LOCAL_NED),
        "body_ned": int(mavutil.mavlink.MAV_FRAME_BODY_NED),
    }

    def __init__(
        self,
        *,
        endpoint: str | None = None,
        command_rate_hz: float = 50.0,
        state_request_hz: float = 20.0,
        guided_custom_mode: int = 4,
        takeoff_altitude_m: float = 5.0,
        takeoff_climb_speed_ms: float = 1.0,
        land_descent_speed_ms: float = 0.6,
        takeoff_timeout_s: float = 15.0,
        land_timeout_s: float = 30.0,
        heartbeat_timeout_s: float = 5.0,
        source_system: int = 255,
        source_component: int = 1,
        respond_to_timesync_requests: bool = True,
        timesync_log_messages: bool = False,
        send_timesync_requests: bool = True,
        timesync_request_interval_s: float = 1.0,
        prepare_for_flight_on_connect: bool = True,
        request_state_messages_on_connect: bool = True,
        highres_imu_enabled: bool = True,
        highres_imu_request_hz: float | None = None,
        highres_imu_log_messages: bool = False,
        highres_imu_max_staleness_ms: float = 1000.0,
        timesync_pending_request_limit: int = 64,
        timesync_stable_window_size: int = 9,
        timesync_stable_best_subset_size: int = 5,
        timesync_min_stable_samples: int = 3,
        timesync_max_stable_rtt_ns: int = 250_000_000,
        timesync_max_offset_jitter_ns: int = 50_000_000,
        connection_factory: Callable[..., Any] | None = None,
        log_commands: bool = True,
    ) -> None:
        self._endpoint = endpoint.strip() if endpoint else self._DEFAULT_ENDPOINT
        _parse_udp_endpoint(self._endpoint)

        self._command_rate_hz = normalize_command_rate_hz(command_rate_hz)
        self._command_rate_gate = CommandRateGate(
            self._command_rate_hz,
            label="MAVLink outbound motion commands",
        )
        self._state_request_hz = state_request_hz
        self._guided_custom_mode = guided_custom_mode
        self._takeoff_altitude_m = takeoff_altitude_m
        self._takeoff_climb_speed_ms = takeoff_climb_speed_ms
        self._land_descent_speed_ms = land_descent_speed_ms
        self._takeoff_timeout_s = takeoff_timeout_s
        self._land_timeout_s = land_timeout_s
        self._heartbeat_timeout_s = heartbeat_timeout_s
        self._source_system = source_system
        self._source_component = source_component
        self._respond_to_timesync_requests = respond_to_timesync_requests
        self._timesync_log_messages = timesync_log_messages
        self._send_timesync_requests = send_timesync_requests
        self._timesync_request_interval_s = max(0.1, timesync_request_interval_s)
        self._prepare_for_flight_on_connect = prepare_for_flight_on_connect
        self._request_state_messages_on_connect = request_state_messages_on_connect
        self._highres_imu_enabled = highres_imu_enabled
        imu_request_hz = (
            highres_imu_request_hz if highres_imu_request_hz is not None else state_request_hz
        )
        self._highres_imu_request_hz = max(1.0, imu_request_hz)
        self._highres_imu_log_messages = highres_imu_log_messages
        self._highres_imu_max_staleness_ms = max(1.0, highres_imu_max_staleness_ms)
        self._connection_factory = connection_factory or mavutil.mavlink_connection
        self._log_commands = log_commands

        self._mav: Any | None = None
        self._target_system: int | None = None
        self._target_component: int | None = None

        self._telemetry_lock = threading.Lock()
        self._telemetry: _Telemetry | None = None
        self._highres_imu_lock = threading.Lock()
        self._highres_imu: HighresImuSample | None = None
        self._highres_imu_sample_count = 0
        self._highres_imu_first_monotonic_ns: int | None = None
        self._highres_imu_last_monotonic_ns: int | None = None
        self._timesync_store = TimesyncStore(
            local_system=self._source_system,
            local_component=self._source_component,
            pending_request_limit=timesync_pending_request_limit,
            stable_window_size=timesync_stable_window_size,
            stable_best_subset_size=timesync_stable_best_subset_size,
            min_stable_samples=timesync_min_stable_samples,
            max_stable_rtt_ns=timesync_max_stable_rtt_ns,
            max_offset_jitter_ns=timesync_max_offset_jitter_ns,
        )

        self._state_ready_evt = threading.Event()
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None
        self._guided_mode_last_sent_monotonic_s: float | None = None

    def confirmConnection(self) -> None:
        self._mav = self._connection_factory(
            self._endpoint,
            source_system=self._source_system,
            source_component=self._source_component,
            autoreconnect=True,
        )

        heartbeat = self._mav.wait_heartbeat(timeout=self._heartbeat_timeout_s)
        if heartbeat is None:
            raise TimeoutError(f"No MAVLink HEARTBEAT received on {self._endpoint}.")

        self._target_system = int(
            getattr(self._mav, "target_system", 0) or heartbeat.get_srcSystem() or 1
        )
        self._target_component = int(
            getattr(self._mav, "target_component", 0) or heartbeat.get_srcComponent() or 1
        )
        if self._target_system <= 0:
            raise TimeoutError(
                "No valid MAVLink heartbeat target detected "
                f"on {self._endpoint} (target_system={self._target_system})."
            )

        armed = (int(heartbeat.base_mode) & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)) != 0
        zero = _Vector3r(0.0, 0.0, 0.0)
        with self._telemetry_lock:
            self._telemetry = _Telemetry(
                state=_MultirotorState(
                    kinematics_estimated=_KinematicsEstimated(
                        position=zero,
                        linear_velocity=zero,
                    )
                ),
                armed=armed,
            )
        self._state_ready_evt.set()

        if self._request_state_messages_on_connect:
            self._request_message_intervals()
        self._start_telemetry_pump()
        if self._prepare_for_flight_on_connect:
            self._set_guided_mode(force=True)

    def enableApiControl(self, enable: bool) -> None:
        _ = enable

    def armDisarm(self, arm: bool) -> None:
        if self._log_commands:
            _logger.info("[MAVLink <<] %s", "ARM" if arm else "DISARM")
        assert self._mav is not None and self._target_system is not None
        self._mav.mav.command_long_send(
            self._target_system,
            self._target_component or 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        deadline = time.time() + 8.0
        while time.time() < deadline:
            telemetry = self._get_latest_telemetry()
            if telemetry is not None and telemetry.armed == arm:
                return
            time.sleep(0.05)

    def takeoffAsync(self) -> _Joinable:
        return _Joinable(self._takeoff)

    def landAsync(self) -> _Joinable:
        return _Joinable(self._land)

    def goHomeAsync(self) -> _Joinable:
        return _Joinable(self._go_home)

    def hoverAsync(self) -> _Joinable:
        if self._log_commands:
            _logger.info("[MAVLink <<] HOVER")
        return self.moveByVelocityAsync(0.0, 0.0, 0.0, 0.25)

    def getMultirotorState(self) -> _MultirotorState:
        telemetry = self._get_latest_telemetry()
        if telemetry is None:
            self._state_ready_evt.wait(timeout=self._heartbeat_timeout_s)
            telemetry = self._get_latest_telemetry()
            if telemetry is None:
                zero = _Vector3r(0.0, 0.0, 0.0)
                return _MultirotorState(
                    kinematics_estimated=_KinematicsEstimated(position=zero, linear_velocity=zero)
                )
        return telemetry.state

    def cancelLastTask(self) -> None:
        return

    def reset(self) -> None:
        return

    def moveByVelocityAsync(
        self, vx: float, vy: float, vz: float, duration: float, **kwargs
    ) -> _Joinable:
        _ = kwargs
        if self._log_commands:
            _logger.info(
                "[MAVLink <<] VELOCITY vx=%.2f vy=%.2f vz=%.2f dur=%.1fs",
                vx, vy, vz, duration,
            )
        return _Joinable(lambda: self._stream_velocity(vx, vy, vz, duration))

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float) -> _Joinable:
        if self._log_commands:
            _logger.info(
                "[MAVLink <<] VELOCITY_Z vx=%.2f vy=%.2f z=%.2f dur=%.1fs",
                vx, vy, z, duration,
            )

        def _run() -> None:
            current_z = float(self.getMultirotorState().kinematics_estimated.position.z_val)
            if duration <= 0:
                vz = 0.0
            else:
                vz = max(-2.5, min(2.5, (float(z) - current_z) / float(duration)))
            self._stream_velocity(vx, vy, vz, duration)

        return _Joinable(_run)

    def submitSetPositionTargetLocalNed(self, command: SetPositionTargetLocalNedCommand) -> None:
        if self._log_commands:
            _logger.info(
                "[MAVLink <<] POSITION_TARGET frame=%s mask=%s",
                command.frame, command.type_mask,
            )
        assert self._mav is not None and self._target_system is not None
        self._set_guided_mode(force=False)
        self._send_set_position_target_local_ned(command)

    def submitVelocityLocalNed(self, vx: float, vy: float, vz: float) -> None:
        if self._log_commands:
            _logger.info(
                "[MAVLink <<] VELOCITY_LOCAL_NED vx=%.2f vy=%.2f vz=%.2f", vx, vy, vz,
            )
        self.submitSetPositionTargetLocalNed(
            SetPositionTargetLocalNedCommand(
                frame=SET_POSITION_FRAME_LOCAL_NED,
                type_mask=build_velocity_type_mask(),
                vx=vx,
                vy=vy,
                vz=vz,
            )
        )

    def submitVelocityBodyNed(self, vx: float, vy: float, vz: float) -> None:
        if self._log_commands:
            _logger.info(
                "[MAVLink <<] VELOCITY_BODY_NED vx=%.2f vy=%.2f vz=%.2f", vx, vy, vz,
            )
        self.submitSetPositionTargetLocalNed(
            SetPositionTargetLocalNedCommand(
                frame=SET_POSITION_FRAME_BODY_NED,
                type_mask=build_velocity_type_mask(),
                vx=vx,
                vy=vy,
                vz=vz,
            )
        )

    def submitPositionLocalNed(self, x: float, y: float, z: float) -> None:
        if self._log_commands:
            _logger.info(
                "[MAVLink <<] POSITION_LOCAL_NED x=%.2f y=%.2f z=%.2f", x, y, z,
            )
        self.submitSetPositionTargetLocalNed(
            SetPositionTargetLocalNedCommand(
                frame=SET_POSITION_FRAME_LOCAL_NED,
                type_mask=build_position_type_mask(),
                x=x,
                y=y,
                z=z,
            )
        )

    def streamSetPositionTargetLocalNedAsync(
        self, command: SetPositionTargetLocalNedCommand, duration: float
    ) -> _Joinable:
        return _Joinable(lambda: self._stream_set_position_target_local_ned(command, duration))

    def submitSetAttitudeTarget(self, command: SetAttitudeTargetCommand) -> None:
        if self._log_commands:
            _logger.info(
                "[MAVLink <<] ATTITUDE_TARGET mask=%s thrust=%.3f",
                command.type_mask, command.thrust,
            )
        assert self._mav is not None and self._target_system is not None
        self._set_guided_mode(force=False)
        self._send_set_attitude_target(command)

    def streamSetAttitudeTargetAsync(
        self, command: SetAttitudeTargetCommand, duration: float
    ) -> _Joinable:
        return _Joinable(lambda: self._stream_set_attitude_target(command, duration))

    def moveByAngleThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> _Joinable:
        return self.moveByRollPitchYawThrottleAsync(roll, pitch, yaw, throttle, duration)

    def moveByAngleRateThrottleAsync(
        self,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
        throttle: float,
        duration: float,
    ) -> _Joinable:
        return _Joinable(
            lambda: self._stream_attitude_rate_target(
                roll_rate,
                pitch_rate,
                yaw_rate,
                throttle,
                duration,
            )
        )

    def moveByRollPitchYawThrottleAsync(
        self, roll: float, pitch: float, yaw: float, throttle: float, duration: float
    ) -> _Joinable:
        return _Joinable(
            lambda: self._stream_attitude_target(
                self._quaternion_from_euler(roll, pitch, yaw),
                throttle,
                duration,
            )
        )

    def rotateByYawRateAsync(self, yaw_rate: float, duration: float) -> _Joinable:
        if self._log_commands:
            _logger.info(
                "[MAVLink <<] YAW_RATE rate=%.1f deg/s dur=%.1fs", yaw_rate, duration,
            )
        yaw_rate_rad_s = math.radians(float(yaw_rate))
        return _Joinable(
            lambda: self._stream_attitude_rate_target(
                0.0,
                0.0,
                yaw_rate_rad_s,
                0.5,
                duration,
            )
        )

    def simSetCameraPose(self, camera_name: str, pose: Any) -> None:
        _ = camera_name, pose
        return

    def simSetVehiclePose(self, pose: Any, ignore_collision: bool = True) -> None:
        _ = pose, ignore_collision
        return

    def simGetImages(self, requests: list[Any]) -> list[Any]:
        _ = requests
        return []

    def simSetTraceLine(
        self, color: list[float], thickness: float, vehicle_name: str = ""
    ) -> None:
        _ = color, thickness, vehicle_name
        return

    def getTimesyncSnapshot(self) -> TimesyncSnapshot:
        return self._timesync_store.snapshot()

    def getHighresImu(self) -> HighresImuSample | None:
        with self._highres_imu_lock:
            return self._highres_imu

    def getCommandRateStats(self) -> CommandRateGateStats:
        return self._command_rate_gate.stats()

    def getHighresImuHealth(self) -> HighresImuHealth | None:
        with self._highres_imu_lock:
            sample = self._highres_imu
            sample_count = self._highres_imu_sample_count
            first_ns = self._highres_imu_first_monotonic_ns
            last_ns = self._highres_imu_last_monotonic_ns

        if not self._highres_imu_enabled:
            return HighresImuHealth(
                status="disabled",
                reason="HIGHRES_IMU stream disabled in config",
                enabled=False,
                sample_count=sample_count,
                stream_rate_hz=None,
                update_age_ms=None,
                max_staleness_ms=self._highres_imu_max_staleness_ms,
            )
        if sample is None or last_ns is None:
            return HighresImuHealth(
                status="missing",
                reason="no HIGHRES_IMU samples received yet",
                enabled=True,
                sample_count=sample_count,
                stream_rate_hz=None,
                update_age_ms=None,
                max_staleness_ms=self._highres_imu_max_staleness_ms,
            )

        now_ns = time.monotonic_ns()
        update_age_ms = (now_ns - last_ns) / 1_000_000.0
        stream_rate_hz = None
        if (
            sample_count >= 2
            and first_ns is not None
            and last_ns > first_ns
        ):
            stream_rate_hz = (sample_count - 1) / ((last_ns - first_ns) / 1_000_000_000.0)

        status = "ok"
        reason = "HIGHRES_IMU samples are fresh"
        if update_age_ms > self._highres_imu_max_staleness_ms:
            status = "stale"
            reason = (
                f"latest HIGHRES_IMU sample age {update_age_ms:.1f} ms exceeds "
                f"{self._highres_imu_max_staleness_ms:.1f} ms"
            )

        return HighresImuHealth(
            status=status,
            reason=reason,
            enabled=True,
            sample_count=sample_count,
            stream_rate_hz=stream_rate_hz,
            update_age_ms=update_age_ms,
            max_staleness_ms=self._highres_imu_max_staleness_ms,
        )

    def close(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._mav is not None:
            try:
                self._mav.close()
            except Exception:
                pass
            self._mav = None
            self._guided_mode_last_sent_monotonic_s = None

    def _takeoff(self) -> None:
        if self._log_commands:
            _logger.info("[MAVLink <<] TAKEOFF alt=%sm", self._takeoff_altitude_m)
        self._state_ready_evt.wait(timeout=self._heartbeat_timeout_s)
        self._set_guided_mode(force=True)
        self._send_takeoff_command()

        target_z = -abs(self._takeoff_altitude_m)
        climb_vz = -abs(self._takeoff_climb_speed_ms)
        expected_s = abs(self._takeoff_altitude_m) / max(0.1, abs(self._takeoff_climb_speed_ms))
        deadline = time.time() + min(self._takeoff_timeout_s, expected_s + 0.8)

        while time.time() < deadline:
            self._stream_velocity(0.0, 0.0, climb_vz, 0.2)
            if float(self.getMultirotorState().kinematics_estimated.position.z_val) <= target_z:
                self._stream_velocity(0.0, 0.0, 0.0, 0.25)
                return

        self._stream_velocity(0.0, 0.0, 0.0, 0.25)

    def _land(self) -> None:
        if self._log_commands:
            _logger.info("[MAVLink <<] LAND")
        self._state_ready_evt.wait(timeout=self._heartbeat_timeout_s)
        target_altitude_m = 0.15
        expected_s = abs(self._takeoff_altitude_m) / max(0.1, abs(self._land_descent_speed_ms))
        deadline = time.time() + min(self._land_timeout_s, expected_s + 3.0)

        while time.time() < deadline:
            state = self.getMultirotorState().kinematics_estimated
            altitude_m = max(0.0, -float(state.position.z_val))
            if altitude_m <= target_altitude_m:
                break
            self._stream_velocity(0.0, 0.0, abs(self._land_descent_speed_ms), 0.2)

        self._stream_velocity(0.0, 0.0, 0.0, 0.25)
        self.armDisarm(False)

    def _go_home(self) -> None:
        if self._log_commands:
            _logger.info("[MAVLink <<] GO_HOME")
        if self._mav is None or self._target_system is None:
            return
        self._mav.mav.command_long_send(
            self._target_system,
            self._target_component or 1,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def _get_latest_telemetry(self) -> _Telemetry | None:
        with self._telemetry_lock:
            return self._telemetry

    def _request_message_intervals(self) -> None:
        assert self._mav is not None and self._target_system is not None
        interval_us = int(1e6 / max(1.0, self._state_request_hz))
        for message_name in ("LOCAL_POSITION_NED", "HEARTBEAT"):
            message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
            if message_id is None:
                continue
            self._mav.mav.message_interval_send(int(message_id), interval_us)
        if self._highres_imu_enabled:
            message_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_HIGHRES_IMU", None)
            if message_id is not None:
                imu_interval_us = int(1e6 / max(1.0, self._highres_imu_request_hz))
                self._mav.mav.message_interval_send(int(message_id), imu_interval_us)

    def _start_telemetry_pump(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._telemetry_loop,
            name="mavlink_telemetry",
            daemon=True,
        )
        self._thread.start()

    def _telemetry_loop(self) -> None:
        assert self._mav is not None
        armed_bit = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        next_timesync_request_s = time.monotonic()
        message_types = ["LOCAL_POSITION_NED", "HEARTBEAT", "TIMESYNC"]
        if self._highres_imu_enabled:
            message_types.append("HIGHRES_IMU")
        while not self._stop_evt.is_set():
            if self._send_timesync_requests and time.monotonic() >= next_timesync_request_s:
                self._send_timesync_request()
                next_timesync_request_s = time.monotonic() + self._timesync_request_interval_s
            try:
                message = self._mav.recv_match(
                    type=message_types,
                    blocking=True,
                    timeout=0.2,
                )
                if message is None:
                    continue
                message_type = message.get_type()
                if message_type == "LOCAL_POSITION_NED":
                    self._handle_local_position(message)
                    if self._log_commands:
                        _logger.info(
                            "[MAVLink >>] LOCAL_POSITION_NED x=%.2f y=%.2f z=%.2f",
                            message.x,
                            message.y,
                            message.z,
                        )
                elif message_type == "HEARTBEAT":
                    armed = (int(message.base_mode) & int(armed_bit)) != 0
                    self._handle_heartbeat(armed)
                    if self._log_commands:
                        mode = getattr(message, "custom_mode", "?")
                        _logger.info(
                            "[MAVLink >>] HEARTBEAT armed=%s mode=%s", armed, mode
                        )
                elif message_type == "TIMESYNC":
                    self._handle_timesync(message)
                elif message_type == "HIGHRES_IMU":
                    self._handle_highres_imu(message)
            except Exception:
                continue

    def _handle_local_position(self, message: Any) -> None:
        position = _Vector3r(float(message.x), float(message.y), float(message.z))
        velocity = _Vector3r(float(message.vx), float(message.vy), float(message.vz))
        previous = self._get_latest_telemetry()
        armed = previous.armed if previous is not None else False
        with self._telemetry_lock:
            self._telemetry = _Telemetry(
                state=_MultirotorState(
                    kinematics_estimated=_KinematicsEstimated(
                        position=position,
                        linear_velocity=velocity,
                    )
                ),
                armed=armed,
            )
            self._state_ready_evt.set()

    def _handle_heartbeat(self, armed: bool) -> None:
        previous = self._get_latest_telemetry()
        if previous is None:
            return
        with self._telemetry_lock:
            self._telemetry = _Telemetry(state=previous.state, armed=armed)

    def _handle_timesync(self, message: Any) -> None:
        event = self._timesync_store.handle_message(message)
        if event.is_request and self._respond_to_timesync_requests:
            self._send_timesync_response(event.ts1)
        if self._timesync_log_messages:
            snapshot = self._timesync_store.snapshot()
            direction = "request" if event.is_request else "response"
            measurement = snapshot.last_measurement
            health = snapshot.sync_health
            measurement_log = ""
            if measurement is not None:
                measurement_log = (
                    f" offset_ms={measurement.offset_ns / 1_000_000.0:.3f}"
                    f" rtt_ms={measurement.rtt_wall_ns / 1_000_000.0:.3f}"
                )
            stable_log = ""
            if snapshot.stable_offset_ns is not None and snapshot.stable_rtt_ns is not None:
                stable_log = (
                    f" stable_offset_ms={snapshot.stable_offset_ns / 1_000_000.0:.3f}"
                    f" stable_rtt_ms={snapshot.stable_rtt_ns / 1_000_000.0:.3f}"
                )
            jitter_log = ""
            if snapshot.offset_jitter_ns is not None:
                jitter_log = f" jitter_ms={snapshot.offset_jitter_ns / 1_000_000.0:.3f}"
            print(
                "[mavlink] TIMESYNC "
                f"{direction} count={snapshot.message_count} tc1={event.tc1} ts1={event.ts1}"
                f" health={health.status}"
                f"{measurement_log}{stable_log}{jitter_log}"
            )

    def _handle_highres_imu(self, message: Any) -> None:
        previous = self.getHighresImu()
        received_ns = time.monotonic_ns()
        source_system = self._message_source_id(message, "get_srcSystem")
        source_component = self._message_source_id(message, "get_srcComponent")
        sample = merge_highres_imu_sample(
            previous,
            time_usec=int(getattr(message, "time_usec", 0)),
            xacc=self._float_or_none(getattr(message, "xacc", None)),
            yacc=self._float_or_none(getattr(message, "yacc", None)),
            zacc=self._float_or_none(getattr(message, "zacc", None)),
            xgyro=self._float_or_none(getattr(message, "xgyro", None)),
            ygyro=self._float_or_none(getattr(message, "ygyro", None)),
            zgyro=self._float_or_none(getattr(message, "zgyro", None)),
            xmag=self._float_or_none(getattr(message, "xmag", None)),
            ymag=self._float_or_none(getattr(message, "ymag", None)),
            zmag=self._float_or_none(getattr(message, "zmag", None)),
            abs_pressure=self._float_or_none(getattr(message, "abs_pressure", None)),
            diff_pressure=self._float_or_none(getattr(message, "diff_pressure", None)),
            pressure_alt=self._float_or_none(getattr(message, "pressure_alt", None)),
            temperature=self._float_or_none(getattr(message, "temperature", None)),
            fields_updated=int(getattr(message, "fields_updated", 0)),
            sensor_id=int(getattr(message, "id", 0)),
            source_system=source_system,
            source_component=source_component,
            local_received_monotonic_ns=received_ns,
            transport="mavlink",
        )
        with self._highres_imu_lock:
            self._highres_imu = sample
            self._highres_imu_sample_count += 1
            if self._highres_imu_first_monotonic_ns is None:
                self._highres_imu_first_monotonic_ns = received_ns
            self._highres_imu_last_monotonic_ns = received_ns
        if self._highres_imu_log_messages:
            print(f"[mavlink] HIGHRES_IMU {format_highres_imu_health(self.getHighresImuHealth())}")

    def _send_timesync_request(self) -> None:
        if self._mav is None:
            return
        request = self._record_outbound_timesync_request()
        self._mav.mav.timesync_send(0, request.ts1)
        if self._timesync_log_messages:
            print(f"[mavlink] TIMESYNC request sent ts1={request.ts1}")

    def _record_outbound_timesync_request(self) -> TimesyncOutboundRequest:
        return self._timesync_store.record_outbound_request()

    def _send_timesync_response(self, ts1: int) -> None:
        if self._mav is None:
            return
        self._mav.mav.timesync_send(time.time_ns(), int(ts1))

    @staticmethod
    def _float_or_none(value: Any) -> float | None:
        if value is None:
            return None
        return float(value)

    @staticmethod
    def _message_source_id(message: Any, getter_name: str) -> int | None:
        getter = getattr(message, getter_name, None)
        if not callable(getter):
            return None
        try:
            return int(getter())
        except Exception:
            return None

    def _set_guided_mode(self, *, force: bool = False) -> None:
        if self._mav is None or self._target_system is None:
            return
        if force and self._log_commands:
            _logger.info("[MAVLink <<] SET_GUIDED_MODE force=%s", force)
        now_s = time.monotonic()
        if not force and self._guided_mode_last_sent_monotonic_s is not None:
            if (
                now_s - self._guided_mode_last_sent_monotonic_s
                < self._GUIDED_MODE_MIN_RESEND_INTERVAL_S
            ):
                return
        self._mav.mav.command_long_send(
            self._target_system,
            self._target_component or 1,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self._guided_custom_mode,
            0,
            0,
            0,
            0,
            0,
        )
        self._guided_mode_last_sent_monotonic_s = now_s

    def _send_takeoff_command(self) -> None:
        if self._log_commands:
            _logger.info("[MAVLink <<] NAV_TAKEOFF alt=%sm", self._takeoff_altitude_m)
        if self._mav is None or self._target_system is None:
            return
        self._mav.mav.command_long_send(
            self._target_system,
            self._target_component or 1,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            abs(self._takeoff_altitude_m),
        )

    def _stream_velocity(self, vx: float, vy: float, vz: float, duration_s: float) -> None:
        assert self._mav is not None and self._target_system is not None
        velocity_only_type_mask = build_velocity_type_mask()
        self._stream_set_position_target_local_ned(
            SetPositionTargetLocalNedCommand(
                frame=SET_POSITION_FRAME_LOCAL_NED,
                type_mask=velocity_only_type_mask,
                vx=float(vx),
                vy=float(vy),
                vz=float(vz),
            ),
            duration_s,
        )

    def _stream_set_position_target_local_ned(
        self, command: SetPositionTargetLocalNedCommand, duration_s: float
    ) -> None:
        assert self._mav is not None and self._target_system is not None
        self._set_guided_mode(force=False)
        period_s = self._command_rate_gate.period_s
        deadline = time.monotonic() + max(0.0, float(duration_s))
        next_tick = time.monotonic()
        while time.monotonic() < deadline:
            now_s = time.monotonic()
            if self._command_rate_gate.allow(now_s):
                self._send_set_position_target_local_ned(command)
            next_tick += period_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    def _send_set_position_target_local_ned(
        self,
        command: SetPositionTargetLocalNedCommand,
        *,
        t_ms: int | None = None,
    ) -> None:
        assert self._mav is not None and self._target_system is not None
        frame = command.frame
        if frame not in self._POSITION_TARGET_FRAME_MAP:
            raise ValueError(
                f"Unsupported SET_POSITION_TARGET_LOCAL_NED frame {command.frame!r}. "
                "Expected one of: local_ned, body_ned."
            )

        timestamp_ms = int((time.time() * 1000) % 2**32) if t_ms is None else int(t_ms)
        self._mav.mav.set_position_target_local_ned_send(
            timestamp_ms,
            self._target_system,
            self._target_component or 1,
            self._POSITION_TARGET_FRAME_MAP[frame],
            int(command.type_mask),
            float(command.x),
            float(command.y),
            float(command.z),
            float(command.vx),
            float(command.vy),
            float(command.vz),
            float(command.afx),
            float(command.afy),
            float(command.afz),
            float(command.yaw),
            float(command.yaw_rate),
        )

    def _stream_attitude_target(
        self,
        quaternion: tuple[float, float, float, float],
        thrust: float,
        duration_s: float,
    ) -> None:
        assert self._mav is not None and self._target_system is not None
        self._set_guided_mode()
        period_s = self._command_rate_gate.period_s
        type_mask = (
            int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE)
            | int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE)
            | int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE)
        )
        deadline = time.monotonic() + max(0.0, float(duration_s))
        next_tick = time.monotonic()
        while time.monotonic() < deadline:
            now_s = time.monotonic()
            if self._command_rate_gate.allow(now_s):
                t_ms = int((time.time() * 1000) % 2**32)
                self._mav.mav.set_attitude_target_send(
                    t_ms,
                    self._target_system,
                    self._target_component or 1,
                    type_mask,
                    quaternion,
                    0.0,
                    0.0,
                    0.0,
                    float(max(0.0, min(1.0, thrust))),
                )
            next_tick += period_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    def _stream_attitude_rate_target(
        self,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
        thrust: float,
        duration_s: float,
    ) -> None:
        assert self._mav is not None and self._target_system is not None
        self._set_guided_mode()
        period_s = self._command_rate_gate.period_s
        type_mask = int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE)
        deadline = time.monotonic() + max(0.0, float(duration_s))
        next_tick = time.monotonic()
        while time.monotonic() < deadline:
            now_s = time.monotonic()
            if self._command_rate_gate.allow(now_s):
                t_ms = int((time.time() * 1000) % 2**32)
                self._mav.mav.set_attitude_target_send(
                    t_ms,
                    self._target_system,
                    self._target_component or 1,
                    type_mask,
                    [1.0, 0.0, 0.0, 0.0],
                    float(roll_rate),
                    float(pitch_rate),
                    float(yaw_rate),
                    float(max(0.0, min(1.0, thrust))),
                )
            next_tick += period_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    @staticmethod
    def _quaternion_from_euler(
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
