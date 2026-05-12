from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Final

from pymavlink import mavutil

from src.control.mavlink_timesync import TimesyncOutboundRequest, TimesyncSnapshot, TimesyncStore


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
    """AirSim-like MAVLink runtime client with TIMESYNC observability."""

    _DEFAULT_ENDPOINT: Final[str] = "udpin:0.0.0.0:14550"

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
        timesync_pending_request_limit: int = 64,
        timesync_stable_window_size: int = 9,
        timesync_stable_best_subset_size: int = 5,
        timesync_min_stable_samples: int = 3,
        timesync_max_stable_rtt_ns: int = 250_000_000,
        timesync_max_offset_jitter_ns: int = 50_000_000,
        connection_factory: Callable[..., Any] | None = None,
    ) -> None:
        self._endpoint = endpoint.strip() if endpoint else self._DEFAULT_ENDPOINT
        _parse_udp_endpoint(self._endpoint)

        self._command_rate_hz = float(command_rate_hz)
        self._state_request_hz = float(state_request_hz)
        self._guided_custom_mode = int(guided_custom_mode)
        self._takeoff_altitude_m = float(takeoff_altitude_m)
        self._takeoff_climb_speed_ms = float(takeoff_climb_speed_ms)
        self._land_descent_speed_ms = float(land_descent_speed_ms)
        self._takeoff_timeout_s = float(takeoff_timeout_s)
        self._land_timeout_s = float(land_timeout_s)
        self._heartbeat_timeout_s = float(heartbeat_timeout_s)
        self._source_system = int(source_system)
        self._source_component = int(source_component)
        self._respond_to_timesync_requests = bool(respond_to_timesync_requests)
        self._timesync_log_messages = bool(timesync_log_messages)
        self._send_timesync_requests = bool(send_timesync_requests)
        self._timesync_request_interval_s = max(0.1, float(timesync_request_interval_s))
        self._prepare_for_flight_on_connect = bool(prepare_for_flight_on_connect)
        self._request_state_messages_on_connect = bool(request_state_messages_on_connect)
        self._connection_factory = connection_factory or mavutil.mavlink_connection

        self._mav: Any | None = None
        self._target_system: int | None = None
        self._target_component: int | None = None

        self._telemetry_lock = threading.Lock()
        self._telemetry: _Telemetry | None = None
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
            self._set_guided_mode()

    def enableApiControl(self, enable: bool) -> None:
        _ = enable

    def armDisarm(self, arm: bool) -> None:
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
        return _Joinable(lambda: self._stream_velocity(vx, vy, vz, duration))

    def moveByVelocityZAsync(self, vx: float, vy: float, z: float, duration: float) -> _Joinable:
        def _run() -> None:
            current_z = float(self.getMultirotorState().kinematics_estimated.position.z_val)
            if duration <= 0:
                vz = 0.0
            else:
                vz = max(-2.5, min(2.5, (float(z) - current_z) / float(duration)))
            self._stream_velocity(vx, vy, vz, duration)

        return _Joinable(_run)

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

    def _takeoff(self) -> None:
        self._state_ready_evt.wait(timeout=self._heartbeat_timeout_s)
        self._set_guided_mode()
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
        while not self._stop_evt.is_set():
            if self._send_timesync_requests and time.monotonic() >= next_timesync_request_s:
                self._send_timesync_request()
                next_timesync_request_s = time.monotonic() + self._timesync_request_interval_s
            try:
                message = self._mav.recv_match(
                    type=["LOCAL_POSITION_NED", "HEARTBEAT", "TIMESYNC"],
                    blocking=True,
                    timeout=0.2,
                )
                if message is None:
                    continue
                message_type = message.get_type()
                if message_type == "LOCAL_POSITION_NED":
                    self._handle_local_position(message)
                elif message_type == "HEARTBEAT":
                    armed = (int(message.base_mode) & int(armed_bit)) != 0
                    self._handle_heartbeat(armed)
                elif message_type == "TIMESYNC":
                    self._handle_timesync(message)
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

    def _set_guided_mode(self) -> None:
        if self._mav is None or self._target_system is None:
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

    def _send_takeoff_command(self) -> None:
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
        self._set_guided_mode()
        period_s = max(0.02, 1.0 / max(5.0, min(50.0, self._command_rate_hz)))
        type_mask = (
            int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        )

        deadline = time.time() + max(0.0, float(duration_s))
        while time.time() < deadline:
            t_ms = int((time.time() * 1000) % 2**32)
            self._mav.mav.set_position_target_local_ned_send(
                t_ms,
                self._target_system,
                self._target_component or 1,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0,
                0.0,
                0.0,
                float(vx),
                float(vy),
                float(vz),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            time.sleep(period_s)

    def _stream_attitude_target(
        self,
        quaternion: tuple[float, float, float, float],
        thrust: float,
        duration_s: float,
    ) -> None:
        assert self._mav is not None and self._target_system is not None
        self._set_guided_mode()
        period_s = max(0.02, 1.0 / max(5.0, min(50.0, self._command_rate_hz)))
        type_mask = (
            int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE)
            | int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE)
            | int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE)
        )
        deadline = time.time() + max(0.0, float(duration_s))
        while time.time() < deadline:
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
            time.sleep(period_s)

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
        period_s = max(0.02, 1.0 / max(5.0, min(50.0, self._command_rate_hz)))
        type_mask = int(mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE)
        deadline = time.time() + max(0.0, float(duration_s))
        while time.time() < deadline:
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
            time.sleep(period_s)

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
