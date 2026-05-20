"""Main control loop: telemetry fusion, optional vision, algorithm execution."""

from __future__ import annotations

import math
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.control.highres_imu import HighresImuSample
from src.control.utils import _yaw_from_orientation

if TYPE_CHECKING:
    from src.control.algorithms import Algorithm
    from src.control.flight_client import FlightClient
    from src.vision import VisionFeed, VisionFrame


@dataclass(frozen=True, slots=True)
class VehicleState:
    position_ned: tuple[float, float, float]
    velocity_ned: tuple[float, float, float]
    attitude_rad: tuple[float, float, float]
    imu: HighresImuSample | None


def _quaternion_to_euler_rad(
    x: float, y: float, z: float, w: float
) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def snapshot_vehicle_state(client: FlightClient) -> VehicleState:
    kin = client.getMultirotorState().kinematics_estimated
    pos = kin.position
    vel = kin.linear_velocity
    imu = client.getHighresImu()
    roll = pitch = yaw = 0.0
    orientation = kin.orientation
    if orientation is not None and hasattr(orientation, "w_val"):
        roll, pitch, yaw = _quaternion_to_euler_rad(
            float(orientation.x_val),
            float(orientation.y_val),
            float(orientation.z_val),
            float(orientation.w_val),
        )
    elif orientation is not None:
        yaw = _yaw_from_orientation(orientation)
    return VehicleState(
        position_ned=(float(pos.x_val), float(pos.y_val), float(pos.z_val)),
        velocity_ned=(float(vel.x_val), float(vel.y_val), float(vel.z_val)),
        attitude_rad=(roll, pitch, yaw),
        imu=imu,
    )


def run_algorithm_with_timeout(
    algo: Algorithm,
    client: FlightClient,
    timeout_seconds: float,
    *,
    vision_feed: VisionFeed | None = None,
    command_rate_hz: float | None = None,
) -> None:
    """Run algorithm in a worker thread with timeout."""
    if getattr(algo, "uses_control_loop", False):
        _run_tick_loop(algo, client, timeout_seconds, vision_feed, command_rate_hz)
        return

    error_holder: dict[str, BaseException] = {}

    def _target() -> None:
        try:
            algo.run(client)
        except BaseException as exc:
            error_holder["error"] = exc

    worker = threading.Thread(target=_target, name="algorithm_runner", daemon=True)
    worker.start()
    started = time.perf_counter()
    deadline = started + timeout_seconds
    join_slice_s = 0.25
    while worker.is_alive():
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            break
        worker.join(timeout=min(join_slice_s, remaining))
    elapsed_s = time.perf_counter() - started

    if worker.is_alive():
        raise TimeoutError(f"Algorithm timed out after {timeout_seconds:.1f}s")

    if "error" in error_holder:
        exc = error_holder["error"]
        print(
            f"Algorithm thread ended after {elapsed_s:.1f}s with error: "
            f"{type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        traceback.print_exception(type(exc), exc, exc.__traceback__, file=sys.stderr)
        raise RuntimeError(
            f"Algorithm raised an exception after {elapsed_s:.1f}s"
        ) from exc


def _run_tick_loop(
    algo: Algorithm,
    client: FlightClient,
    timeout_seconds: float,
    vision_feed: VisionFeed | None,
    command_rate_hz: float | None,
) -> None:
    rate_hz = max(5.0, float(command_rate_hz or 50.0))
    period_s = 1.0 / rate_hz
    deadline = time.perf_counter() + timeout_seconds
    next_tick = time.perf_counter()
    error_holder: dict[str, BaseException] = {}

    while time.perf_counter() < deadline:
        try:
            state = snapshot_vehicle_state(client)
            frame: VisionFrame | None = None
            if vision_feed is not None and vision_feed.enabled:
                frame = vision_feed.get_latest()
            algo.run_tick(client, state, frame)
            if algo.flight_complete:
                break
        except BaseException as exc:
            error_holder["error"] = exc
            break
        next_tick += period_s
        sleep_s = next_tick - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.perf_counter()

    if "error" in error_holder:
        raise error_holder["error"]
