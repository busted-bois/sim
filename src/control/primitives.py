"""Shared flight primitives used across algorithms."""

from __future__ import annotations

import errno
import math
import os
import sys
import threading
import time
from typing import TYPE_CHECKING

import airsim

if TYPE_CHECKING:
    from src.config import Config
    from src.control.flight_client import FlightClient
    from src.landing_telemetry import LandingTelemetrySampler


def _airsim_quaternion_from_euler(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> airsim.Quaternionr:
    cr = math.cos(roll_rad / 2.0)
    sr = math.sin(roll_rad / 2.0)
    cp = math.cos(pitch_rad / 2.0)
    sp = math.sin(pitch_rad / 2.0)
    cy = math.cos(yaw_rad / 2.0)
    sy = math.sin(yaw_rad / 2.0)
    return airsim.Quaternionr(
        cr * sp * cy + sr * cp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def set_front_camera_pose(client: FlightClient, config: Config | dict) -> None:
    vision_cfg = config.get("vision", {})
    camera_name = str(vision_cfg.get("camera_name", "0"))
    cam_cfg = config.get("camera", {})
    pose_offset = tuple(cam_cfg.get("pose_offset", [0.35, 0.0, -0.05]))
    pitch_up_degrees = float(cam_cfg.get("pitch_up_degrees", 20.0))
    roll_degrees = float(cam_cfg.get("roll_degrees", 0.0))
    yaw_degrees = float(cam_cfg.get("yaw_degrees", 0.0))
    front_pose = airsim.Pose(
        airsim.Vector3r(pose_offset[0], pose_offset[1], pose_offset[2]),
        _airsim_quaternion_from_euler(
            math.radians(roll_degrees),
            math.radians(pitch_up_degrees),
            math.radians(yaw_degrees),
        ),
    )
    try:
        client.simSetCameraPose(camera_name, front_pose)
    except Exception as exc:
        print(f"Warning: failed to set front camera pose for '{camera_name}': {exc}")


def apply_trace_style(client: FlightClient, config: Config | dict) -> None:
    sim_cfg = config.get("simulator", {})
    trace_cfg = sim_cfg.get("trace", {})
    trace_enabled = bool(trace_cfg.get(
        "enabled",
        os.environ.get("AIGP_ENABLE_TRACE", "").strip() == "1",
    ))
    if not trace_enabled:
        return

    raw_color = trace_cfg.get("color_rgba", [1.0, 0.0, 1.0, 1.0])
    color = [max(0.0, min(1.0, float(v))) for v in raw_color]

    thickness = max(1.0, float(trace_cfg.get("thickness", 4.0)))

    vehicle_name = str(trace_cfg.get("vehicle_name", "")).strip()
    try:
        client.simSetTraceLine(color, thickness, vehicle_name)
        print(f"Trace style applied: color={color}, thickness={thickness:.1f}")
    except Exception as exc:
        print(f"Warning: failed to set trace line style: {exc}")


def suppress_api_cleanup_warning(exc: BaseException) -> bool:
    """True when disarm/API cleanup failed because the connection is already gone."""
    if isinstance(exc, (BrokenPipeError, ConnectionAbortedError, ConnectionResetError)):
        return True
    if isinstance(exc, OSError):
        if getattr(exc, "winerror", None) in (10053, 10054):
            return True
        if exc.errno in (errno.ECONNRESET, errno.EPIPE, errno.ECONNABORTED):
            return True
    return False


def run_algorithm_with_timeout(algo, client, timeout_seconds: float) -> None:
    import traceback

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

    if elapsed_s < 8.0:
        print(
            f"Warning: algorithm reported completion in {elapsed_s:.1f}s -- much shorter than "
            "a full attitude routine. If the drone barely moved, check that the simulation "
            "is real-time, and watch for errors above.",
            file=sys.stderr,
        )


def takeoff_with_settle(
    client: FlightClient, max_attempts: int = 4, label: str = "primitives"
) -> None:
    last_exc: Exception | None = None
    for attempt in range(1, max_attempts + 1):
        try:
            client.takeoffAsync().join()
            return
        except Exception as exc:
            last_exc = exc
            if attempt == max_attempts:
                raise
            print(
                f"[{label}] takeoff attempt {attempt}/{max_attempts} "
                f"failed ({type(exc).__name__}: {exc}); retrying...",
                file=sys.stderr,
            )
            time.sleep(1.5 * attempt)
    if last_exc is not None:
        raise last_exc


def rotate_yaw(
    client: FlightClient,
    rate_dps: float,
    duration_s: float,
    label: str = "primitives",
) -> None:
    client.rotateByYawRateAsync(rate_dps, duration_s).join()


def hold_position(client: FlightClient, duration_s: float) -> None:
    client.moveByVelocityAsync(0.0, 0.0, 0.0, duration_s).join()


def land_with_telemetry(
    client: FlightClient,
    config: Config | dict,
    label: str = "primitives",
) -> None:
    landing_cfg = config.get("landing", {})
    profile = os.environ.get("AIGP_LANDING_PROFILE", "").strip() or landing_cfg.get(
        "profile", "faster_soft"
    )

    min_hover_seconds = max(0.5, float(landing_cfg.get("min_hover_seconds", 1.0)))
    max_descent_speed_ms = max(0.5, float(landing_cfg.get("max_descent_speed_ms", 2.5)))

    sampler = _landing_telemetry_if_enabled(client, landing_cfg, label)
    try:
        print(f"[{label}] Landing profile: {profile}")
        if sampler:
            sampler.set_command("hover_async")
        client.hoverAsync().join()
        if min_hover_seconds > 0:
            if sampler:
                sampler.set_command("hover_settle")
            print(f"[{label}] Hover settle: {min_hover_seconds:.2f}s")
            time.sleep(min_hover_seconds)

        if profile == "very_soft":
            print(f"[{label}] Hover settle complete -- starting final land.")
            if sampler:
                sampler.set_command("land_async")
            client.landAsync().join()
            return

        print(
            f"[{label}] Hover settle complete -- next: controlled descent if above final altitude, "
            "then final land."
        )
        descent_speed_ms = max(0.5, float(landing_cfg.get("descent_speed_ms", 2.0)))
        descent_speed_ms = min(descent_speed_ms, max_descent_speed_ms)
        final_land_altitude_m = max(0.3, float(landing_cfg.get("final_land_altitude_m", 1.0)))

        state = client.getMultirotorState().kinematics_estimated
        altitude_m = max(0.0, -float(state.position.z_val))
        if altitude_m > final_land_altitude_m:
            descent_distance = altitude_m - final_land_altitude_m
            descent_duration_s = descent_distance / descent_speed_ms
            print(
                f"[{label}] Controlled descent: speed={descent_speed_ms:.2f} m/s "
                f"for {descent_duration_s:.2f}s before final land."
            )
            if sampler:
                sampler.set_command(f"move_by_velocity vz_ms={descent_speed_ms:.3f}")
            client.moveByVelocityAsync(0.0, 0.0, descent_speed_ms, descent_duration_s).join()
            if sampler:
                sampler.set_command("hover_async")
            client.hoverAsync().join()

        if sampler:
            sampler.set_command("land_async")
        client.landAsync().join()
    finally:
        if sampler is not None:
            sampler.stop()
            print(f"[{label}] Landing telemetry saved: {sampler.out_path}")


def wait_until_stationary(
    client: FlightClient,
    timeout_s: float = 8.0,
    velocity_eps_ms: float = 0.05,
    label: str = "primitives",
) -> None:
    """Block until drone velocity drops below velocity_eps_ms."""
    deadline = time.monotonic() + timeout_s
    last_speed = float("inf")
    consecutive_quiet = 0
    while time.monotonic() < deadline:
        try:
            v = client.getMultirotorState().kinematics_estimated.linear_velocity
            speed = (float(v.x_val) ** 2 + float(v.y_val) ** 2 + float(v.z_val) ** 2) ** 0.5
        except Exception:
            speed = float("inf")
        last_speed = speed
        if speed < velocity_eps_ms:
            consecutive_quiet += 1
            if consecutive_quiet >= 2:
                return
        else:
            consecutive_quiet = 0
            try:
                client.cancelLastTask()
                client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.2).join()
            except Exception:
                pass
        time.sleep(0.1)
    print(
        f"[{label}] Warning: drone still moving ({last_speed:.2f} m/s) after "
        f"{timeout_s:.1f}s settle; proceeding anyway",
        file=sys.stderr,
    )


def _landing_telemetry_if_enabled(
    client: FlightClient, landing_cfg: dict, label: str = "primitives"
) -> LandingTelemetrySampler | None:
    from pathlib import Path

    from src.landing_telemetry import LandingTelemetrySampler

    tel_cfg = landing_cfg.get("telemetry_log", {})
    if not tel_cfg.get("enabled", False):
        return None
    raw_path = str(tel_cfg.get("path", "logs/landing_telemetry.csv")).strip()
    out_path = Path(raw_path)
    if not out_path.is_absolute():
        out_path = Path(__file__).resolve().parent.parent.parent / out_path
    sample_hz = float(tel_cfg.get("sample_hz", 20.0))
    sampler = LandingTelemetrySampler(client, out_path, sample_hz)
    sampler.set_command("start")
    sampler.start()
    return sampler
