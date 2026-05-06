"""Shared flight primitives for drone control.

Plain functions for common flight sequences used across algorithms.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import airsim
from msgpackrpc.error import RPCError

if TYPE_CHECKING:
    from src.config import Config
    from src.control.flight_client import FlightClient
    from src.landing_telemetry import LandingTelemetrySampler


def set_front_camera_pose(client: FlightClient, config: Config | dict) -> None:
    vision_cfg = config.get("vision", {})
    camera_name = str(vision_cfg.get("camera_name", "0"))
    cam_cfg = config.get("camera", {})
    pose_offset = tuple(cam_cfg.get("pose_offset", [0.35, 0.0, -0.05]))
    front_pose = airsim.Pose(
        airsim.Vector3r(pose_offset[0], pose_offset[1], pose_offset[2]),
        airsim.Quaternionr(0.0, 0.0, 0.0, 1.0),
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
        __import__("os").environ.get("AIGP_ENABLE_TRACE", "").strip() == "1",
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
    """True when disarm/API cleanup failed because the sim or socket is already gone."""
    import errno

    if isinstance(exc, (BrokenPipeError, ConnectionAbortedError, ConnectionResetError)):
        return True
    from msgpackrpc.error import RPCError, TransportError

    if isinstance(exc, (RPCError, TransportError)):
        msg = str(exc).lower()
        return any(
            token in msg
            for token in (
                "connection reset",
                "connection aborted",
                "broken pipe",
                "forcibly closed",
                "transport endpoint is not connected",
                "not connected",
                "failed to send request",
            )
        )
    if isinstance(exc, OSError):
        if getattr(exc, "winerror", None) in (10053, 10054):
            return True
        if exc.errno in (errno.ECONNRESET, errno.EPIPE, errno.ECONNABORTED):
            return True
    return False


def run_algorithm_with_timeout(algo, client, timeout_seconds: float) -> None:
    import sys
    import threading
    import time
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
            f"Warning: algorithm reported completion in {elapsed_s:.1f}s — much shorter than "
            "a full attitude routine. If the drone barely moved, check Unreal is unpaused, "
            "simulation is real-time, and watch for errors above.",
            file=sys.stderr,
        )


def takeoff_with_settle(
    client: FlightClient, max_attempts: int = 4, label: str = "primitives"
) -> None:
    """Call takeoffAsync, retrying after a re-settle if AirSim complains about velocity.

    Some maps / physics ticks leave the vehicle just barely moving even after
    reset() + a stationary wait. AirSim then throws "vehicle is already moving
    with velocity X m/s" — re-settling and retrying clears this reliably.

    Args:
        client: Flight client instance.
        max_attempts: Maximum number of takeoff attempts.
        label: Prefix for log messages (e.g., algorithm name).
    """
    last_exc: Exception | None = None
    for attempt in range(1, max_attempts + 1):
        try:
            client.takeoffAsync().join()
            return
        except RPCError as exc:
            msg = str(exc).lower()
            if "already moving" not in msg:
                raise
            last_exc = exc
            print(
                f"[{label}] Takeoff attempt {attempt}/{max_attempts} rejected: {exc}; "
                "re-settling...",
                file=__import__("sys").stderr,
            )
            try:
                client.cancelLastTask()
                client.armDisarm(False)
                time.sleep(0.3)
                client.armDisarm(True)
            except Exception:
                pass
            _wait_until_stationary(client, timeout_s=6.0, velocity_eps_ms=0.03, label=label)
    if last_exc is not None:
        raise last_exc


def rotate_yaw(
    client: FlightClient,
    rate_dps: float,
    duration_s: float,
    label: str = "primitives",
) -> None:
    """Rotate by yaw rate for duration.

    Args:
        client: Flight client instance.
        rate_dps: Yaw rate in degrees per second.
        duration_s: Duration in seconds.
        label: Prefix for log messages.
    """
    client.rotateByYawRateAsync(rate_dps, duration_s).join()


def hold_position(client: FlightClient, duration_s: float) -> None:
    """Hold current position for duration.

    Args:
        client: Flight client instance.
        duration_s: Duration in seconds.
    """
    client.moveByVelocityAsync(0.0, 0.0, 0.0, duration_s).join()


def land_with_telemetry(
    client: FlightClient,
    config: Config | dict,
    label: str = "primitives",
) -> None:
    """Execute landing sequence with optional telemetry logging.

    Args:
        client: Flight client instance.
        config: Configuration dict containing landing settings.
        label: Prefix for log messages.
    """

    landing_cfg = config.get("landing", {})
    profile = __import__("os").environ.get("AIGP_LANDING_PROFILE", "").strip() or landing_cfg.get(
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
            print(f"[{label}] Hover settle complete — starting final land.")
            if sampler:
                sampler.set_command("land_async")
            client.landAsync().join()
            return

        print(
            f"[{label}] Hover settle complete — next: controlled descent if above final altitude, "
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


def _wait_until_stationary(
    client: FlightClient,
    timeout_s: float = 8.0,
    velocity_eps_ms: float = 0.05,
    label: str = "primitives",
) -> None:
    """Block until the drone is stationary, so takeoff doesn't get rejected.

    AirSim's takeoff RPC refuses if |velocity| is non-trivial — observed
    rejection at 0.19 m/s, so the epsilon must be well below that. After
    client.reset() the drone usually settles within ~0.5s, but a previous run
    that left residual motion can take several physics steps to bleed off.

    Args:
        client: Flight client instance.
        timeout_s: Maximum time to wait for drone to settle.
        velocity_eps_ms: Velocity threshold in m/s to consider stationary.
        label: Prefix for log messages.
    """
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
            # Two consecutive quiet samples — guards against catching the
            # vehicle mid-zero-crossing while it's actually still oscillating.
            if consecutive_quiet >= 2:
                return
        else:
            consecutive_quiet = 0
            try:
                client.cancelLastTask()
                # Pin to spawn z so residual vertical motion damps fast.
                client.moveByVelocityAsync(0.0, 0.0, 0.0, 0.2).join()
            except Exception:
                pass
        time.sleep(0.1)
    print(
        f"[{label}] Warning: drone still moving ({last_speed:.2f} m/s) after "
        f"{timeout_s:.1f}s settle; proceeding anyway",
        file=__import__("sys").stderr,
    )


def _landing_telemetry_if_enabled(
    client: FlightClient, landing_cfg: dict, label: str = "primitives"
) -> LandingTelemetrySampler | None:
    """Create landing telemetry sampler if enabled in config.

    Args:
        client: Flight client instance.
        landing_cfg: Landing configuration dict.
        label: Prefix for log messages.

    Returns:
        LandingTelemetrySampler instance if enabled, None otherwise.
    """
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
