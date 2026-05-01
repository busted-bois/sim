import errno
import os
import sys
import threading
import time
from pathlib import Path

import airsim
from msgpackrpc.error import RPCError, TransportError
from src.airsim_maze_flight import (
    MazeFlightClient,
    maze_arm_disarm,
    maze_enable_api_control,
    resolve_maze_vehicle_name,
    try_maze_unpause_and_clock,
)
from src.config import load_config, simulator_endpoint
from src.control.algorithms import get_algorithm
from src.control.maze_runtime import choose_algorithm_name, is_maze_mode
from src.landing_telemetry import LandingTelemetrySampler
from src.vision import VisionFeed

ROOT = Path(__file__).resolve().parent


def _suppress_api_cleanup_warning(exc: BaseException) -> bool:
    """True when disarm/API cleanup failed because the sim or socket is already gone."""
    if isinstance(exc, (BrokenPipeError, ConnectionAbortedError, ConnectionResetError)):
        return True
    if isinstance(exc, (RPCError, TransportError)):
        # RPC wrappers frequently carry plain strings/bytes from the wire.
        # Only suppress clear connection-closed/reset cases.
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
        if getattr(exc, "winerror", None) in (10053, 10054):  # WSAECONNABORTED / WSAECONNRESET
            return True
        if exc.errno in (errno.ECONNRESET, errno.EPIPE, errno.ECONNABORTED):
            return True
    return False


def _set_front_camera_pose(client: airsim.MultirotorClient, config: dict) -> None:
    vision_cfg = config.get("vision", {})
    camera_name = str(vision_cfg.get("camera_name", "0"))
    front_pose = airsim.Pose(
        airsim.Vector3r(0.35, 0.0, -0.05),
        airsim.Quaternionr(0.0, 0.0, 0.0, 1.0),
    )
    vehicle_name = ""
    if is_maze_mode():
        vehicle_name = (
            os.environ.get("AIGP_AIRSIM_VEHICLE_NAME", "SimpleFlight").strip() or "SimpleFlight"
        )
    try:
        client.simSetCameraPose(camera_name, front_pose, vehicle_name=vehicle_name)
    except Exception as exc:
        print(f"Warning: failed to set front camera pose for '{camera_name}': {exc}")


def _apply_trace_style(client: airsim.MultirotorClient, config: dict) -> None:
    sim_cfg = config.get("simulator", {})
    trace_cfg = sim_cfg.get("trace", {})
    trace_enabled = bool(
        trace_cfg.get("enabled", os.environ.get("AIGP_ENABLE_TRACE", "").strip() == "1")
    )
    if not trace_enabled:
        return

    raw_color = trace_cfg.get("color_rgba", [1.0, 0.0, 1.0, 1.0])
    color = [max(0.0, min(1.0, float(v))) for v in raw_color]

    thickness = max(1.0, float(trace_cfg.get("thickness", 4.0)))

    vehicle_name = str(trace_cfg.get("vehicle_name", "")).strip()
    if is_maze_mode() and not vehicle_name:
        vehicle_name = (
            os.environ.get("AIGP_AIRSIM_VEHICLE_NAME", "SimpleFlight").strip() or "SimpleFlight"
        )
    try:
        client.simSetTraceLine(color, thickness, vehicle_name)
        print(f"Trace style applied: color={color}, thickness={thickness:.1f}")
    except Exception as exc:
        print(f"Warning: failed to set trace line style: {exc}")


def _apply_low_end_overrides(config: dict) -> None:
    if os.environ.get("AIGP_LOW_END", "").strip() != "1":
        return
    print("Low-end mode enabled: prioritizing smooth flight over detailed logging.")
    low_end_cfg = config.setdefault("low_end_profile", {})

    vision_cfg = config.setdefault("vision", {})
    vision_cfg["enabled"] = bool(low_end_cfg.get("vision_enabled", False))
    if vision_cfg["enabled"]:
        vision_cfg["fps"] = float(low_end_cfg.get("vision_fps", 8.0))

    control_cfg = config.setdefault("control", {})
    command_rate_hz = float(low_end_cfg.get("command_rate_hz", 25.0))
    control_cfg["command_rate_hz"] = max(10.0, min(35.0, command_rate_hz))
    latency_cfg = control_cfg.setdefault("latency_tuning", {})
    latency_cfg["enabled"] = False
    latency_cfg.setdefault("autotuner", {})["enabled"] = False

    landing_cfg = config.setdefault("landing", {})
    landing_cfg.setdefault("telemetry_log", {})["enabled"] = False
    landing_cfg["min_hover_seconds"] = min(0.6, float(landing_cfg.get("min_hover_seconds", 1.0)))

    log_cfg = config.setdefault("logging", {})
    log_cfg["basic_flight_logs"] = True
    config["algorithm"] = str(low_end_cfg.get("algorithm", "attitude_four_motion"))
    config.setdefault("safety", {})["algorithm_timeout_seconds"] = float(
        low_end_cfg.get("algorithm_timeout_seconds", 90.0)
    )

    six_cfg = config.setdefault("six_directions", {})
    six_cfg["duration_s"] = float(low_end_cfg.get("segment_duration_s", 1.2))
    six_cfg["speed_ms"] = float(low_end_cfg.get("speed_ms", 1.6))
    six_cfg["direction_labels"] = list(
        low_end_cfg.get("direction_labels", ["+X", "-X", "+Y", "-Y"])
    )


def _resolve_landing_cfg(config: dict) -> dict:
    """Base landing config, with optional maze-specific overrides when AIGP_MAZE=1."""
    base = dict(config.get("landing", {}))
    if not is_maze_mode():
        return base
    maze = base.get("maze")
    if not isinstance(maze, dict):
        return base
    merged = {**base}
    for key, value in maze.items():
        if key == "maze":
            continue
        merged[key] = value
    return merged


def _landing_telemetry_if_enabled(
    client: airsim.MultirotorClient, landing_cfg: dict
) -> LandingTelemetrySampler | None:
    tel_cfg = landing_cfg.get("telemetry_log", {})
    if not tel_cfg.get("enabled", False):
        return None
    raw_path = str(tel_cfg.get("path", "logs/landing_telemetry.csv")).strip()
    out_path = Path(raw_path)
    if not out_path.is_absolute():
        out_path = ROOT / out_path
    sample_hz = float(tel_cfg.get("sample_hz", 20.0))
    sampler = LandingTelemetrySampler(client, out_path, sample_hz)
    sampler.set_command("start")
    sampler.start()
    return sampler


def _run_landing(client: airsim.MultirotorClient, config: dict) -> None:
    landing_cfg = _resolve_landing_cfg(config)
    profile = os.environ.get("AIGP_LANDING_PROFILE", "").strip() or landing_cfg.get(
        "profile", "faster_soft"
    )

    min_hover_seconds = max(0.5, float(landing_cfg.get("min_hover_seconds", 1.0)))
    max_descent_speed_ms = max(0.5, float(landing_cfg.get("max_descent_speed_ms", 2.5)))

    sampler = _landing_telemetry_if_enabled(client, landing_cfg)
    try:
        print(f"Landing profile: {profile}")
        if sampler:
            sampler.set_command("hover_async")
        client.hoverAsync().join()
        if min_hover_seconds > 0:
            if sampler:
                sampler.set_command("hover_settle")
            print(f"Hover settle: {min_hover_seconds:.2f}s")
            time.sleep(min_hover_seconds)

        if profile == "very_soft":
            print("Hover settle complete — starting final land.")
            if sampler:
                sampler.set_command("land_async")
            client.landAsync().join()
            return

        if profile == "maze_soft":
            print(
                "Maze soft landing: stepped slow descent with settle between chunks, "
                "then final land."
            )
            descent_speed_ms = max(
                0.25,
                min(
                    float(landing_cfg.get("maze_descent_speed_ms", 0.55)),
                    max_descent_speed_ms,
                ),
            )
            chunk_m = max(0.15, float(landing_cfg.get("maze_descent_chunk_m", 0.35)))
            settle_s = max(0.0, float(landing_cfg.get("maze_settle_between_chunks_s", 0.35)))
            final_land_altitude_m = max(0.3, float(landing_cfg.get("final_land_altitude_m", 0.8)))
            pre_land_hover_s = max(0.0, float(landing_cfg.get("maze_pre_land_hover_s", 1.0)))
            max_steps = max(1, int(landing_cfg.get("maze_soft_max_steps", 40)))

            for step in range(max_steps):
                state = client.getMultirotorState().kinematics_estimated
                altitude_m = max(0.0, -float(state.position.z_val))
                if altitude_m <= final_land_altitude_m + 0.05:
                    print(
                        f"Maze soft landing: altitude {altitude_m:.2f}m ≤ "
                        f"{final_land_altitude_m:.2f}m — final approach."
                    )
                    break
                step_down = min(chunk_m, altitude_m - final_land_altitude_m)
                duration_s = max(0.2, min(4.0, step_down / descent_speed_ms))
                print(
                    f"Maze soft landing: chunk step={step + 1} "
                    f"alt={altitude_m:.2f}m descend ~{step_down:.2f}m "
                    f"at {descent_speed_ms:.2f} m/s for {duration_s:.2f}s"
                )
                if sampler:
                    sampler.set_command(
                        f"maze_soft_descent step={step + 1} vz={descent_speed_ms:.3f}"
                    )
                try:
                    client.moveByVelocityAsync(
                        0.0, 0.0, descent_speed_ms, duration_s
                    ).join()
                except Exception as exc:
                    print(
                        f"Maze soft landing: descent chunk failed ({type(exc).__name__}: {exc}); "
                        "hovering then final land.",
                        file=sys.stderr,
                    )
                    break
                if sampler:
                    sampler.set_command("hover_async")
                client.hoverAsync().join()
                if settle_s > 0:
                    time.sleep(settle_s)

            if pre_land_hover_s > 0:
                print(f"Maze soft landing: pre-land hover {pre_land_hover_s:.2f}s")
                time.sleep(pre_land_hover_s)
            if sampler:
                sampler.set_command("land_async")
            client.landAsync().join()
            return

        print(
            "Hover settle complete — next: controlled descent if above final altitude, "
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
                f"Controlled descent: speed={descent_speed_ms:.2f} m/s "
                f"for {descent_duration_s:.2f}s before final land."
            )
            if sampler:
                sampler.set_command(f"move_by_velocity vz_ms={descent_speed_ms:.3f}")
            if is_maze_mode() and hasattr(client, "moveByVelocityZAsync"):
                target_z = -final_land_altitude_m
                client.moveByVelocityZAsync(0.0, 0.0, target_z, descent_duration_s).join()
            else:
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
            print(f"Landing telemetry saved: {sampler.out_path}")


def _run_algorithm_with_timeout(algo, client, timeout_seconds: float) -> None:
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
        raise RuntimeError(
            f"Algorithm raised an exception after {elapsed_s:.1f}s"
        ) from exc


def main() -> None:
    config = load_config()
    _apply_low_end_overrides(config)
    maze_mode = is_maze_mode()
    if maze_mode:
        config.setdefault("vision", {})["enabled"] = False
        print("Maze mode (AIGP_MAZE=1): vision disabled for AirSim 1.x / UE 4.16 compatibility.")
    config["algorithm"] = choose_algorithm_name(config)
    sim_cfg = config["simulator"]
    host, port = simulator_endpoint(config)
    profile = os.environ.get("AIGP_PROFILE", "").strip()
    map_name = str(sim_cfg.get("map_name", "")).strip()
    print(
        "Flight session: "
        f"algorithm={config.get('algorithm', 'six_directions')!r} "
        f"rpc={host}:{port}"
        + (f" profile={profile!r}" if profile else "")
        + (f" map={map_name!r}" if map_name else "")
    )
    client = airsim.MultirotorClient(ip=host, port=port)
    vision_feed = VisionFeed(client, config.get("vision", {}))
    maze_flight = maze_mode
    maze_vn = "SimpleFlight"
    flight_client = client

    try:
        client.confirmConnection()
        if maze_flight:
            try_maze_unpause_and_clock(client)
            maze_vn = resolve_maze_vehicle_name(client)
            maze_enable_api_control(client, True, maze_vn)
            maze_arm_disarm(client, True, maze_vn)
            flight_client = MazeFlightClient(client, maze_vn)
        else:
            client.enableApiControl(True)
            client.armDisarm(True)
        _set_front_camera_pose(client, config)
        _apply_trace_style(client, config)

        try:
            vision_feed.start()
            algo_name = choose_algorithm_name(config)
            algo = get_algorithm(algo_name, config)
            algo.set_vision_feed(vision_feed if vision_feed.enabled else None)
            safety_cfg = config.get("safety", {})
            algo_timeout_seconds = max(
                5.0, float(safety_cfg.get("algorithm_timeout_seconds", 180.0))
            )

            print(f"Algorithm: {algo_name}")
            _run_algorithm_with_timeout(algo, flight_client, algo_timeout_seconds)

            print("Algorithm complete. Starting landing sequence...")
            _run_landing(flight_client, config)
            print("Flight client finished normally (landing complete).", file=sys.stderr)
        except Exception as exc:
            print(f"Failsafe triggered: {exc}")
            print("Attempting hover and landing for safe recovery...")
            _run_landing(flight_client, config)
    finally:
        vision_feed.stop()
        try:
            if maze_flight:
                maze_arm_disarm(client, False, maze_vn)
                maze_enable_api_control(client, False, maze_vn)
            else:
                client.armDisarm(False)
                client.enableApiControl(False)
        except Exception as cleanup_exc:
            if not _suppress_api_cleanup_warning(cleanup_exc):
                print(
                    "Warning: API cleanup failed (often harmless if sim/editor already "
                    f"closed): {cleanup_exc}",
                    file=sys.stderr,
                )

    if os.environ.get("AIGP_PAUSE_BEFORE_EXIT", "").strip() == "1":
        input("AIGP_PAUSE_BEFORE_EXIT=1 — press Enter to exit the flight client...")


if __name__ == "__main__":
    main()
