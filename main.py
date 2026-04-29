import errno
import os
import sys
import threading
import time
import traceback
from pathlib import Path

import airsim
from msgpackrpc.error import RPCError, TransportError
from src.config import load_config, simulator_endpoint
from src.control.algorithms import get_algorithm
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
    try:
        client.simSetCameraPose(camera_name, front_pose)
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
    try:
        client.simSetTraceLine(color, thickness, vehicle_name)
        print(f"Trace style applied: color={color}, thickness={thickness:.1f}")
    except Exception as exc:  # noqa: BLE001
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
    landing_cfg = config.get("landing", {})
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

    # Daemon: if the main thread exits (e.g. Ctrl+C), CPython must not hang waiting on this
    # thread at interpreter shutdown. Main still blocks on join until the algorithm completes
    # or times out.
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


def main() -> None:
    config = load_config()
    _apply_low_end_overrides(config)
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

    try:
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        _set_front_camera_pose(client, config)
        _apply_trace_style(client, config)

        try:
            vision_feed.start()
            algo_name = config.get("algorithm", "six_directions")
            algo = get_algorithm(algo_name, config)
            algo.set_vision_feed(vision_feed if vision_feed.enabled else None)
            safety_cfg = config.get("safety", {})
            algo_timeout_seconds = max(
                5.0, float(safety_cfg.get("algorithm_timeout_seconds", 180.0))
            )

            print(f"Algorithm: {algo_name}")
            _run_algorithm_with_timeout(algo, client, algo_timeout_seconds)

            print("Algorithm complete. Starting landing sequence...")
            _run_landing(client, config)
            print("Flight client finished normally (landing complete).", file=sys.stderr)
        except Exception as exc:
            print(f"Failsafe triggered: {exc}")
            print("Attempting hover and landing for safe recovery...")
            _run_landing(client, config)
    finally:
        vision_feed.stop()
        try:
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
