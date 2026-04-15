import os
import threading
import time
from pathlib import Path

import airsim
from src.config import load_config
from src.control.algorithms import get_algorithm
from src.landing_telemetry import LandingTelemetrySampler
from src.vision import VisionFeed

ROOT = Path(__file__).resolve().parent


def _set_front_camera_pose(client: airsim.MultirotorClient, config: dict) -> None:
    vision_cfg = config.get("vision", {})
    camera_name = str(vision_cfg.get("camera_name", "0"))
    # NED frame: +X is forward, +Y is right, +Z is down.
    front_pose = airsim.Pose(
        airsim.Vector3r(0.35, 0.0, -0.05),
        airsim.Quaternionr(0.0, 0.0, 0.0, 1.0),
    )
    try:
        client.simSetCameraPose(camera_name, front_pose)
    except Exception as exc:  # noqa: BLE001
        print(f"Warning: failed to set front camera pose for '{camera_name}': {exc}")


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
            if sampler:
                sampler.set_command("land_async")
            client.landAsync().join()
            return

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
            # NED frame: +vz commands downward motion.
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
        except BaseException as exc:  # noqa: BLE001
            error_holder["error"] = exc

    worker = threading.Thread(target=_target, name="algorithm_runner", daemon=True)
    worker.start()
    worker.join(timeout=timeout_seconds)

    if worker.is_alive():
        raise TimeoutError(f"Algorithm timed out after {timeout_seconds:.1f}s")

    if "error" in error_holder:
        raise RuntimeError("Algorithm raised an exception") from error_holder["error"]


def main() -> None:
    config = load_config()
    _apply_low_end_overrides(config)
    sim_cfg = config["simulator"]

    client = airsim.MultirotorClient(
        ip=sim_cfg.get("host", "127.0.0.1"),
        port=sim_cfg.get("airsim_port", 41451),
    )
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    _set_front_camera_pose(client, config)
    vision_feed = VisionFeed(client, config.get("vision", {}))

    try:
        vision_feed.start()
        algo_name = config.get("algorithm", "six_directions")
        algo = get_algorithm(algo_name, config)
        algo.set_vision_feed(vision_feed if vision_feed.enabled else None)
        safety_cfg = config.get("safety", {})
        algo_timeout_seconds = max(5.0, float(safety_cfg.get("algorithm_timeout_seconds", 180.0)))

        print(f"Algorithm: {algo_name}")
        _run_algorithm_with_timeout(algo, client, algo_timeout_seconds)

        print("Algorithm complete. Starting landing sequence...")
        _run_landing(client, config)
    except Exception as exc:
        print(f"Failsafe triggered: {exc}")
        print("Attempting hover and landing for safe recovery...")
        _run_landing(client, config)
    finally:
        vision_feed.stop()
        client.armDisarm(False)
        client.enableApiControl(False)


if __name__ == "__main__":
    main()
