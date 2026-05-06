import os
import sys
from pathlib import Path

import airsim
from src.config import apply_low_end_overrides, load_config, simulator_endpoint
from src.control.algorithms import get_algorithm, list_algorithms
from src.control.flight_client import AirSimAdapter
from src.control.primitives import (
    apply_trace_style,
    land_with_telemetry,
    run_algorithm_with_timeout,
    set_front_camera_pose,
    suppress_api_cleanup_warning,
    wait_until_stationary,
)
from src.vision import VisionFeed

ROOT = Path(__file__).resolve().parent


def main() -> None:
    config = load_config()
    apply_low_end_overrides(config)
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
    airsim_client = airsim.MultirotorClient(ip=host, port=port)
    client = AirSimAdapter(airsim_client)
    vision_feed = VisionFeed(airsim_client, config.get("vision", {}))

    try:
        client.confirmConnection()
        try:
            client.reset()
        except Exception as reset_exc:
            print(f"Warning: client.reset() failed (continuing): {reset_exc}", file=sys.stderr)
        client.enableApiControl(True)
        client.armDisarm(True)
        wait_until_stationary(client)
        set_front_camera_pose(client, config)
        apply_trace_style(client, config)

        try:
            vision_feed.start()
            algo_name = config.get("algorithm", "six_directions")
            algo = get_algorithm(algo_name, config)
            algo.set_vision_feed(vision_feed if vision_feed.enabled else None)
            safety_cfg = config.get("safety", {})
            algo_timeout_seconds = max(
                5.0, float(safety_cfg.get("algorithm_timeout_seconds", 180.0))
            )

            print(f"Algorithm: {algo_name} (available: {', '.join(list_algorithms())})")
            run_algorithm_with_timeout(algo, client, algo_timeout_seconds)

            print("Algorithm complete. Starting landing sequence...")
            land_with_telemetry(client, config, label="main")
            print("Flight client finished normally (landing complete).", file=sys.stderr)
        except Exception as exc:
            print(f"Failsafe triggered: {exc}")
            print("Attempting hover and landing for safe recovery...")
            land_with_telemetry(client, config, label="main")
    finally:
        vision_feed.stop()
        try:
            client.armDisarm(False)
            client.enableApiControl(False)
        except Exception as cleanup_exc:
            if not suppress_api_cleanup_warning(cleanup_exc):
                print(
                    "Warning: API cleanup failed (often harmless if sim/editor already "
                    f"closed): {cleanup_exc}",
                    file=sys.stderr,
                )

    if os.environ.get("AIGP_PAUSE_BEFORE_EXIT", "").strip() == "1":
        input("AIGP_PAUSE_BEFORE_EXIT=1 — press Enter to exit the flight client...")


if __name__ == "__main__":
    main()
