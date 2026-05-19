import logging
import os
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(message)s")

import airsim  # noqa: E402
from src.config import apply_low_end_overrides, load_config, simulator_endpoint  # noqa: E402
from src.control.algorithms import get_algorithm, list_algorithms  # noqa: E402
from src.control.flight_client import AirSimAdapter  # noqa: E402
from src.control.highres_imu import format_highres_imu_health  # noqa: E402
from src.control.primitives import (  # noqa: E402
    apply_trace_style,
    land_with_telemetry,
    run_algorithm_with_timeout,
    set_front_camera_pose,
    suppress_api_cleanup_warning,
    wait_until_stationary,
)
from src.mavlink_endpoints import (  # noqa: E402
    pymavlink_flight_client_from_config,
    resolve_control_transport,
)
from src.simulator_specs import assert_specification_snapshot_if_required  # noqa: E402
from src.vision import VisionFeed  # noqa: E402

ROOT = Path(__file__).resolve().parent


def _format_timesync_ns_ms(value: int | None) -> str:
    if value is None:
        return "none"
    return f"{value / 1_000_000.0:.3f}"


def _format_timesync_event(event) -> str:
    if event is None:
        return "none"
    return f"tc1={event.tc1} ts1={event.ts1}"


def _log_timesync_status(client, label: str) -> None:
    getter = getattr(client, "getTimesyncSnapshot", None)
    if not callable(getter):
        return
    snapshot = getter()
    health = snapshot.sync_health
    print(
        f"[{label}] TIMESYNC messages={snapshot.message_count} "
        f"outbound={snapshot.outbound_request_count} "
        f"matched={snapshot.matched_response_count} "
        f"pending={snapshot.pending_request_count} "
        f"health={health.status} "
        f"reason={health.reason!r} "
        f"last_request={_format_timesync_event(snapshot.last_request)} "
        f"last_response={_format_timesync_event(snapshot.last_response)} "
        f"best_offset_ms={_format_timesync_ns_ms(snapshot.estimated_offset_ns)} "
        f"best_rtt_ms={_format_timesync_ns_ms(snapshot.estimated_rtt_ns)} "
        f"stable_offset_ms={_format_timesync_ns_ms(snapshot.stable_offset_ns)} "
        f"stable_rtt_ms={_format_timesync_ns_ms(snapshot.stable_rtt_ns)} "
        f"jitter_ms={_format_timesync_ns_ms(snapshot.offset_jitter_ns)}"
    )


def _apply_camera_and_trace(client, config) -> None:
    set_front_camera_pose(client, config)
    apply_trace_style(client, config)


def _log_highres_imu_status(client, label: str) -> None:
    health_getter = getattr(client, "getHighresImuHealth", None)
    sample_getter = getattr(client, "getHighresImu", None)
    if not callable(health_getter) or not callable(sample_getter):
        return
    health = health_getter()
    sample = sample_getter()
    sample_log = "sample=none"
    if sample is not None:
        sample_log = (
            f"sample(id={sample.sensor_id},time_usec={sample.time_usec},"
            f"xacc={sample.xacc},yacc={sample.yacc},zacc={sample.zacc},"
            f"xgyro={sample.xgyro},ygyro={sample.ygyro},zgyro={sample.zgyro})"
        )
    print(f"[{label}] HIGHRES_IMU {format_highres_imu_health(health)} {sample_log}")


def main() -> None:
    config = load_config()
    apply_low_end_overrides(config)
    assert_specification_snapshot_if_required(config)
    sim_cfg = config["simulator"]
    transport = resolve_control_transport(config)
    host, port = simulator_endpoint(config)
    profile = os.environ.get("AIGP_PROFILE", "").strip()
    map_name = str(sim_cfg.get("map_name", "")).strip()
    print(
        "Flight session: "
        f"algorithm={config.algorithm_name!r} "
        f"transport={transport!r} rpc={host}:{port}"
        + (f" profile={profile!r}" if profile else "")
        + (f" map={map_name!r}" if map_name else "")
    )

    airsim_client: airsim.MultirotorClient | None = None
    vision_feed: VisionFeed | None = None

    if transport == "mavlink":
        client = pymavlink_flight_client_from_config(config)
        allow_airsim_vision = (
            bool(config.get("vision", {}).get("enabled", False))
            and os.environ.get("AIGP_ENABLE_AIRSIM_VISION", "").strip() == "1"
        )
        if allow_airsim_vision:
            airsim_client = airsim.MultirotorClient(ip=host, port=port)
            vision_feed = VisionFeed(airsim_client, config.get("vision", {}))
        else:
            config.setdefault("vision", {})["enabled"] = False
    else:
        airsim_client = airsim.MultirotorClient(ip=host, port=port)
        client = AirSimAdapter(
            airsim_client,
            command_rate_hz=float(config.get("control", {}).get("command_rate_hz", 50.0)),
        )
        vision_feed = VisionFeed(airsim_client, config.get("vision", {}))

    try:
        client.confirmConnection()
        if transport == "airsim":
            try:
                client.reset()
            except Exception as reset_exc:
                print(f"Warning: client.reset() failed (continuing): {reset_exc}", file=sys.stderr)
        client.enableApiControl(True)
        client.armDisarm(True)
        if transport == "airsim":
            wait_until_stationary(client)
        _apply_camera_and_trace(client, config)
        if airsim_client is not None and airsim_client is not client:
            airsim_client.confirmConnection()
            _apply_camera_and_trace(airsim_client, config)
        _log_timesync_status(client, "startup")
        _log_highres_imu_status(client, "startup")

        try:
            if vision_feed is not None:
                vision_feed.start()
            algo_name = config.algorithm_name
            algo = get_algorithm(algo_name, config)
            algo.set_vision_feed(
                vision_feed if vision_feed is not None and vision_feed.enabled else None
            )
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
        if vision_feed is not None:
            vision_feed.stop()
        _log_timesync_status(client, "shutdown")
        _log_highres_imu_status(client, "shutdown")
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
        closer = getattr(client, "close", None)
        if callable(closer):
            closer()

    if os.environ.get("AIGP_PAUSE_BEFORE_EXIT", "").strip() == "1":
        input("AIGP_PAUSE_BEFORE_EXIT=1 -- press Enter to exit the flight client...")


if __name__ == "__main__":
    main()
