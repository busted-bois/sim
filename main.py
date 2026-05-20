import logging
import os
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(message)s")

from src.config import apply_low_end_overrides, load_config  # noqa: E402
from src.control.algorithms import get_algorithm, list_algorithms  # noqa: E402
from src.control.highres_imu import format_highres_imu_health  # noqa: E402
from src.control.main_loop import run_algorithm_with_timeout  # noqa: E402
from src.control.primitives import (  # noqa: E402
    apply_trace_style,
    land_with_telemetry,
    suppress_api_cleanup_warning,
)
from src.mavlink_endpoints import (  # noqa: E402
    mavlink_endpoint_from_config,
    pymavlink_flight_client_from_config,
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
    endpoint = mavlink_endpoint_from_config(config)
    profile = os.environ.get("AIGP_PROFILE", "").strip()
    map_name = str(sim_cfg.get("map_name", "")).strip()
    print(
        "Flight session: "
        f"algorithm={config.algorithm_name!r} "
        f"transport=mavlink endpoint={endpoint!r}"
        + (f" profile={profile!r}" if profile else "")
        + (f" map={map_name!r}" if map_name else "")
    )

    vision_cfg = config.get("vision", {})
    if bool(vision_cfg.get("enabled", False)):
        print(
            "Warning: vision.enabled is true but camera capture is stubbed; "
            "disabling vision for this session.",
            file=sys.stderr,
        )
        vision_cfg["enabled"] = False

    client = pymavlink_flight_client_from_config(config)
    vision_feed = VisionFeed(None, vision_cfg)

    try:
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        apply_trace_style(client, config)
        _log_timesync_status(client, "startup")
        _log_highres_imu_status(client, "startup")

        try:
            if vision_feed.enabled:
                vision_feed.start()
            algo_name = config.algorithm_name
            algo = get_algorithm(algo_name, config)
            algo.set_vision_feed(vision_feed if vision_feed.enabled else None)
            safety_cfg = config.get("safety", {})
            algo_timeout_seconds = max(
                5.0, float(safety_cfg.get("algorithm_timeout_seconds", 180.0))
            )
            command_rate_hz = float(config.get("control", {}).get("command_rate_hz", 50.0))

            print(f"Algorithm: {algo_name} (available: {', '.join(list_algorithms())})")
            run_algorithm_with_timeout(
                algo,
                client,
                algo_timeout_seconds,
                vision_feed=vision_feed if vision_feed.enabled else None,
                command_rate_hz=command_rate_hz,
            )

            print("Algorithm complete. Starting landing sequence...")
            land_with_telemetry(client, config, label="main")
            print("Flight client finished normally (landing complete).", file=sys.stderr)
        except Exception as exc:
            print(f"Failsafe triggered: {exc}")
            print("Attempting hover and landing for safe recovery...")
            land_with_telemetry(client, config, label="main")
    finally:
        if vision_feed.enabled:
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
