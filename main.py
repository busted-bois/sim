import logging
import os
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(message)s")

from src.config import apply_low_end_overrides, load_config, simulator_endpoint  # noqa: E402
from src.control.algorithms import get_algorithm, list_algorithms  # noqa: E402
from src.control.highres_imu import format_highres_imu_health  # noqa: E402
from src.control.mavlink_client import PymavlinkFlightClient  # noqa: E402
from src.control.primitives import (  # noqa: E402
    apply_trace_style,
    land_with_telemetry,
    run_algorithm_with_timeout,
    suppress_api_cleanup_warning,
)
from src.simulator_specs import assert_specification_snapshot_if_required  # noqa: E402

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
    host, port = simulator_endpoint(config)
    profile = os.environ.get("AIGP_PROFILE", "").strip()
    map_name = str(sim_cfg.get("map_name", "")).strip()
    print(
        "Flight session: "
        f"algorithm={config.algorithm_name!r} "
        f"transport='mavlink' rpc={host}:{port}"
        + (f" profile={profile!r}" if profile else "")
        + (f" map={map_name!r}" if map_name else "")
    )

    mav_cfg = config.get("control", {}).get("mavlink", {})
    timesync_cfg = mav_cfg.get("timesync", {})
    highres_imu_cfg = mav_cfg.get("highres_imu", {})
    endpoint = os.environ.get("AIGP_MAVLINK_ENDPOINT", "").strip() or str(
        mav_cfg.get("endpoint", "udpin:0.0.0.0:14550")
    ).strip()

    client = PymavlinkFlightClient(
        endpoint=endpoint,
        command_rate_hz=float(config.get("control", {}).get("command_rate_hz", 50.0)),
        state_request_hz=float(mav_cfg.get("state_request_hz", 20.0)),
        guided_custom_mode=int(mav_cfg.get("guided_custom_mode", 4)),
        takeoff_altitude_m=float(mav_cfg.get("takeoff_altitude_m", 5.0)),
        land_descent_speed_ms=float(config.get("landing", {}).get("descent_speed_ms", 2.0)),
        source_system=int(mav_cfg.get("source_system", 255)),
        source_component=int(mav_cfg.get("source_component", 1)),
        respond_to_timesync_requests=bool(timesync_cfg.get("respond_to_requests", True)),
        timesync_log_messages=bool(timesync_cfg.get("log_messages", True)),
        send_timesync_requests=bool(timesync_cfg.get("send_requests", True)),
        timesync_request_interval_s=float(timesync_cfg.get("request_interval_seconds", 1.0)),
        highres_imu_enabled=bool(highres_imu_cfg.get("enabled", True)),
        highres_imu_request_hz=float(
            highres_imu_cfg.get("request_hz", mav_cfg.get("state_request_hz", 20.0))
        ),
        highres_imu_log_messages=bool(highres_imu_cfg.get("log_messages", False)),
        highres_imu_max_staleness_ms=float(
            highres_imu_cfg.get("max_staleness_ms", 1000.0)
        ),
        timesync_pending_request_limit=int(timesync_cfg.get("pending_request_limit", 64)),
        timesync_stable_window_size=int(timesync_cfg.get("stable_window_size", 9)),
        timesync_stable_best_subset_size=int(timesync_cfg.get("stable_best_subset_size", 5)),
        timesync_min_stable_samples=int(timesync_cfg.get("min_stable_samples", 3)),
        timesync_max_stable_rtt_ns=int(
            float(timesync_cfg.get("max_stable_rtt_ms", 250.0)) * 1_000_000
        ),
        timesync_max_offset_jitter_ns=int(
            float(timesync_cfg.get("max_offset_jitter_ms", 50.0)) * 1_000_000
        ),
        attitude_target_throttle_body_z=bool(
            mav_cfg.get("attitude_target", {}).get("throttle_body_z", False)
        ),
    )

    try:
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
        apply_trace_style(client, config)
        _log_timesync_status(client, "startup")
        _log_highres_imu_status(client, "startup")

        try:
            algo_name = config.algorithm_name
            algo = get_algorithm(algo_name, config)
            algo.set_vision_feed(None)
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
