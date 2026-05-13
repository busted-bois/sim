"""Live MAVLink HIGHRES_IMU smoke test for simulator-backed sessions."""

from __future__ import annotations

import argparse
import os
import time

from src.config import apply_low_end_overrides, load_config
from src.control.highres_imu import format_highres_imu_health, format_highres_imu_sample
from src.control.mavlink_client import PymavlinkFlightClient
from src.mavlink_endpoints import (
    describe_mavlink_heartbeat_failure,
    probe_mavlink_heartbeat,
    resolve_control_transport,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Connect to the live MAVLink endpoint and validate HIGHRES_IMU behavior.",
    )
    parser.add_argument(
        "--endpoint",
        help="Explicit MAVLink endpoint, e.g. udpin:0.0.0.0:14550",
    )
    parser.add_argument(
        "--duration-seconds",
        type=float,
        help="How long to observe HIGHRES_IMU traffic.",
    )
    parser.add_argument(
        "--poll-interval-seconds",
        type=float,
        help="How often to print health snapshots while the smoke test runs.",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        help="Minimum HIGHRES_IMU samples required for success.",
    )
    parser.add_argument(
        "--require-fresh",
        action="store_true",
        help="Require the final HIGHRES_IMU health status to be 'ok'.",
    )
    parser.add_argument(
        "--no-require-fresh",
        action="store_true",
        help="Allow stale health so long as the sample count threshold is met.",
    )
    parser.add_argument(
        "--verbose-highres-imu",
        action="store_true",
        help="Enable per-message HIGHRES_IMU logging from the MAVLink client.",
    )
    return parser


def _print_snapshot(label: str, client: PymavlinkFlightClient) -> None:
    health = client.getHighresImuHealth()
    sample = client.getHighresImu()
    print(
        f"[highres-imu-smoke] {label} "
        f"{format_highres_imu_health(health)} {format_highres_imu_sample(sample)}"
    )


def _highres_imu_client(
    config,
    args: argparse.Namespace,
) -> tuple[PymavlinkFlightClient, str, dict]:
    transport = resolve_control_transport(config)
    if transport != "mavlink":
        raise SystemExit(
            "HIGHRES_IMU smoke test requires MAVLink transport, but the current session "
            f"resolved to {transport!r}. Set control.transport=\"mavlink\", or run it via "
            "`uv run sim-highres-imu-smoke` so the launcher can establish a MAVLink heartbeat."
        )

    control_cfg = config.get("control", {})
    mav_cfg = control_cfg.get("mavlink", {})
    imu_cfg = mav_cfg.get("highres_imu", {})
    capture_cfg = imu_cfg.get("capture_log", {})
    endpoint = (
        (args.endpoint or "").strip()
        or os.environ.get("AIGP_MAVLINK_ENDPOINT", "").strip()
        or str(mav_cfg.get("endpoint", "udpin:0.0.0.0:14550")).strip()
    )
    client = PymavlinkFlightClient(
        endpoint=endpoint,
        command_rate_hz=float(control_cfg.get("command_rate_hz", 50.0)),
        state_request_hz=float(mav_cfg.get("state_request_hz", 20.0)),
        guided_custom_mode=int(mav_cfg.get("guided_custom_mode", 4)),
        takeoff_altitude_m=float(mav_cfg.get("takeoff_altitude_m", 5.0)),
        land_descent_speed_ms=float(config.get("landing", {}).get("descent_speed_ms", 2.0)),
        source_system=int(mav_cfg.get("source_system", 255)),
        source_component=int(mav_cfg.get("source_component", 1)),
        respond_to_timesync_requests=False,
        timesync_log_messages=False,
        send_timesync_requests=False,
        prepare_for_flight_on_connect=False,
        request_state_messages_on_connect=True,
        highres_imu_enabled=bool(imu_cfg.get("enabled", True)),
        highres_imu_request_hz=float(
            imu_cfg.get("request_hz", mav_cfg.get("state_request_hz", 20.0))
        ),
        highres_imu_log_messages=bool(args.verbose_highres_imu),
        highres_imu_max_staleness_ms=float(imu_cfg.get("max_staleness_ms", 1000.0)),
        highres_imu_summary_interval_s=float(imu_cfg.get("summary_interval_seconds", 5.0)),
        highres_imu_warn_on_stale=bool(imu_cfg.get("warn_on_stale", True)),
        highres_imu_capture_path=(
            str(capture_cfg.get("path", "logs/highres_imu_capture.csv")).strip()
            if bool(capture_cfg.get("enabled", False))
            else None
        ),
    )
    return client, endpoint, imu_cfg.get("smoke_test", {})


def _confirm_connection(client: PymavlinkFlightClient, config) -> None:
    try:
        client.confirmConnection()
    except TimeoutError as exc:
        probe = probe_mavlink_heartbeat(config, timeout_s=2.0)
        failure_detail = describe_mavlink_heartbeat_failure(config, probe)
        raise SystemExit(f"{exc} {failure_detail}") from exc


def main() -> None:
    args = _build_parser().parse_args()
    config = load_config()
    apply_low_end_overrides(config)
    client, endpoint, smoke_cfg = _highres_imu_client(config, args)

    duration_s = max(2.0, float(args.duration_seconds or smoke_cfg.get("duration_seconds", 8.0)))
    poll_s = max(
        0.2,
        float(args.poll_interval_seconds or smoke_cfg.get("poll_interval_seconds", 1.0)),
    )
    min_samples = max(1, int(args.min_samples or smoke_cfg.get("min_samples", 3)))
    require_fresh = bool(smoke_cfg.get("require_fresh", True))
    if args.require_fresh:
        require_fresh = True
    if args.no_require_fresh:
        require_fresh = False

    print(
        "[highres-imu-smoke] Starting live HIGHRES_IMU probe "
        f"endpoint={endpoint!r} duration_s={duration_s:.1f} "
        f"min_samples={min_samples} require_fresh={require_fresh}"
    )

    try:
        _confirm_connection(client, config)
        deadline = time.monotonic() + duration_s
        next_poll = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_poll:
                _print_snapshot("progress", client)
                next_poll = now + poll_s
            time.sleep(0.05)
        _print_snapshot("final", client)
        health = client.getHighresImuHealth()
    finally:
        client.close()

    errors: list[str] = []
    if health is None:
        errors.append("client does not expose HIGHRES_IMU health")
    else:
        if health.sample_count < min_samples:
            errors.append(f"sample count below threshold ({health.sample_count} < {min_samples})")
        if require_fresh and health.status != "ok":
            errors.append(f"final HIGHRES_IMU health is {health.status!r}, expected 'ok'")

    if errors:
        raise SystemExit("HIGHRES_IMU smoke test FAILED: " + "; ".join(errors))

    print("HIGHRES_IMU smoke test PASSED")


if __name__ == "__main__":
    main()
