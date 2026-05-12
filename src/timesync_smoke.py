"""Live MAVLink TIMESYNC smoke test for simulator-backed sessions."""

from __future__ import annotations

import argparse
import os
import time

from src.config import apply_low_end_overrides, load_config
from src.control.mavlink_client import PymavlinkFlightClient
from src.mavlink_endpoints import resolve_control_transport


def _format_ms(value_ns: int | None) -> str:
    if value_ns is None:
        return "none"
    return f"{value_ns / 1_000_000.0:.3f}"


def _print_snapshot(label: str, snapshot) -> None:
    health = snapshot.sync_health
    print(
        f"[timesync-smoke] {label} messages={snapshot.message_count} "
        f"outbound={snapshot.outbound_request_count} "
        f"matched={snapshot.matched_response_count} "
        f"pending={snapshot.pending_request_count} "
        f"health={health.status} "
        f"stable_offset_ms={_format_ms(snapshot.stable_offset_ns)} "
        f"stable_rtt_ms={_format_ms(snapshot.stable_rtt_ns)} "
        f"jitter_ms={_format_ms(snapshot.offset_jitter_ns)} "
        f"reason={health.reason!r}"
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Connect to the live MAVLink endpoint and validate TIMESYNC behavior.",
    )
    parser.add_argument(
        "--endpoint",
        help="Explicit MAVLink endpoint, e.g. udpin:0.0.0.0:14550",
    )
    parser.add_argument(
        "--duration-seconds",
        type=float,
        help="How long to observe TIMESYNC traffic.",
    )
    parser.add_argument(
        "--poll-interval-seconds",
        type=float,
        help="How often to print health snapshots while the smoke test runs.",
    )
    parser.add_argument(
        "--min-matched-responses",
        type=int,
        help="Minimum matched TIMESYNC responses required for success.",
    )
    parser.add_argument(
        "--require-stable-health",
        action="store_true",
        help="Require the final sync health state to be stable.",
    )
    parser.add_argument(
        "--no-require-stable-health",
        action="store_true",
        help="Allow warming_up or degraded health while still requiring matched responses.",
    )
    parser.add_argument(
        "--verbose-timesync",
        action="store_true",
        help="Enable per-message TIMESYNC logging from the MAVLink client.",
    )
    return parser


def _timesync_client(config, args: argparse.Namespace) -> tuple[PymavlinkFlightClient, str, dict]:
    transport = resolve_control_transport(config)
    if transport != "mavlink":
        raise SystemExit(
            'TIMESYNC smoke test requires MAVLink transport. Set control.transport="mavlink" '
            "or run it via `uv run sim-timesync-smoke`."
        )

    control_cfg = config.get("control", {})
    mav_cfg = control_cfg.get("mavlink", {})
    timesync_cfg = mav_cfg.get("timesync", {})
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
        respond_to_timesync_requests=bool(timesync_cfg.get("respond_to_requests", True)),
        timesync_log_messages=bool(args.verbose_timesync),
        send_timesync_requests=bool(timesync_cfg.get("send_requests", True)),
        timesync_request_interval_s=float(timesync_cfg.get("request_interval_seconds", 1.0)),
        prepare_for_flight_on_connect=False,
        request_state_messages_on_connect=False,
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
    )
    return client, endpoint, timesync_cfg.get("smoke_test", {})


def main() -> None:
    args = _build_parser().parse_args()
    config = load_config()
    apply_low_end_overrides(config)
    client, endpoint, smoke_cfg = _timesync_client(config, args)

    duration_s = max(2.0, float(args.duration_seconds or smoke_cfg.get("duration_seconds", 12.0)))
    poll_s = max(
        0.2,
        float(args.poll_interval_seconds or smoke_cfg.get("poll_interval_seconds", 1.0)),
    )
    min_matched = max(
        1,
        int(args.min_matched_responses or smoke_cfg.get("min_matched_responses", 3)),
    )
    require_stable = bool(smoke_cfg.get("require_stable_health", True))
    if args.require_stable_health:
        require_stable = True
    if args.no_require_stable_health:
        require_stable = False

    print(
        "[timesync-smoke] Starting live TIMESYNC probe "
        f"endpoint={endpoint!r} duration_s={duration_s:.1f} "
        f"min_matched={min_matched} require_stable={require_stable}"
    )

    try:
        client.confirmConnection()
        deadline = time.monotonic() + duration_s
        next_poll = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_poll:
                _print_snapshot("progress", client.getTimesyncSnapshot())
                next_poll = now + poll_s
            time.sleep(0.05)

        snapshot = client.getTimesyncSnapshot()
        _print_snapshot("final", snapshot)
    finally:
        client.close()

    errors: list[str] = []
    if snapshot.matched_response_count < min_matched:
        errors.append(
            "matched response count below threshold "
            f"({snapshot.matched_response_count} < {min_matched})"
        )
    if require_stable and snapshot.sync_health.status != "stable":
        errors.append(f"final sync health is {snapshot.sync_health.status!r}, expected 'stable'")

    if errors:
        raise SystemExit("TIMESYNC smoke test FAILED: " + "; ".join(errors))

    print("TIMESYNC smoke test PASSED")


if __name__ == "__main__":
    main()
