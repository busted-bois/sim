"""Connect over MAVLink, stream a short attitude command, print command-rate stats."""

from __future__ import annotations

import argparse
import os

from src.config import apply_low_end_overrides, load_config
from src.control.mavlink_client import PymavlinkFlightClient
from src.mavlink_endpoints import resolve_control_transport


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Connect over MAVLink, send a brief SET_ATTITUDE_TARGET stream, report stats.",
    )
    parser.add_argument(
        "--endpoint",
        help="MAVLink endpoint, e.g. udpin:0.0.0.0:14550",
    )
    parser.add_argument(
        "--duration-seconds",
        type=float,
        default=0.35,
        help="How long to stream the attitude command (default: 0.35).",
    )
    return parser


def _client(config, args: argparse.Namespace) -> tuple[PymavlinkFlightClient, str]:
    transport = resolve_control_transport(config)
    if transport != "mavlink":
        raise SystemExit(
            'Attitude smoke requires MAVLink transport. Set control.transport="mavlink".'
        )
    control_cfg = config.get("control", {})
    mav_cfg = control_cfg.get("mavlink", {})
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
        highres_imu_enabled=False,
    )
    return client, endpoint


def main() -> None:
    args = _build_parser().parse_args()
    config = load_config()
    apply_low_end_overrides(config)
    client, endpoint = _client(config, args)
    duration_s = max(0.05, float(args.duration_seconds))
    print(
        f"[attitude-smoke] endpoint={endpoint!r} duration_s={duration_s:.2f} "
        "(small roll setpoint, throttle ~0.55)"
    )
    try:
        client.confirmConnection()
        before = client.getCommandRateStats()
        client.moveByRollPitchYawThrottleAsync(0.05, 0.0, 0.0, 0.55, duration_s).join()
        after = client.getCommandRateStats()
    finally:
        client.close()

    attempted_delta = after.attempted_count - before.attempted_count
    allowed_delta = after.allowed_count - before.allowed_count
    print(
        "[attitude-smoke] command_rate gate: "
        f"attempted +{attempted_delta}, allowed +{allowed_delta}"
    )
    if allowed_delta <= 0:
        raise SystemExit("Attitude smoke FAILED: no gated motion commands were allowed.")
    print("Attitude smoke PASSED")


if __name__ == "__main__":
    main()
