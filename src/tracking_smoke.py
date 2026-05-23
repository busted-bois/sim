"""Smoke test for LocalTracker + optional live MAVLink session."""

from __future__ import annotations

import os
import time

from src.config import load_config
from src.mavlink_endpoints import resolve_control_transport
from src.tracking import local_tracker_from_config

ROOT = __import__("pathlib").Path(__file__).resolve().parent.parent


def main() -> None:
    config = load_config()
    transport = resolve_control_transport(config)
    if transport != "mavlink":
        raise SystemExit(
            'tracking-smoke requires control.transport="mavlink" and a running PX4/MAVLink bridge.'
        )

    tracker = local_tracker_from_config(config, ROOT)
    if tracker is None:
        raise SystemExit("control.mavlink.tracking.enabled is false in sim.config.json")

    from src.control.mavlink_client import PymavlinkFlightClient

    mav_cfg = config.get("control", {}).get("mavlink", {})
    endpoint = os.environ.get("AIGP_MAVLINK_ENDPOINT", "").strip() or str(
        mav_cfg.get("endpoint", "udpin:0.0.0.0:14550")
    )
    client = PymavlinkFlightClient(
        endpoint=endpoint,
        local_tracker=tracker,
        highres_imu_enabled=True,
        highres_imu_request_hz=float(mav_cfg.get("highres_imu", {}).get("request_hz", 120.0)),
        prepare_for_flight_on_connect=False,
    )
    duration_s = float(mav_cfg.get("tracking", {}).get("smoke_duration_seconds", 10.0))
    print(f"[tracking-smoke] Probing {endpoint} for {duration_s:.0f}s...")
    client.confirmConnection()
    deadline = time.time() + duration_s
    while time.time() < deadline:
        time.sleep(0.5)
        health = tracker.health()
        print(
            f"[tracking-smoke] status={health.status} imu={health.imu_sample_count} "
            f"rate={health.imu_rate_hz} origin={health.origin_set}"
        )
    client.close()
    tracker.flush()
    health = tracker.health()
    if health.imu_sample_count < 3:
        raise SystemExit(
            f"tracking-smoke FAILED: only {health.imu_sample_count} IMU samples "
            "(need PX4-SITL + sim-mavlink)"
        )
    print(f"[tracking-smoke] PASSED csv={tracker.csv_path}")


if __name__ == "__main__":
    main()
