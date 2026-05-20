"""Shared MAVLink endpoint probing for launcher, preflight, and main."""

from __future__ import annotations

import os
import time


def _bridge_profile(config: dict) -> dict:
    mav_cfg = config.get("control", {}).get("mavlink", {})
    profile = mav_cfg.get("bridge_profile")
    if isinstance(profile, dict):
        return profile
    legacy = mav_cfg.get("simulator_bridge_profile")
    if isinstance(legacy, dict):
        return legacy
    return {}


def candidate_mavlink_endpoints(config: dict) -> list[str]:
    """Ordered UDP endpoints to probe for HEARTBEAT."""

    mav_cfg = config.get("control", {}).get("mavlink", {})
    bridge = _bridge_profile(config)
    qgc_port = int(bridge.get("qgc_port", 14550))

    raw_endpoint = str(mav_cfg.get("endpoint", f"udpin:0.0.0.0:{qgc_port}")).strip()
    endpoints: list[str] = [raw_endpoint, f"udpin:0.0.0.0:{qgc_port}"]

    occupied_ports = {
        int(bridge.get("udp_port", 14560)),
        int(bridge.get("control_port_local", 14540)),
        int(bridge.get("control_port_remote", 14580)),
    }

    raw_candidates = mav_cfg.get("endpoint_candidates", [])
    if isinstance(raw_candidates, list):
        for item in raw_candidates:
            if isinstance(item, int):
                if item not in occupied_ports:
                    endpoints.append(f"udpin:0.0.0.0:{item}")
                continue

            text = str(item).strip()
            if text.startswith("udpin:"):
                endpoints.append(text)
                continue
            if not text.startswith("udp:"):
                continue
            parts = text.split(":")
            if len(parts) == 3 and parts[2].isdigit():
                port = int(parts[2])
                if port not in occupied_ports:
                    endpoints.append(f"udpin:0.0.0.0:{port}")

    if bool(mav_cfg.get("auto_discover_endpoints", True)):
        for port in (14540, 14550, 14560, 14580, 5760, 5762):
            if port in occupied_ports:
                continue
            candidate = f"udpin:0.0.0.0:{port}"
            if candidate not in endpoints:
                endpoints.append(candidate)

    seen: set[str] = set()
    ordered: list[str] = []
    for endpoint in endpoints:
        if endpoint in seen:
            continue
        seen.add(endpoint)
        ordered.append(endpoint)
    return ordered


def first_mavlink_heartbeat_endpoint(config: dict, *, timeout_s: float) -> str | None:
    """Return first endpoint that yields a valid HEARTBEAT, or None."""

    from pymavlink import mavutil as _mavutil

    endpoints = candidate_mavlink_endpoints(config)
    deadline = time.time() + max(0.1, float(timeout_s))
    last_exc: str | None = None

    while time.time() < deadline:
        for endpoint in endpoints:
            connection = None
            try:
                connection = _mavutil.mavlink_connection(endpoint, autoreconnect=False)
                heartbeat = connection.wait_heartbeat(timeout=0.8)
                target_system = int(getattr(connection, "target_system", 0) or 0)
                if heartbeat is not None and target_system > 0:
                    return endpoint
            except Exception as exc:
                last_exc = str(exc)
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
        time.sleep(0.5)

    if last_exc:
        print(f"MAVLink heartbeat probe failed: {last_exc}")
    return None


def mavlink_endpoint_from_config(config: dict) -> str:
    mav_cfg = config.get("control", {}).get("mavlink", {})
    env_endpoint = os.environ.get("AIGP_MAVLINK_ENDPOINT", "").strip()
    if env_endpoint:
        return env_endpoint
    return str(mav_cfg.get("endpoint", "udpin:0.0.0.0:14550")).strip()


def pymavlink_flight_client_from_config(config: dict):
    from src.control.mavlink_client import PymavlinkFlightClient
    from src.mavlink.config import load_attitude_mavlink_config

    control_cfg = config.get("control", {})
    mav_cfg = control_cfg.get("mavlink", {})
    attitude_cfg = load_attitude_mavlink_config(config)
    timesync_cfg = mav_cfg.get("timesync", {})
    highres_imu_cfg = mav_cfg.get("highres_imu", {})
    endpoint = mavlink_endpoint_from_config(config)
    return PymavlinkFlightClient(
        endpoint=endpoint,
        command_rate_hz=float(control_cfg.get("command_rate_hz", 50.0)),
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
        highres_imu_max_staleness_ms=float(highres_imu_cfg.get("max_staleness_ms", 1000.0)),
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
        attitude_request_enabled=bool(attitude_cfg.get("enabled", True)),
        attitude_request_hz=float(attitude_cfg.get("request_hz", 50.0)),
    )


def resolve_control_transport(config: dict) -> str:
    """Flight control always uses MAVLink in this codebase."""
    _ = config
    return "mavlink"
