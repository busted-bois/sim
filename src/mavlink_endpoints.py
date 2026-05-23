"""Shared MAVLink endpoint probing for launcher, preflight, and main."""

from __future__ import annotations

import os
import time


def _mavlink_profile_looks_like_simpleflight(config: dict) -> bool:
    """True when MAVLink profile is configured with SimpleFlight semantics."""
    mav_cfg = config.get("control", {}).get("mavlink", {})
    profile = mav_cfg.get("airsim_profile", {})
    vehicle_type = str(profile.get("vehicle_type", "")).strip().lower()
    firmware_name = str(profile.get("firmware_name", "")).strip().lower()
    return vehicle_type == "simpleflight" or firmware_name == "simpleflight"


def _coerce_transport_with_guardrails(config: dict, requested: str) -> str:
    normalized = requested.strip().lower()
    if normalized in {"airsim", "mavlink", "auto"}:
        if (
            normalized == "mavlink"
            and _mavlink_profile_looks_like_simpleflight(config)
            and os.environ.get("AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT", "").strip() != "1"
        ):
            print(
                'Warning: control.transport="mavlink" with SimpleFlight profile is usually '
                "RpcLib-only and can block launch waiting for HEARTBEAT. "
                "Falling back to 'airsim'. Set AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT=1 to force MAVLink."
            )
            return "airsim"
        return normalized

    print(
        f"Warning: unsupported control.transport={requested!r}; falling back to 'airsim'. "
        "Supported values: 'airsim', 'mavlink', 'auto'."
    )
    return "airsim"


def candidate_mavlink_endpoints(config: dict) -> list[str]:
    """Ordered UDP endpoints to probe for HEARTBEAT."""

    mav_cfg = config.get("control", {}).get("mavlink", {})
    airsim_profile = mav_cfg.get("airsim_profile", {})
    qgc_port = int(airsim_profile.get("qgc_port", 14550))

    raw_endpoint = str(mav_cfg.get("endpoint", f"udpin:0.0.0.0:{qgc_port}")).strip()
    endpoints: list[str] = [raw_endpoint, f"udpin:0.0.0.0:{qgc_port}"]

    occupied_ports = {
        int(airsim_profile.get("udp_port", 14560)),
        int(airsim_profile.get("control_port_local", 14540)),
        int(airsim_profile.get("control_port_remote", 14580)),
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
    probe_round = 0

    while time.time() < deadline:
        probe_round += 1
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


def resolve_control_transport(config: dict) -> str:
    """Resolve the effective control transport for this session."""

    env_value = os.environ.get("AIGP_CONTROL_TRANSPORT", "").strip().lower()
    raw = (
        env_value
        if env_value
        else str(config.get("control", {}).get("transport", "airsim")).strip().lower()
    )
    requested = _coerce_transport_with_guardrails(config, raw)
    if requested == "auto":
        if first_mavlink_heartbeat_endpoint(config, timeout_s=2.0) is not None:
            return "mavlink"
        return "airsim"
    return requested
