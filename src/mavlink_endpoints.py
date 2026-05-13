"""Shared MAVLink endpoint probing for launcher, preflight, and main."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MavlinkHeartbeatProbeResult:
    endpoint: str | None
    attempted_endpoints: tuple[str, ...]
    last_error: str | None
    elapsed_s: float


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


def probe_mavlink_heartbeat(
    config: dict,
    *,
    timeout_s: float,
    connection_factory=None,
    heartbeat_timeout_s: float = 0.8,
    retry_sleep_s: float = 0.5,
) -> MavlinkHeartbeatProbeResult:
    """Probe candidate endpoints and return the first heartbeat-bearing MAVLink link."""

    from pymavlink import mavutil as _mavutil

    factory = connection_factory or _mavutil.mavlink_connection
    endpoints = tuple(candidate_mavlink_endpoints(config))
    started_s = time.monotonic()
    deadline = started_s + max(0.1, float(timeout_s))
    last_error: str | None = None

    while time.monotonic() < deadline:
        for endpoint in endpoints:
            connection = None
            try:
                connection = factory(endpoint, autoreconnect=False)
                heartbeat = connection.wait_heartbeat(timeout=heartbeat_timeout_s)
                target_system = int(getattr(connection, "target_system", 0) or 0)
                if heartbeat is not None and target_system > 0:
                    return MavlinkHeartbeatProbeResult(
                        endpoint=endpoint,
                        attempted_endpoints=endpoints,
                        last_error=None,
                        elapsed_s=time.monotonic() - started_s,
                    )
                if heartbeat is not None:
                    last_error = (
                        f"{endpoint}: heartbeat received but target_system={target_system}"
                    )
                else:
                    last_error = f"{endpoint}: no heartbeat within {heartbeat_timeout_s:.1f}s"
            except Exception as exc:
                last_error = f"{endpoint}: {exc}"
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass

        remaining_s = deadline - time.monotonic()
        if remaining_s > 0:
            time.sleep(min(retry_sleep_s, remaining_s))

    if last_error is None:
        last_error = "no MAVLink heartbeat observed before timeout"
    return MavlinkHeartbeatProbeResult(
        endpoint=None,
        attempted_endpoints=endpoints,
        last_error=last_error,
        elapsed_s=time.monotonic() - started_s,
    )


def first_mavlink_heartbeat_endpoint(config: dict, *, timeout_s: float) -> str | None:
    """Return first endpoint that yields a valid HEARTBEAT, or None."""

    return probe_mavlink_heartbeat(config, timeout_s=timeout_s).endpoint


def describe_mavlink_heartbeat_failure(
    config: dict,
    probe: MavlinkHeartbeatProbeResult,
) -> str:
    """Human-readable diagnosis for missing MAVLink heartbeats."""

    mav_cfg = config.get("control", {}).get("mavlink", {})
    profile = mav_cfg.get("airsim_profile", {})
    attempted = ", ".join(probe.attempted_endpoints) or "(none)"
    vehicle_type = str(profile.get("vehicle_type", "unknown")).strip() or "unknown"
    udp_port = int(profile.get("udp_port", 14560))
    control_port_local = int(profile.get("control_port_local", 14540))
    control_port_remote = int(profile.get("control_port_remote", 14580))
    qgc_port = int(profile.get("qgc_port", 14550))
    likely_causes = (
        "Likely causes: the simulator is still using AirSim-only transport, the AirSim "
        "MAVLink backend is not configured in settings.json, the endpoint/port is wrong, "
        "or the upstream autopilot bridge never started publishing heartbeats."
    )
    if _mavlink_profile_looks_like_simpleflight(config):
        likely_causes = (
            "Likely causes: the profile still looks like SimpleFlight/RpcLib-only, which "
            "will not emit PX4-style heartbeats, the endpoint/port is wrong, or the upstream "
            "autopilot bridge never started publishing heartbeats."
        )
    last_error = probe.last_error or "no probe error captured"
    return (
        "No MAVLink HEARTBEAT detected "
        f"within {probe.elapsed_s:.1f}s. Tried endpoints: {attempted}. "
        f"Last probe result: {last_error}. "
        "AirSim MAVLink profile: "
        f"vehicle_type={vehicle_type!r}, udp_port={udp_port}, "
        f"control_port_local={control_port_local}, "
        f"control_port_remote={control_port_remote}, qgc_port={qgc_port}. "
        f"{likely_causes}"
    )


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
