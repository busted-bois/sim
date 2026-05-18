"""Load MAVLink ATTITUDE settings from sim.config.json."""

from __future__ import annotations

from typing import Any

_DEFAULT_ATTITUDE: dict[str, Any] = {
    "enabled": True,
    "request_hz": 50.0,
    "max_staleness_ms": 500.0,
    "log_messages": False,
}


def load_attitude_mavlink_config(config: dict[str, Any]) -> dict[str, Any]:
    """Return ``control.mavlink.attitude`` merged with defaults."""
    control = config.get("control")
    if not isinstance(control, dict):
        return dict(_DEFAULT_ATTITUDE)
    mavlink = control.get("mavlink")
    if not isinstance(mavlink, dict):
        return dict(_DEFAULT_ATTITUDE)
    attitude = mavlink.get("attitude")
    if not isinstance(attitude, dict):
        return dict(_DEFAULT_ATTITUDE)
    merged = dict(_DEFAULT_ATTITUDE)
    merged.update(attitude)
    merged["enabled"] = bool(merged.get("enabled", True))
    merged["request_hz"] = max(1.0, float(merged.get("request_hz", 50.0)))
    merged["max_staleness_ms"] = max(1.0, float(merged.get("max_staleness_ms", 500.0)))
    merged["log_messages"] = bool(merged.get("log_messages", False))
    return merged
