"""Hooks to attach ATTITUDE telemetry when PymavlinkFlightClient is available.

On branches with ``src/control/mavlink_client.py``, wire in four places:

1. ``attitude_cfg = load_attitude_mavlink_config(config)``
2. ``self._attitude = create_attitude_bridge(attitude_cfg)``
3. In ``_request_message_intervals()``: ``self._attitude.request_interval(self._mav.mav)``
4. In ``_telemetry_loop()``: add ``ATTITUDE`` to recv types; call ``on_message`` for that type
5. Expose ``getAttitude()`` / ``getAttitudeHealth()`` delegating to the bridge.
"""

from __future__ import annotations

from typing import Any

from src.mavlink.attitude import AttitudeHealth, AttitudeSample
from src.mavlink.attitude_bridge import AttitudeTelemetryBridge
from src.mavlink.attitude_store import AttitudeStore
from src.mavlink.config import load_attitude_mavlink_config


def create_attitude_bridge(attitude_cfg: dict[str, Any]) -> AttitudeTelemetryBridge:
    return AttitudeTelemetryBridge(
        AttitudeStore(),
        enabled=bool(attitude_cfg.get("enabled", True)),
        request_hz=float(attitude_cfg.get("request_hz", 50.0)),
        max_staleness_ms=float(attitude_cfg.get("max_staleness_ms", 500.0)),
        log_messages=bool(attitude_cfg.get("log_messages", False)),
    )


def create_attitude_bridge_from_config(config: dict[str, Any]) -> AttitudeTelemetryBridge:
    return create_attitude_bridge(load_attitude_mavlink_config(config))


def attach_attitude_bridge(client: Any, bridge: AttitudeTelemetryBridge) -> None:
    """Store bridge on client; call request_interval after MAVLink connect."""
    client._attitude = bridge


def get_attitude_sample(client: Any) -> AttitudeSample | None:
    bridge = getattr(client, "_attitude", None)
    if bridge is None:
        return None
    return bridge.get_sample()


def get_attitude_health(client: Any) -> AttitudeHealth | None:
    bridge = getattr(client, "_attitude", None)
    if bridge is None:
        return None
    return bridge.get_health()
