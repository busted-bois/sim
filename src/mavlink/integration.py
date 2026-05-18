"""Attach AttitudeTelemetryBridge to PymavlinkFlightClient."""

from __future__ import annotations

from typing import Any

from src.mavlink.attitude import AttitudeHealth, AttitudeSample
from src.mavlink.attitude_bridge import AttitudeTelemetryBridge


def attach_attitude_bridge(client: Any, bridge: AttitudeTelemetryBridge) -> None:
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
