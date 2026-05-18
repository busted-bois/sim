"""Feed pymavlink ATTITUDE messages into AttitudeStore."""

from __future__ import annotations

import time
from typing import Any

from src.mavlink.attitude import AttitudeHealth, AttitudeSample
from src.mavlink.attitude_store import AttitudeStore, message_source_id
from src.mavlink.config import load_attitude_mavlink_config
from src.mavlink.messages import MAVLINK_MSG_ID_ATTITUDE


class AttitudeTelemetryBridge:
    def __init__(
        self,
        store: AttitudeStore,
        *,
        enabled: bool = True,
        request_hz: float = 50.0,
        max_staleness_ms: float = 500.0,
        log_messages: bool = False,
        transport: str = "mavlink",
    ) -> None:
        self._store = store
        self.enabled = bool(enabled)
        self.request_hz = max(1.0, float(request_hz))
        self.max_staleness_ms = max(1.0, float(max_staleness_ms))
        self.log_messages = bool(log_messages)
        self._transport = transport

    @classmethod
    def from_sim_config(
        cls,
        config: dict[str, Any],
        *,
        store: AttitudeStore | None = None,
    ) -> AttitudeTelemetryBridge:
        cfg = load_attitude_mavlink_config(config)
        return cls(
            store or AttitudeStore(),
            enabled=bool(cfg["enabled"]),
            request_hz=float(cfg["request_hz"]),
            max_staleness_ms=float(cfg["max_staleness_ms"]),
            log_messages=bool(cfg["log_messages"]),
        )

    def on_message(self, message: Any) -> None:
        if not self.enabled:
            return
        if getattr(message, "get_type", lambda: "")() != "ATTITUDE":
            return
        received_ns = time.monotonic_ns()
        sample = AttitudeSample(
            time_boot_ms=int(getattr(message, "time_boot_ms", 0)),
            roll=float(getattr(message, "roll", 0.0)),
            pitch=float(getattr(message, "pitch", 0.0)),
            yaw=float(getattr(message, "yaw", 0.0)),
            rollspeed=float(getattr(message, "rollspeed", 0.0)),
            pitchspeed=float(getattr(message, "pitchspeed", 0.0)),
            yawspeed=float(getattr(message, "yawspeed", 0.0)),
            source_system=message_source_id(message, "get_srcSystem"),
            source_component=message_source_id(message, "get_srcComponent"),
            local_received_monotonic_ns=received_ns,
            transport=self._transport,
        )
        self._store.update(sample)
        if self.log_messages:
            from src.mavlink.attitude import format_attitude_sample

            print(f"[mavlink] ATTITUDE {format_attitude_sample(sample)}")

    def request_interval(self, mav: Any) -> None:
        if not self.enabled:
            return
        message_interval_send = getattr(mav, "message_interval_send", None)
        if message_interval_send is None:
            return
        interval_us = int(1e6 / self.request_hz)
        message_interval_send(int(MAVLINK_MSG_ID_ATTITUDE), interval_us)

    def get_health(self) -> AttitudeHealth:
        return self._store.get_health(
            enabled=self.enabled,
            max_staleness_ms=self.max_staleness_ms,
            expected_rate_hz=self.request_hz,
        )

    def get_sample(self) -> AttitudeSample | None:
        return self._store.get()
