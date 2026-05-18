"""MAVLink ATTITUDE (#30) decode and health types."""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass

_ATTITUDE_PAYLOAD_LEN = 28
_ATTITUDE_STRUCT = struct.Struct("<Iffffff")


@dataclass(frozen=True, slots=True)
class AttitudeSample:
    time_boot_ms: int
    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float
    source_system: int | None = None
    source_component: int | None = None
    local_received_monotonic_ns: int = 0
    transport: str = "mavlink"

    def age_ms(self, now_monotonic_ns: int | None = None) -> float:
        now_ns = time_monotonic_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
        return max(0.0, (now_ns - self.local_received_monotonic_ns) / 1_000_000.0)


@dataclass(frozen=True, slots=True)
class AttitudeHealth:
    status: str
    reason: str
    enabled: bool
    sample_count: int
    stream_rate_hz: float | None
    update_age_ms: float | None
    max_staleness_ms: float
    expected_rate_hz: float | None = None

    def is_stale(self) -> bool:
        return self.status == "stale"

    def is_healthy(self) -> bool:
        return self.status == "ok"


def decode_attitude_payload(payload: bytes) -> AttitudeSample | None:
    if len(payload) < _ATTITUDE_PAYLOAD_LEN:
        return None
    (
        time_boot_ms,
        roll,
        pitch,
        yaw,
        rollspeed,
        pitchspeed,
        yawspeed,
    ) = _ATTITUDE_STRUCT.unpack_from(payload, 0)
    return AttitudeSample(
        time_boot_ms=int(time_boot_ms),
        roll=float(roll),
        pitch=float(pitch),
        yaw=float(yaw),
        rollspeed=float(rollspeed),
        pitchspeed=float(pitchspeed),
        yawspeed=float(yawspeed),
    )


def roll_pitch_yaw_deg(sample: AttitudeSample) -> tuple[float, float, float]:
    return (
        math.degrees(sample.roll),
        math.degrees(sample.pitch),
        math.degrees(sample.yaw),
    )


def format_attitude_sample(sample: AttitudeSample | None) -> str:
    if sample is None:
        return "sample=none"
    roll_d, pitch_d, yaw_d = roll_pitch_yaw_deg(sample)
    return (
        "sample("
        f"time_boot_ms={sample.time_boot_ms},"
        f"rpy_deg=({roll_d:.2f},{pitch_d:.2f},{yaw_d:.2f}),"
        f"rates=({sample.rollspeed:.3f},{sample.pitchspeed:.3f},{sample.yawspeed:.3f}),"
        f"age_ms={sample.age_ms():.1f})"
    )


def format_attitude_health(health: AttitudeHealth | None) -> str:
    if health is None:
        return "unsupported"
    rate_text = "none" if health.stream_rate_hz is None else f"{health.stream_rate_hz:.2f}"
    age_text = "none" if health.update_age_ms is None else f"{health.update_age_ms:.1f}"
    expected_text = "none"
    if health.expected_rate_hz is not None:
        expected_text = f"{health.expected_rate_hz:.2f}"
    return (
        f"status={health.status} reason={health.reason!r} enabled={health.enabled} "
        f"samples={health.sample_count} rate_hz={rate_text} age_ms={age_text} "
        f"expected_rate_hz={expected_text} max_staleness_ms={health.max_staleness_ms:.1f}"
    )


def time_monotonic_ns() -> int:
    return time.monotonic_ns()
