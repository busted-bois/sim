from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

HIGHRES_IMU_UPDATED_XACC = 1 << 0
HIGHRES_IMU_UPDATED_YACC = 1 << 1
HIGHRES_IMU_UPDATED_ZACC = 1 << 2
HIGHRES_IMU_UPDATED_XGYRO = 1 << 3
HIGHRES_IMU_UPDATED_YGYRO = 1 << 4
HIGHRES_IMU_UPDATED_ZGYRO = 1 << 5
HIGHRES_IMU_UPDATED_XMAG = 1 << 6
HIGHRES_IMU_UPDATED_YMAG = 1 << 7
HIGHRES_IMU_UPDATED_ZMAG = 1 << 8
HIGHRES_IMU_UPDATED_ABS_PRESSURE = 1 << 9
HIGHRES_IMU_UPDATED_DIFF_PRESSURE = 1 << 10
HIGHRES_IMU_UPDATED_PRESSURE_ALT = 1 << 11
HIGHRES_IMU_UPDATED_TEMPERATURE = 1 << 12

HIGHRES_IMU_FIELD_BITS: dict[str, int] = {
    "xacc": HIGHRES_IMU_UPDATED_XACC,
    "yacc": HIGHRES_IMU_UPDATED_YACC,
    "zacc": HIGHRES_IMU_UPDATED_ZACC,
    "xgyro": HIGHRES_IMU_UPDATED_XGYRO,
    "ygyro": HIGHRES_IMU_UPDATED_YGYRO,
    "zgyro": HIGHRES_IMU_UPDATED_ZGYRO,
    "xmag": HIGHRES_IMU_UPDATED_XMAG,
    "ymag": HIGHRES_IMU_UPDATED_YMAG,
    "zmag": HIGHRES_IMU_UPDATED_ZMAG,
    "abs_pressure": HIGHRES_IMU_UPDATED_ABS_PRESSURE,
    "diff_pressure": HIGHRES_IMU_UPDATED_DIFF_PRESSURE,
    "pressure_alt": HIGHRES_IMU_UPDATED_PRESSURE_ALT,
    "temperature": HIGHRES_IMU_UPDATED_TEMPERATURE,
}

HIGHRES_IMU_ALL_FIELDS_MASK = 0
for _bit in HIGHRES_IMU_FIELD_BITS.values():
    HIGHRES_IMU_ALL_FIELDS_MASK |= _bit


@dataclass(frozen=True, slots=True)
class HighresImuSample:
    time_usec: int
    xacc: float | None
    yacc: float | None
    zacc: float | None
    xgyro: float | None
    ygyro: float | None
    zgyro: float | None
    xmag: float | None
    ymag: float | None
    zmag: float | None
    abs_pressure: float | None
    diff_pressure: float | None
    pressure_alt: float | None
    temperature: float | None
    fields_updated: int
    sensor_id: int
    source_system: int | None
    source_component: int | None
    local_received_monotonic_ns: int
    transport: str

    def acceleration_norm(self) -> float | None:
        return _norm3(self.xacc, self.yacc, self.zacc)

    def angular_velocity_norm(self) -> float | None:
        return _norm3(self.xgyro, self.ygyro, self.zgyro)

    def magnetic_field_norm(self) -> float | None:
        return _norm3(self.xmag, self.ymag, self.zmag)

    def age_ms(self, now_monotonic_ns: int | None = None) -> float:
        now_ns = time_monotonic_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
        return max(0.0, (now_ns - self.local_received_monotonic_ns) / 1_000_000.0)

    def is_field_updated(self, field_name: str) -> bool:
        bit = HIGHRES_IMU_FIELD_BITS.get(field_name)
        if bit is None:
            raise KeyError(f"Unknown HIGHRES_IMU field: {field_name!r}")
        return self.fields_updated == 0 or bool(self.fields_updated & bit)


@dataclass(frozen=True, slots=True)
class HighresImuHealth:
    status: str
    reason: str
    enabled: bool
    sample_count: int
    stream_rate_hz: float | None
    update_age_ms: float | None
    max_staleness_ms: float
    sensor_count: int = 0
    active_sensor_ids: tuple[int, ...] = ()
    expected_rate_hz: float | None = None

    def is_stale(self) -> bool:
        return self.status == "stale"

    def is_healthy(self) -> bool:
        return self.status == "ok"


@dataclass(frozen=True, slots=True)
class SensorSnapshot:
    state: Any | None
    highres_imu: HighresImuSample | None
    highres_imu_health: HighresImuHealth | None
    captured_monotonic_ns: int
    transport: str

    def imu_age_ms(self) -> float | None:
        if self.highres_imu is None:
            return None
        return self.highres_imu.age_ms(self.captured_monotonic_ns)


def merge_highres_imu_sample(
    previous: HighresImuSample | None,
    *,
    time_usec: int,
    xacc: float | None,
    yacc: float | None,
    zacc: float | None,
    xgyro: float | None,
    ygyro: float | None,
    zgyro: float | None,
    xmag: float | None,
    ymag: float | None,
    zmag: float | None,
    abs_pressure: float | None,
    diff_pressure: float | None,
    pressure_alt: float | None,
    temperature: float | None,
    fields_updated: int,
    sensor_id: int,
    source_system: int | None,
    source_component: int | None,
    local_received_monotonic_ns: int,
    transport: str,
) -> HighresImuSample:
    sample = HighresImuSample(
        time_usec=int(time_usec),
        xacc=xacc,
        yacc=yacc,
        zacc=zacc,
        xgyro=xgyro,
        ygyro=ygyro,
        zgyro=zgyro,
        xmag=xmag,
        ymag=ymag,
        zmag=zmag,
        abs_pressure=abs_pressure,
        diff_pressure=diff_pressure,
        pressure_alt=pressure_alt,
        temperature=temperature,
        fields_updated=int(fields_updated),
        sensor_id=int(sensor_id),
        source_system=source_system,
        source_component=source_component,
        local_received_monotonic_ns=int(local_received_monotonic_ns),
        transport=transport,
    )
    if (
        previous is None
        or fields_updated == 0
        or (fields_updated & HIGHRES_IMU_ALL_FIELDS_MASK) == 0
    ):
        return sample

    for field_name, bit in HIGHRES_IMU_FIELD_BITS.items():
        if fields_updated & bit:
            continue
        sample = replace(sample, **{field_name: getattr(previous, field_name)})
    return sample


def format_highres_imu_health(health: HighresImuHealth | None) -> str:
    if health is None:
        return "unsupported"
    rate_text = "none" if health.stream_rate_hz is None else f"{health.stream_rate_hz:.2f}"
    age_text = "none" if health.update_age_ms is None else f"{health.update_age_ms:.1f}"
    expected_rate_text = "none"
    if health.expected_rate_hz is not None:
        expected_rate_text = f"{health.expected_rate_hz:.2f}"
    sensors_text = ",".join(str(sensor_id) for sensor_id in health.active_sensor_ids) or "none"
    return (
        f"status={health.status} reason={health.reason!r} enabled={health.enabled} "
        f"samples={health.sample_count} rate_hz={rate_text} age_ms={age_text} "
        f"expected_rate_hz={expected_rate_text} sensors={health.sensor_count}[{sensors_text}] "
        f"max_staleness_ms={health.max_staleness_ms:.1f}"
    )


def format_highres_imu_sample(sample: HighresImuSample | None) -> str:
    if sample is None:
        return "sample=none"
    accel_norm = sample.acceleration_norm()
    gyro_norm = sample.angular_velocity_norm()
    accel_text = "none" if accel_norm is None else f"{accel_norm:.3f}"
    gyro_text = "none" if gyro_norm is None else f"{gyro_norm:.3f}"
    return (
        "sample("
        f"id={sample.sensor_id},transport={sample.transport},time_usec={sample.time_usec},"
        f"age_ms={sample.age_ms():.1f},accel_norm={accel_text},gyro_norm={gyro_text},"
        f"zacc={sample.zacc},zgyro={sample.zgyro})"
    )


def _norm3(x: float | None, y: float | None, z: float | None) -> float | None:
    if x is None or y is None or z is None:
        return None
    return math.sqrt((x * x) + (y * y) + (z * z))


def time_monotonic_ns() -> int:
    import time

    return time.monotonic_ns()
