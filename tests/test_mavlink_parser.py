"""Unit tests for the MAVLink message parser — no simulator needed."""

import math
import struct

import pytest

from comms.mavlink_parser import (
    Attitude,
    Heartbeat,
    HighresImu,
    MsgId,
    Odometry,
    Timesync,
    decode,
    encode,
    mavlink_crc,
    MAVLINK_STX_V2,
)


# ── Heartbeat ────────────────────────────────────────────────

class TestHeartbeat:
    def test_roundtrip(self):
        original = Heartbeat(
            custom_mode=65536,
            type=2,
            autopilot=12,
            base_mode=209,
            system_status=4,
            mavlink_version=3,
        )
        data = encode(original)
        decoded = decode(data)

        assert isinstance(decoded, Heartbeat)
        assert decoded.custom_mode == 65536
        assert decoded.type == 2
        assert decoded.autopilot == 12
        assert decoded.base_mode == 209
        assert decoded.system_status == 4
        assert decoded.mavlink_version == 3

    def test_default_values(self):
        hb = Heartbeat()
        data = encode(hb)
        decoded = decode(data)
        assert decoded.custom_mode == 0
        assert decoded.mavlink_version == 3

    def test_frame_starts_with_stx(self):
        data = encode(Heartbeat())
        assert data[0] == MAVLINK_STX_V2


# ── Attitude ─────────────────────────────────────────────────

class TestAttitude:
    def test_roundtrip(self):
        original = Attitude(
            time_boot_ms=123456,
            roll=0.1,
            pitch=-0.2,
            yaw=3.14,
            rollspeed=0.01,
            pitchspeed=-0.02,
            yawspeed=0.05,
        )
        decoded = decode(encode(original))

        assert isinstance(decoded, Attitude)
        assert decoded.time_boot_ms == 123456
        assert decoded.roll == pytest.approx(0.1, abs=1e-6)
        assert decoded.pitch == pytest.approx(-0.2, abs=1e-6)
        assert decoded.yaw == pytest.approx(3.14, abs=1e-5)
        assert decoded.rollspeed == pytest.approx(0.01, abs=1e-6)
        assert decoded.pitchspeed == pytest.approx(-0.02, abs=1e-6)
        assert decoded.yawspeed == pytest.approx(0.05, abs=1e-6)

    def test_extreme_values(self):
        att = Attitude(roll=math.pi, pitch=-math.pi, yaw=2 * math.pi)
        decoded = decode(encode(att))
        assert decoded.roll == pytest.approx(math.pi, abs=1e-5)
        assert decoded.pitch == pytest.approx(-math.pi, abs=1e-5)


# ── HighresImu ───────────────────────────────────────────────

class TestHighresImu:
    def test_roundtrip(self):
        original = HighresImu(
            time_usec=9876543210,
            xacc=0.5, yacc=-1.0, zacc=-9.81,
            xgyro=0.01, ygyro=-0.02, zgyro=0.03,
            xmag=0.2, ymag=0.0, zmag=0.4,
            abs_pressure=1013.25,
            diff_pressure=0.5,
            pressure_alt=100.0,
            temperature=25.0,
            fields_updated=0xFFFF,
        )
        decoded = decode(encode(original))

        assert isinstance(decoded, HighresImu)
        assert decoded.time_usec == 9876543210
        assert decoded.xacc == pytest.approx(0.5, abs=1e-6)
        assert decoded.zacc == pytest.approx(-9.81, abs=1e-4)
        assert decoded.abs_pressure == pytest.approx(1013.25, abs=1e-2)
        assert decoded.temperature == pytest.approx(25.0, abs=1e-4)
        assert decoded.fields_updated == 0xFFFF

    def test_zero_fields(self):
        imu = HighresImu()  # all defaults (zeros)
        decoded = decode(encode(imu))
        assert decoded.time_usec == 0
        assert decoded.xacc == 0.0


# ── Timesync ─────────────────────────────────────────────────

class TestTimesync:
    def test_roundtrip(self):
        original = Timesync(tc1=0, ts1=1711234567890123456)
        decoded = decode(encode(original))

        assert isinstance(decoded, Timesync)
        assert decoded.tc1 == 0
        assert decoded.ts1 == 1711234567890123456

    def test_negative_values(self):
        ts = Timesync(tc1=-100, ts1=-200)
        decoded = decode(encode(ts))
        assert decoded.tc1 == -100
        assert decoded.ts1 == -200


# ── Odometry ─────────────────────────────────────────────────

class TestOdometry:
    def test_roundtrip(self):
        original = Odometry(
            time_usec=5000000,
            frame_id=1,
            child_frame_id=1,
            x=10.0, y=20.0, z=-5.0,
            q=(0.707, 0.0, 0.707, 0.0),
            vx=1.0, vy=2.0, vz=-0.5,
            rollspeed=0.01, pitchspeed=0.02, yawspeed=0.1,
        )
        decoded = decode(encode(original))

        assert isinstance(decoded, Odometry)
        assert decoded.time_usec == 5000000
        assert decoded.frame_id == 1
        assert decoded.x == pytest.approx(10.0, abs=1e-5)
        assert decoded.y == pytest.approx(20.0, abs=1e-5)
        assert decoded.z == pytest.approx(-5.0, abs=1e-5)
        assert decoded.q[0] == pytest.approx(0.707, abs=1e-3)
        assert decoded.q[2] == pytest.approx(0.707, abs=1e-3)
        assert decoded.vx == pytest.approx(1.0, abs=1e-5)

    def test_identity_quaternion(self):
        odom = Odometry(q=(1.0, 0.0, 0.0, 0.0))
        decoded = decode(encode(odom))
        assert decoded.q == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-6)


# ── CRC and frame integrity ─────────────────────────────────

class TestFrameIntegrity:
    def test_corrupted_payload_rejected(self):
        data = bytearray(encode(Heartbeat()))
        # Flip a byte in the payload area
        data[12] ^= 0xFF
        assert decode(bytes(data)) is None

    def test_truncated_frame_rejected(self):
        data = encode(Heartbeat())
        assert decode(data[:5]) is None

    def test_wrong_stx_rejected(self):
        data = bytearray(encode(Heartbeat()))
        data[0] = 0xAA
        assert decode(bytes(data)) is None

    def test_empty_input(self):
        assert decode(b"") is None

    def test_unknown_msgid_returns_none(self):
        data = bytearray(encode(Heartbeat()))
        # Overwrite msg_id bytes to an unknown ID (won't pass CRC, but tests early rejection)
        data[7] = 0xFF
        data[8] = 0xFF
        data[9] = 0x0F
        assert decode(bytes(data)) is None

    def test_crc_known_value(self):
        """Verify CRC function produces consistent results."""
        crc1 = mavlink_crc(b"\x00\x01\x02\x03", extra=50)
        crc2 = mavlink_crc(b"\x00\x01\x02\x03", extra=50)
        assert crc1 == crc2
        # Different data should produce different CRC
        crc3 = mavlink_crc(b"\x00\x01\x02\x04", extra=50)
        assert crc1 != crc3


# ── Sequence numbering ───────────────────────────────────────

class TestSequencing:
    def test_sequence_increments(self):
        data1 = encode(Heartbeat())
        data2 = encode(Heartbeat())
        seq1 = data1[4]
        seq2 = data2[4]
        assert (seq2 - seq1) % 256 == 1
