import math
import struct
import unittest

from src.mavlink.attitude import (
    decode_attitude_payload,
    roll_pitch_yaw_deg,
)
from src.mavlink.messages import MAVLINK_MSG_ID_ATTITUDE


class AttitudeDecodeTests(unittest.TestCase):
    def test_decode_known_payload(self) -> None:
        payload = struct.pack(
            "<Iffffff",
            1_000,
            0.1,
            -0.2,
            1.5,
            0.01,
            -0.02,
            0.03,
        )
        sample = decode_attitude_payload(payload)
        assert sample is not None
        self.assertEqual(sample.time_boot_ms, 1_000)
        self.assertAlmostEqual(sample.roll, 0.1)
        self.assertAlmostEqual(sample.pitch, -0.2)
        self.assertAlmostEqual(sample.yaw, 1.5)
        roll_d, pitch_d, yaw_d = roll_pitch_yaw_deg(sample)
        self.assertAlmostEqual(roll_d, math.degrees(sample.roll), places=5)
        self.assertAlmostEqual(pitch_d, math.degrees(sample.pitch), places=5)
        self.assertAlmostEqual(yaw_d, math.degrees(sample.yaw), places=5)

    def test_short_payload_returns_none(self) -> None:
        self.assertIsNone(decode_attitude_payload(b"\x00" * 10))

    def test_msg_id_constant(self) -> None:
        self.assertEqual(MAVLINK_MSG_ID_ATTITUDE, 30)


if __name__ == "__main__":
    unittest.main()
