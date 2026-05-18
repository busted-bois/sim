import struct
import unittest

from src.mavlink.frame import MAVLINK_V1_STX, MAVLINK_V2_STX, frame_payload, parse_mavlink
from src.mavlink.messages import MAVLINK_MSG_ID_ATTITUDE


def _v1_frame(msgid: int, payload: bytes, sysid: int = 1, compid: int = 1) -> bytes:
    plen = len(payload)
    header = bytes([MAVLINK_V1_STX, plen, 0, sysid, compid, msgid])
    return header + payload + b"\x00\x00"


def _v2_frame(msgid: int, payload: bytes, sysid: int = 1, compid: int = 1) -> bytes:
    plen = len(payload)
    msgid_b = bytes([msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    header = bytes([MAVLINK_V2_STX, plen, 0, 0, 0, sysid, compid, *msgid_b])
    return header + payload + b"\x00\x00"


class MavlinkFrameTests(unittest.TestCase):
    def test_parse_v1_attitude(self) -> None:
        payload = struct.pack("<Iffffff", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        buf = _v1_frame(MAVLINK_MSG_ID_ATTITUDE, payload)
        frame = parse_mavlink(buf)
        assert frame is not None
        self.assertEqual(frame.version, 1)
        self.assertEqual(frame.msgid, MAVLINK_MSG_ID_ATTITUDE)
        self.assertEqual(frame_payload(buf, frame), payload)

    def test_parse_v2_attitude(self) -> None:
        payload = struct.pack("<Iffffff", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        buf = _v2_frame(MAVLINK_MSG_ID_ATTITUDE, payload)
        frame = parse_mavlink(buf)
        assert frame is not None
        self.assertEqual(frame.version, 2)
        self.assertEqual(frame.msgid, MAVLINK_MSG_ID_ATTITUDE)
        self.assertEqual(frame_payload(buf, frame), payload)

    def test_invalid_buffer_returns_none(self) -> None:
        self.assertIsNone(parse_mavlink(b""))
        self.assertIsNone(parse_mavlink(b"\x00\x01"))


if __name__ == "__main__":
    unittest.main()
