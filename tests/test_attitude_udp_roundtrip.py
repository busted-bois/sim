import socket
import struct
import unittest

from src.mavlink.attitude import decode_attitude_payload
from src.mavlink.frame import MAVLINK_V2_STX, frame_payload, parse_mavlink
from src.mavlink.messages import MAVLINK_MSG_ID_ATTITUDE


def _v2_attitude_frame(roll: float = 0.1, pitch: float = -0.2, yaw: float = 0.3) -> bytes:
    payload = struct.pack("<Iffffff", 1000, roll, pitch, yaw, 0.01, 0.02, 0.03)
    plen = len(payload)
    msgid_b = bytes([MAVLINK_MSG_ID_ATTITUDE & 0xFF, 0, 0])
    return bytes([MAVLINK_V2_STX, plen, 0, 0, 0, 1, 1, *msgid_b]) + payload + b"\x00\x00"


class AttitudeUdpRoundtripTests(unittest.TestCase):
    def test_decode_from_udp_datagram(self) -> None:
        port = 14599
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receiver.bind(("127.0.0.1", port))
        receiver.settimeout(2.0)
        try:
            sender.sendto(_v2_attitude_frame(), ("127.0.0.1", port))
            buf, _ = receiver.recvfrom(4096)
        finally:
            sender.close()
            receiver.close()

        frame = parse_mavlink(buf)
        assert frame is not None
        self.assertEqual(frame.msgid, MAVLINK_MSG_ID_ATTITUDE)
        payload = frame_payload(buf, frame)
        assert payload is not None
        sample = decode_attitude_payload(payload)
        assert sample is not None
        self.assertAlmostEqual(sample.roll, 0.1, places=5)
        self.assertAlmostEqual(sample.pitch, -0.2, places=5)


if __name__ == "__main__":
    unittest.main()
