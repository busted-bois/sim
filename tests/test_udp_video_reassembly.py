import unittest

from src.vision.udp_video import HEADER_STRUCT, UdpVideoReassembler


def _packet(
    frame_id: int, chunk_index: int, chunk_total: int, sim_time_ns: int, payload: bytes
) -> bytes:
    header = HEADER_STRUCT.pack(
        frame_id,
        chunk_index,
        chunk_total,
        sim_time_ns,
        len(payload),
        0,
    )
    return header + payload


class UdpVideoReassemblyTests(unittest.TestCase):
    def test_reassembles_two_chunk_frame(self) -> None:
        reassembler = UdpVideoReassembler(frame_timeout_s=1.0)
        jpeg = b"\xff\xd8\xff\xd9"
        p0 = _packet(1, 0, 2, 99_000_000, jpeg[:2])
        p1 = _packet(1, 1, 2, 99_000_000, jpeg[2:])
        self.assertIsNone(reassembler.ingest(p0))
        result = reassembler.ingest(p1)
        self.assertIsNotNone(result)
        assert result is not None
        self.assertEqual(result[0], 99_000_000)
        self.assertEqual(result[1], jpeg)


if __name__ == "__main__":
    unittest.main()
