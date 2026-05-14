import unittest

from src.preflight import official_conformant_vision_errors


class OfficialConformantVisionTests(unittest.TestCase):
    def test_skips_when_not_official_profile(self) -> None:
        sim = {"specification_profile": "low_end_nonconformant"}
        self.assertEqual(official_conformant_vision_errors(sim, [1280, 720], 100.0), [])

    def test_requires_resolution_and_fov(self) -> None:
        sim = {"specification_profile": "official_conformant"}
        self.assertTrue(official_conformant_vision_errors(sim, [1280, 720], 90.0))
        self.assertTrue(official_conformant_vision_errors(sim, [640, 360], 100.0))
        self.assertEqual(official_conformant_vision_errors(sim, [640, 360], 90.0), [])


if __name__ == "__main__":
    unittest.main()
