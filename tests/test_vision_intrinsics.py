import unittest

from src.vision.intrinsics import (
    camera_matrix_k,
    horizontal_fov_degrees,
    horizontal_fov_degrees_from_vision,
    official_resolution,
    official_resolution_list,
    yaw_mapping_half_fov_degrees,
)


class VisionIntrinsicsTests(unittest.TestCase):
    def test_official_resolution_helpers(self) -> None:
        self.assertEqual(official_resolution(), (640, 360))
        self.assertEqual(official_resolution_list(), [640, 360])

    def test_horizontal_fov_matches_intrinsics(self) -> None:
        self.assertAlmostEqual(horizontal_fov_degrees(), 90.0, places=6)

    def test_camera_matrix(self) -> None:
        k = camera_matrix_k()
        self.assertAlmostEqual(k[0][0], 320.0)
        self.assertAlmostEqual(k[1][1], 320.0)
        self.assertAlmostEqual(k[0][2], 320.0)
        self.assertAlmostEqual(k[1][2], 180.0)

    def test_horizontal_fov_from_vision_config(self) -> None:
        self.assertAlmostEqual(horizontal_fov_degrees_from_vision({"fov_degrees": 60.0}), 60.0)
        self.assertAlmostEqual(horizontal_fov_degrees_from_vision({}), 90.0)

    def test_yaw_mapping_half_fov(self) -> None:
        self.assertAlmostEqual(yaw_mapping_half_fov_degrees({"fov_degrees": 90.0}), 45.0)
        self.assertAlmostEqual(yaw_mapping_half_fov_degrees({"fov_degrees": 18.0}), 10.0)


if __name__ == "__main__":
    unittest.main()
