import unittest

from src.control.algorithms.yolo_explore import _pick_target
from src.vision.yolo_detector import Detection


class YoloExploreTests(unittest.TestCase):
    def test_pick_target_prefers_blue_over_red(self) -> None:
        detections = [
            Detection("red_target", 0.9, 0, 0, 10, 10, 0.0, 0.0, 0.05),
            Detection("blue_ring", 0.8, 20, 20, 40, 40, 0.1, 0.0, 0.04),
        ]
        picked = _pick_target(
            detections,
            blue_classes={"blue_ring"},
            red_classes={"red_target"},
            blue_active=True,
            red_active=True,
            min_size_frac=0.001,
        )
        self.assertEqual(picked, ("blue_ring", detections[1]))

    def test_pick_target_skips_small_boxes(self) -> None:
        detections = [
            Detection("blue_ring", 0.9, 0, 0, 2, 2, 0.0, 0.0, 0.0001),
        ]
        self.assertIsNone(
            _pick_target(
                detections,
                blue_classes={"blue_ring"},
                red_classes=set(),
                blue_active=True,
                red_active=False,
                min_size_frac=0.01,
            )
        )


if __name__ == "__main__":
    unittest.main()
