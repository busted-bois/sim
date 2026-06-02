import unittest
from pathlib import Path

from src.tracking.local_tracker import LocalTracker, TrackingConfig


class OriginOnArmTests(unittest.TestCase):
    def test_origin_resets_on_arm_rising_edge(self) -> None:
        tracker = LocalTracker(
            TrackingConfig(
                enabled=True,
                log_csv=False,
                csv_path=Path("logs/test_tracking.csv"),
            )
        )
        tracker.on_local_position(
            time_boot_ms=1000,
            x=10.0,
            y=20.0,
            z=-5.0,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            armed=False,
        )
        self.assertFalse(tracker.health().origin_set)
        tracker.on_local_position(
            time_boot_ms=1100,
            x=10.0,
            y=20.0,
            z=-5.0,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            armed=True,
        )
        self.assertTrue(tracker.health().origin_set)
        state = tracker.latest_state()
        self.assertIsNotNone(state)
        assert state is not None
        self.assertAlmostEqual(state.position_ned[0], 0.0, places=3)
        self.assertAlmostEqual(state.position_ned[1], 0.0, places=3)
        self.assertAlmostEqual(state.position_ned[2], 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
