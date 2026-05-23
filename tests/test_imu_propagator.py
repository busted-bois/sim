import unittest

from src.tracking.imu_propagator import ImuPropagator


class ImuPropagatorTests(unittest.TestCase):
    def test_gyro_integration_advances_yaw(self) -> None:
        prop = ImuPropagator()
        prop.reset()
        t0 = 1_000_000
        ok = prop.step(t0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1)
        self.assertTrue(ok)
        ok2 = prop.step(t0 + 100_000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1)
        self.assertTrue(ok2)
        self.assertGreater(abs(prop.state.yaw), 0.005)

    def test_rejects_non_monotonic_time(self) -> None:
        prop = ImuPropagator()
        prop.reset()
        prop.step(2_000_000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertFalse(prop.step(1_000_000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
