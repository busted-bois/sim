import unittest

from src.config import load_config
from src.mavlink.config import load_attitude_mavlink_config


class AttitudeConfigTests(unittest.TestCase):
    def test_load_defaults_when_missing(self) -> None:
        cfg = load_attitude_mavlink_config({})
        self.assertTrue(cfg["enabled"])
        self.assertEqual(cfg["request_hz"], 50.0)

    def test_load_from_sim_config(self) -> None:
        cfg = load_attitude_mavlink_config(load_config())
        self.assertTrue(cfg["enabled"])
        self.assertEqual(cfg["request_hz"], 50.0)


if __name__ == "__main__":
    unittest.main()
