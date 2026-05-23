import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from src.internal_mapping import CSV_HEADER, InternalMappingLogger, internal_mapping_config


class InternalMappingTests(unittest.TestCase):
    def test_config_reads_exploration_section(self) -> None:
        cfg = {
            "autonomous_explore": {
                "exploration": {"internal_mapping": {"enabled": True, "sample_hz": 10.0}}
            }
        }
        im = internal_mapping_config(cfg)
        self.assertTrue(im["enabled"])
        self.assertEqual(im["sample_hz"], 10.0)

    def test_stop_writes_csv_rows(self) -> None:
        state = MagicMock()
        state.timestamp = 123
        state.kinematics_estimated.position.x_val = 1.0
        state.kinematics_estimated.position.y_val = 2.0
        state.kinematics_estimated.position.z_val = -3.0
        state.kinematics_estimated.linear_velocity.x_val = 0.1
        state.kinematics_estimated.linear_velocity.y_val = 0.2
        state.kinematics_estimated.linear_velocity.z_val = 0.3
        state.kinematics_estimated.orientation.w_val = 1.0
        state.kinematics_estimated.orientation.x_val = 0.0
        state.kinematics_estimated.orientation.y_val = 0.0
        state.kinematics_estimated.orientation.z_val = 0.0
        client = MagicMock()
        client.getMultirotorState.return_value = state

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "internal_mapping.csv"
            logger = InternalMappingLogger(client, out_path=path, sample_hz=50.0)
            logger.start()
            logger.stop()
            text = path.read_text(encoding="utf-8")
            self.assertIn(",".join(CSV_HEADER), text)
            self.assertIn("1.000000", text)


if __name__ == "__main__":
    unittest.main()
