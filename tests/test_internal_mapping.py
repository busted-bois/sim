import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from src.control.highres_imu import HighresImuSample
from src.internal_mapping import (
    CSV_HEADER,
    InternalMappingLogger,
    internal_mapping_config,
    resolve_internal_mapping_data_source,
)
from src.tracking.snapshot import TrackingSnapshot
from src.tracking.state import TrackingHealth, TrackingState


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

    def test_resolve_data_source_defaults_to_tracking_for_mavlink(self) -> None:
        cfg = {
            "control": {
                "transport": "mavlink",
                "mavlink": {"tracking": {"enabled": True}},
            }
        }
        self.assertEqual(resolve_internal_mapping_data_source(cfg), "tracking")

    def test_stop_writes_csv_rows_rpc(self) -> None:
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
            logger = InternalMappingLogger(
                client, out_path=path, sample_hz=50.0, data_source="rpc"
            )
            logger.start()
            logger.stop()
            text = path.read_text(encoding="utf-8")
            self.assertIn(",".join(CSV_HEADER), text)
            self.assertIn("1.000000", text)
            self.assertIn("rpc", text)

    def test_tracking_snapshot_writes_full_attitude(self) -> None:
        health = TrackingHealth(
            status="ok",
            reason="",
            imu_sample_count=10,
            imu_rate_hz=120.0,
            vision_correction_count=0,
            origin_set=True,
            armed=True,
        )
        state = TrackingState(
            sim_time_ns=1_000_000,
            t_s=0.5,
            position_ned=(1.0, 2.0, -3.0),
            velocity_ned=(0.1, 0.2, 0.3),
            attitude_rpy=(0.05, -0.02, 1.5),
            armed=True,
            origin_set=True,
        )
        snapshot = TrackingSnapshot(
            sim_time_ns=1_000_000,
            image_rgb=None,
            position_ned=state.position_ned,
            velocity_ned=state.velocity_ned,
            attitude_rpy=state.attitude_rpy,
            health=health,
            state=state,
        )
        imu = HighresImuSample(
            time_usec=42,
            xacc=0.1,
            yacc=0.2,
            zacc=9.8,
            xgyro=0.01,
            ygyro=0.02,
            zgyro=0.03,
            xmag=None,
            ymag=None,
            zmag=None,
            abs_pressure=None,
            diff_pressure=None,
            pressure_alt=None,
            temperature=None,
            fields_updated=0,
            sensor_id=0,
            source_system=None,
            source_component=None,
            local_received_monotonic_ns=0,
            transport="mavlink",
        )
        client = MagicMock()
        client.getTrackingSnapshot.return_value = snapshot
        client.getHighresImu.return_value = imu

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "internal_mapping.csv"
            logger = InternalMappingLogger(
                client,
                out_path=path,
                sample_hz=50.0,
                data_source="tracking",
                log_imu=True,
            )
            logger.start()
            logger.stop()
            text = path.read_text(encoding="utf-8")
            self.assertIn("0.050000", text)
            self.assertIn("-0.020000", text)
            self.assertIn("ok", text)
            self.assertIn("120.0", text)
            self.assertIn("9.800000", text)


if __name__ == "__main__":
    unittest.main()
