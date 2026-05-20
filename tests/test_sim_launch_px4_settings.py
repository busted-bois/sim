import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from src import sim_launch


class SimLaunchPx4SettingsTests(unittest.TestCase):
    def test_px4_settings_use_tcp_hil_and_cameras(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            settings_path = Path(tmp) / "settings.json"
            config = {
                "simulator": {"settings_path": str(settings_path), "rpc_port": 41451},
                "vision": {
                    "camera_name": "0",
                    "fov_degrees": 90,
                    "resolution": [640, 360],
                },
                "camera": {
                    "pose_offset": [0.35, 0.0, -0.05],
                    "pitch_up_degrees": 20.0,
                },
                "control": {
                    "transport": "mavlink",
                    "mavlink": {
                        "auto_start_px4": True,
                        "bridge_profile": {"vehicle_type": "PX4Multirotor"},
                    },
                },
            }
            sim_launch._ensure_px4_settings_for_launch(
                41451,
                "Fpv",
                corner_chase_pip=False,
                enable_trace=True,
                config=config,
            )
            written = json.loads(settings_path.read_text(encoding="utf-8"))
            px4 = written["Vehicles"]["PX4"]
            self.assertEqual(px4["VehicleType"], "PX4Multirotor")
            self.assertTrue(px4["UseTcp"])
            self.assertEqual(px4["TcpPort"], 4560)
            self.assertTrue(px4["LockStep"])
            self.assertTrue(px4["EnableTrace"])
            self.assertIn("Cameras", px4)
            self.assertIn("0", px4["Cameras"])
            self.assertEqual(written["ViewMode"], "Fpv")
            self.assertEqual(written["ClockType"], "SteppableClock")

    def test_auto_start_px4_env_override(self) -> None:
        config = {"control": {"mavlink": {"auto_start_px4": True}}}
        old = os.environ.get("AIGP_AUTO_PX4")
        try:
            os.environ["AIGP_AUTO_PX4"] = "0"
            self.assertFalse(sim_launch._auto_start_px4_enabled(config))
            os.environ["AIGP_AUTO_PX4"] = "1"
            self.assertTrue(sim_launch._auto_start_px4_enabled(config))
        finally:
            if old is None:
                os.environ.pop("AIGP_AUTO_PX4", None)
            else:
                os.environ["AIGP_AUTO_PX4"] = old

    def test_restore_skipped_for_px4_vehicle(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            settings_path = Path(tmp) / "settings.json"
            backup_path = Path(tmp) / "settings.simpleflight.bak.json"
            px4_settings = {
                "Vehicles": {"PX4": {"VehicleType": "PX4Multirotor", "UseTcp": True}},
            }
            simple_settings = {
                "Vehicles": {"Drone1": {"VehicleType": "SimpleFlight"}},
            }
            settings_path.write_text(json.dumps(px4_settings), encoding="utf-8")
            backup_path.write_text(json.dumps(simple_settings), encoding="utf-8")
            config = {
                "simulator": {"settings_path": str(settings_path)},
                "control": {
                    "mavlink": {"bridge_profile": {"vehicle_type": "PX4Multirotor"}},
                },
            }
            self.assertFalse(sim_launch._maybe_restore_simpleflight_from_backup(config))
            self.assertEqual(
                json.loads(settings_path.read_text(encoding="utf-8"))["Vehicles"]["PX4"][
                    "VehicleType"
                ],
                "PX4Multirotor",
            )

    def test_uses_px4_multirotor_for_default_mavlink_config(self) -> None:
        config = {
            "control": {
                "mavlink": {"bridge_profile": {"vehicle_type": "PX4Multirotor"}},
            },
        }
        self.assertTrue(sim_launch._uses_px4_multirotor(config, transport="mavlink"))

    @mock.patch("src.config.load_config")
    @mock.patch("src.simulator_specs.assert_specification_snapshot_if_required")
    @mock.patch("src.sim_launch._resolve_project_path", return_value="/fake/project.uproject")
    @mock.patch("src.sim_launch.subprocess.Popen")
    @mock.patch(
        "src.sim_launch._wait_for_control_link",
        return_value=("mavlink", "udpin:0.0.0.0:14550"),
    )
    @mock.patch("src.mavlink_endpoints.first_mavlink_heartbeat_endpoint", return_value=None)
    @mock.patch("src.sim_launch._ensure_simulator_settings_for_launch")
    @mock.patch("src.sim_launch._bring_up_px4_hil_after_ue")
    def test_launch_calls_px4_bringup_on_windows(
        self,
        bringup_mock: mock.MagicMock,
        _settings_mock: mock.MagicMock,
        _heartbeat_mock: mock.MagicMock,
        _wait_link_mock: mock.MagicMock,
        popen_mock: mock.MagicMock,
        _project_mock: mock.MagicMock,
        _assert_spec_mock: mock.MagicMock,
        load_config_mock: mock.MagicMock,
    ) -> None:
        with tempfile.NamedTemporaryFile(suffix=".exe", delete=False) as tmp:
            colosseum = Path(tmp.name)
        try:
            ue_proc = mock.MagicMock()
            ue_proc.poll.return_value = None
            main_proc = mock.MagicMock()
            main_proc.wait.return_value = 0
            popen_mock.side_effect = [ue_proc, main_proc]
            load_config_mock.return_value = {
                "simulator": {
                    "colosseum_path": str(colosseum),
                    "rpc_port": 41451,
                    "rpc_ready_timeout_seconds": 30,
                    "specification_required": False,
                },
                "vision": {"resolution": [640, 360], "fov_degrees": 90},
                "camera": {"pose_offset": [0.35, 0.0, -0.05], "pitch_up_degrees": 20.0},
                "control": {
                    "transport": "mavlink",
                    "mavlink": {
                        "auto_start_px4": True,
                        "bridge_profile": {"vehicle_type": "PX4Multirotor"},
                    },
                },
            }
            with mock.patch.object(sim_launch.sys, "platform", "win32"):
                with self.assertRaises(SystemExit) as ctx:
                    sim_launch.launch()
                self.assertEqual(ctx.exception.code, 0)
            bringup_mock.assert_called_once()
        finally:
            colosseum.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
