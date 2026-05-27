import math
import unittest

from src.config import apply_low_end_overrides
from src.control.command_rate import CommandRateGate, normalize_command_rate_hz
from src.control.flight_client import AirSimAdapter
from src.control.primitives import _airsim_quaternion_from_euler, set_front_camera_pose
from src.preflight import is_official_conformant_profile, official_conformant_vision_errors
from src.simulator_specs import conformity_fingerprint, resolve_specification_path
from src.vision.feed import VisionFeed
from src.vision.intrinsics import horizontal_fov_degrees, official_resolution_list


class _FakeAsyncResult:
    def join(self, timeout=None) -> None:
        _ = timeout
        return


class _FakeAirSimClient:
    def __init__(self) -> None:
        self.velocity_calls = 0
        self.camera_pose = None

    def moveByVelocityAsync(self, *args, **kwargs):
        _ = args, kwargs
        self.velocity_calls += 1
        return _FakeAsyncResult()

    def simSetCameraPose(self, camera_name, pose) -> None:
        _ = camera_name
        self.camera_pose = pose


class SimulatorConformityTests(unittest.TestCase):
    def test_official_conformant_vision_errors_use_intrinsics_fov(self) -> None:
        sim = {"specification_profile": "official_conformant"}
        fov = horizontal_fov_degrees()
        res = official_resolution_list()
        self.assertEqual(official_conformant_vision_errors(sim, res, fov), [])
        self.assertGreater(len(official_conformant_vision_errors(sim, res, 60.0)), 0)

    def test_is_official_conformant_profile(self) -> None:
        self.assertTrue(
            is_official_conformant_profile({"specification_profile": "official_conformant"})
        )
        self.assertFalse(is_official_conformant_profile({"specification_profile": "other"}))

    def test_official_conformant_vision_errors_skip_other_profiles(self) -> None:
        sim = {"specification_profile": "low_end_nonconformant"}
        self.assertEqual(official_conformant_vision_errors(sim, [1280, 720], 100.0), [])

    def test_official_conformant_vision_errors_require_resolution(self) -> None:
        sim = {"specification_profile": "official_conformant"}
        fov = horizontal_fov_degrees()
        self.assertGreater(len(official_conformant_vision_errors(sim, [1280, 720], fov)), 0)

    def test_normalize_command_rate_clamps_under_100_hz(self) -> None:
        self.assertEqual(normalize_command_rate_hz(120.0), 99.0)
        self.assertEqual(normalize_command_rate_hz(50.0), 50.0)

    def test_airsim_adapter_drops_fast_motion_commands(self) -> None:
        client = _FakeAirSimClient()
        adapter = AirSimAdapter(client, command_rate_hz=50.0)

        first = adapter.moveByVelocityAsync(1.0, 0.0, 0.0, 0.1)
        second = adapter.moveByVelocityAsync(1.0, 0.0, 0.0, 0.1)

        first.join()
        second.join()
        self.assertEqual(client.velocity_calls, 1)
        stats = adapter.getCommandRateStats()
        self.assertIsNotNone(stats)
        assert stats is not None
        self.assertEqual(stats.allowed_count, 1)
        self.assertEqual(stats.dropped_count, 1)

    def test_command_rate_gate_reports_and_tracks_drops(self) -> None:
        messages: list[str] = []
        gate = CommandRateGate(
            50.0,
            label="test motion commands",
            reporter=messages.append,
            report_interval_s=0.0,
        )

        self.assertTrue(gate.allow(now_s=1.0))
        self.assertFalse(gate.allow(now_s=1.0))
        self.assertFalse(gate.allow(now_s=1.0))

        stats = gate.stats()
        self.assertEqual(stats.allowed_count, 1)
        self.assertEqual(stats.dropped_count, 2)
        self.assertEqual(stats.attempted_count, 3)
        self.assertTrue(messages)
        self.assertIn("dropped", messages[0])

    def test_set_front_camera_pose_applies_configured_pitch(self) -> None:
        client = _FakeAirSimClient()
        config = {
            "vision": {"camera_name": "0"},
            "camera": {
                "pose_offset": [0.35, 0.0, -0.05],
                "pitch_up_degrees": 20.0,
            },
        }

        set_front_camera_pose(client, config)

        self.assertIsNotNone(client.camera_pose)
        assert client.camera_pose is not None
        expected = _airsim_quaternion_from_euler(0.0, math.radians(-20.0), 0.0)
        self.assertAlmostEqual(client.camera_pose.orientation.x_val, expected.x_val, places=6)
        self.assertAlmostEqual(client.camera_pose.orientation.y_val, expected.y_val, places=6)
        self.assertAlmostEqual(client.camera_pose.orientation.z_val, expected.z_val, places=6)
        self.assertAlmostEqual(client.camera_pose.orientation.w_val, expected.w_val, places=6)

    def test_vision_feed_disables_autotune_when_strict_timing(self) -> None:
        feed = VisionFeed(
            client=None,  # type: ignore[arg-type]
            config={
                "enabled": True,
                "fps": 30.0,
                "strict_timing": True,
                "startup_autotune_enabled": True,
            },
        )
        self.assertFalse(feed._startup_autotune_enabled)  # type: ignore[attr-defined]
        self.assertEqual(feed._startup_min_fps, 30.0)  # type: ignore[attr-defined]

    def test_low_end_overrides_preserve_spec_values_when_required(self) -> None:
        config = {
            "simulator": {"specification_required": True},
            "vision": {"enabled": True, "fps": 30.0},
            "control": {"command_rate_hz": 50.0, "latency_tuning": {"enabled": True}},
            "low_end_profile": {"vision_enabled": False, "command_rate_hz": 25.0},
            "landing": {"telemetry_log": {"enabled": True}, "min_hover_seconds": 2.0},
            "logging": {},
        }
        try:
            import os

            old = os.environ.get("AIGP_LOW_END")
            os.environ["AIGP_LOW_END"] = "1"
            apply_low_end_overrides(config)
        finally:
            if old is None:
                os.environ.pop("AIGP_LOW_END", None)
            else:
                os.environ["AIGP_LOW_END"] = old

        self.assertTrue(config["vision"]["enabled"])
        self.assertEqual(config["vision"]["fps"], 30.0)
        self.assertEqual(config["control"]["command_rate_hz"], 50.0)
        self.assertFalse(config["control"]["latency_tuning"]["enabled"])

    def test_resolve_specification_path_is_repo_relative(self) -> None:
        path = resolve_specification_path(
            {"simulator": {"specification_path": "docs/simulator_specs.json"}}
        )
        self.assertIsNotNone(path)
        assert path is not None
        self.assertEqual(path.name, "simulator_specs.json")

    def test_conformity_fingerprint_changes_when_camera_spec_changes(self) -> None:
        config = {
            "simulator": {
                "map_asset": "/Game/FlyingCPP/Maps/FlyingExampleMapV2",
                "pawn_asset": "Class'/AirSim/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'",
                "gate_search_tokens": "gate,ring,torus",
                "physics_update_hz": 120.0,
                "specification_required": True,
            },
            "vision": {
                "camera_name": "0",
                "fps": 30.0,
                "resolution": [640, 360],
                "fov_degrees": 90.0,
                "strict_timing": True,
                "startup_autotune_enabled": False,
            },
            "control": {
                "command_rate_hz": 90.0,
                "latency_tuning": {"enabled": False},
            },
            "camera": {
                "pose_offset": [0.35, 0.0, -0.05],
                "pitch_up_degrees": 20.0,
                "roll_degrees": 0.0,
                "yaw_degrees": 0.0,
            },
        }
        baseline = conformity_fingerprint(config)
        config["vision"]["resolution"] = [1280, 720]
        changed = conformity_fingerprint(config)
        self.assertNotEqual(baseline, changed)


if __name__ == "__main__":
    unittest.main()
