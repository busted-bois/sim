import json
import unittest
from pathlib import Path

from src.simulator_specs import (
    PHYSICS_120_HZ_STEP_S,
    physics_snapshot_matches_120hz,
    specification_snapshot_validation,
)


class SimulatorSpecsPhysicsTests(unittest.TestCase):
    def test_repo_snapshot_documents_120hz_physics(self) -> None:
        root = Path(__file__).resolve().parents[1]
        path = root / "docs" / "simulator_specs.json"
        spec = json.loads(path.read_text(encoding="utf-8"))
        self.assertTrue(
            physics_snapshot_matches_120hz(spec.get("physics")),
            msg=f"Expected ~{PHYSICS_120_HZ_STEP_S}s async/substep in docs/simulator_specs.json",
        )

    def test_specification_snapshot_validation_physics_branch(self) -> None:
        snap = {
            "metadata": {"config_sha256": "", "runtime_camera_spec": None},
            "drone": {"dimensions_m": [1, 1, 1]},
            "gate_reference": {"dimensions_m": [1, 1, 1]},
            "physics": {
                "async_fixed_timestep_s": 0.01,
                "max_substep_delta_time_s": 0.01,
            },
        }
        errs, _, _ = specification_snapshot_validation(
            {"simulator": {"specification_path": "docs/simulator_specs.json"}},
            snap,
            spec_path="docs/simulator_specs.json",
            camera_resolution=[640, 360],
            camera_fov=100.0,
            normalized_pose_offset=[0.35, 0.0, -0.05],
        )
        self.assertTrue(any("120 Hz physics timing" in e for e in errs))


if __name__ == "__main__":
    unittest.main()
