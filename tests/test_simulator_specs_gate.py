import json
import unittest
from pathlib import Path

from src.competition_specs import competition_snapshot_validation, load_competition_specs


class SimulatorSpecsGateTests(unittest.TestCase):
    def test_repo_snapshot_gate_opening_matches_official(self) -> None:
        root = Path(__file__).resolve().parents[1]
        snap = json.loads((root / "docs" / "simulator_specs.json").read_text(encoding="utf-8"))
        specs = load_competition_specs(root / "docs" / "competition_specs.json")
        errs, _, _ = competition_snapshot_validation(snap, specs, tolerance_m=0.15)
        self.assertFalse(errs, msg="; ".join(errs))


if __name__ == "__main__":
    unittest.main()
