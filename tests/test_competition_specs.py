import json
import unittest
from pathlib import Path

from src.competition_specs import (
    competition_snapshot_validation,
    dimensions_close,
    load_competition_specs,
)


class CompetitionSpecsTests(unittest.TestCase):
    def test_official_gate_clearance(self) -> None:
        specs = load_competition_specs()
        cx, cy = specs.gate_opening_clearance_m()
        self.assertAlmostEqual(cx, 0.61, places=2)
        self.assertAlmostEqual(cy, 0.61, places=2)

    def test_repo_json_loads(self) -> None:
        path = Path(__file__).resolve().parents[1] / "docs" / "competition_specs.json"
        raw = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(raw["metadata"]["document_id"], "VADR-TS-002")
        specs = load_competition_specs(path)
        self.assertAlmostEqual(specs.gate_opening.width_m, 1.5)

    def test_snapshot_gate_opening_matches_official(self) -> None:
        specs = load_competition_specs()
        snap = {"gate_reference": {"dimensions_m": [1.5, 1.5, 0.5]}}
        errs, passes, warns = competition_snapshot_validation(snap, specs, tolerance_m=0.15)
        self.assertFalse(errs)
        self.assertTrue(any("width x height matches" in p for p in passes))
        self.assertTrue(any("depth" in w and "differs" in w for w in warns))

    def test_snapshot_gate_opening_mismatch_errors(self) -> None:
        specs = load_competition_specs()
        snap = {"gate_reference": {"dimensions_m": [2.0, 2.0, 0.26]}}
        errs, _, _ = competition_snapshot_validation(snap, specs, tolerance_m=0.1)
        self.assertTrue(errs)

    def test_dimensions_close(self) -> None:
        specs = load_competition_specs()
        self.assertTrue(
            dimensions_close([1.5, 1.5, 0.26], specs.gate_opening, tolerance=0.15)
        )
        self.assertFalse(
            dimensions_close([2.0, 1.5, 0.26], specs.gate_opening, tolerance=0.1)
        )


if __name__ == "__main__":
    unittest.main()
