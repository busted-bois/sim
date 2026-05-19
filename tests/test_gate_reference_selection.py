import unittest

from src.simulator_gate_selection import select_gates_for_reference


def _gate(width_m: float, height_m: float, label: str = "") -> dict:
    return {"label": label, "dimensions_m": [width_m, height_m, 0.5]}


class GateReferenceSelectionTests(unittest.TestCase):
    def test_prefers_opening_sized_asset_over_oversized_level(self) -> None:
        level = [_gate(5.5, 5.5, label="scaled_actor")]
        asset = [_gate(1.5, 1.5, label="torus_mesh")]
        chosen = select_gates_for_reference(level, asset, (1.5, 1.5), 0.15)
        self.assertEqual([gate["label"] for gate in chosen], ["torus_mesh"])

    def test_prefers_matching_level_gate_when_present(self) -> None:
        level = [_gate(1.5, 1.5, label="good_actor"), _gate(5.5, 5.5, label="bad_actor")]
        asset = [_gate(1.5, 1.5, label="torus_mesh")]
        chosen = select_gates_for_reference(level, asset, (1.5, 1.5), 0.15)
        labels = {gate["label"] for gate in chosen}
        self.assertIn("good_actor", labels)
        self.assertNotIn("bad_actor", labels)

    def test_closest_fallback_when_nothing_within_tolerance(self) -> None:
        only_oversized = [_gate(5.5, 5.5, label="only")]
        chosen = select_gates_for_reference(only_oversized, [], (1.5, 1.5), 0.15)
        self.assertEqual(chosen[0]["label"], "only")


if __name__ == "__main__":
    unittest.main()
