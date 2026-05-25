import unittest

from src.pathfinding import VoxelMap, VoxelObject, create_placeholder_voxel_map
from src.pathfinding.voxel_dijkstra import plan_route_through_gates


class VoxelDijkstraTests(unittest.TestCase):
    def test_placeholder_map_visits_all_gates(self) -> None:
        voxel_map = create_placeholder_voxel_map()
        path = plan_route_through_gates(voxel_map, (0, 0, -3), connectivity=26)

        self.assertEqual(set(path.gate_order), set(voxel_map.gates()))
        self.assertEqual(path.coords[0], (0, 0, -3))
        self.assertEqual(path.coords[-1], path.gate_order[-1])

    def test_hard_obstacles_are_never_traversed(self) -> None:
        voxel_map = VoxelMap(
            min_coord=(0, 0, 0),
            max_coord=(3, 2, 0),
            objects=(
                VoxelObject((1, 0, 0), "hard_obstacle"),
                VoxelObject((2, 0, 0), "gate"),
            ),
        )

        path = plan_route_through_gates(voxel_map, (0, 0, 0), connectivity=6)

        self.assertNotIn((1, 0, 0), path.coords)
        self.assertEqual(path.coords[-1], (2, 0, 0))

    def test_weighted_obstacles_are_avoided_when_cheaper(self) -> None:
        voxel_map = VoxelMap(
            min_coord=(0, 0, 0),
            max_coord=(2, 1, 0),
            objects=(
                VoxelObject((1, 0, 0), "weighted_obstacle", weight=10.0),
                VoxelObject((2, 0, 0), "gate"),
            ),
        )

        path = plan_route_through_gates(voxel_map, (0, 0, 0), connectivity=6)

        self.assertNotIn((1, 0, 0), path.coords)
        self.assertIn((1, 1, 0), path.coords)

    def test_optimized_route_can_choose_non_listed_gate_order(self) -> None:
        voxel_map = VoxelMap(
            min_coord=(0, -4, 0),
            max_coord=(10, 4, 0),
            objects=(
                VoxelObject((10, 0, 0), "gate", label="listed_first"),
                VoxelObject((1, 0, 0), "gate", label="listed_second"),
            ),
        )

        path = plan_route_through_gates(voxel_map, (0, 0, 0), connectivity=6)

        self.assertEqual(path.gate_order, ((1, 0, 0), (10, 0, 0)))


if __name__ == "__main__":
    unittest.main()
