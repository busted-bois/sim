from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import permutations, product
from math import inf, sqrt
from typing import Literal

VoxelCoord = tuple[int, int, int]
VoxelObjectKind = Literal["hard_obstacle", "weighted_obstacle", "gate"]
Connectivity = Literal[6, 26]


@dataclass(frozen=True, slots=True)
class VoxelObject:
    coord: VoxelCoord
    kind: VoxelObjectKind
    weight: float = 0.0
    label: str = ""


@dataclass(frozen=True, slots=True)
class VoxelPath:
    coords: tuple[VoxelCoord, ...]
    cost: float
    gate_order: tuple[VoxelCoord, ...]


@dataclass(frozen=True, slots=True)
class VoxelMap:
    min_coord: VoxelCoord
    max_coord: VoxelCoord
    objects: tuple[VoxelObject, ...]
    default_weight: float = 0.0

    def gates(self) -> tuple[VoxelCoord, ...]:
        return tuple(obj.coord for obj in self.objects if obj.kind == "gate")

    def contains(self, coord: VoxelCoord) -> bool:
        return all(
            lo <= value <= hi
            for value, lo, hi in zip(coord, self.min_coord, self.max_coord)
        )

    def traversal_weight(self, coord: VoxelCoord) -> float:
        weight = self.default_weight
        for obj in self.objects:
            if obj.coord != coord:
                continue
            if obj.kind == "hard_obstacle":
                return inf
            if obj.kind == "weighted_obstacle":
                weight += max(0.0, obj.weight)
        return weight

    def traversable(self, coord: VoxelCoord) -> bool:
        return self.contains(coord) and self.traversal_weight(coord) < inf


def create_placeholder_voxel_map() -> VoxelMap:
    objects: list[VoxelObject] = [
        VoxelObject((5, -2, -3), "gate", label="gate_alpha"),
        VoxelObject((10, 2, -4), "gate", label="gate_beta"),
        VoxelObject((15, 0, -3), "gate", label="gate_gamma"),
    ]
    objects.extend(
        VoxelObject((x, 0, z), "hard_obstacle", label="wall")
        for x in range(2, 13)
        for z in range(-5, -1)
        if x not in {6, 7}
    )
    objects.extend(
        [
            VoxelObject((3, -1, -3), "weighted_obstacle", weight=4.0, label="near_crate"),
            VoxelObject((8, 1, -4), "weighted_obstacle", weight=2.5, label="hanging_cable"),
            VoxelObject((12, 1, -3), "weighted_obstacle", weight=5.0, label="narrow_gap"),
        ]
    )
    return VoxelMap(min_coord=(0, -5, -6), max_coord=(18, 5, -1), objects=tuple(objects))


def plan_route_through_gates(
    voxel_map: VoxelMap,
    start: VoxelCoord,
    *,
    connectivity: Connectivity = 26,
) -> VoxelPath:
    if not voxel_map.traversable(start):
        raise ValueError(f"start voxel is not traversable: {start}")

    gates = voxel_map.gates()
    if not gates:
        return VoxelPath(coords=(start,), cost=0.0, gate_order=())

    pair_cache: dict[tuple[VoxelCoord, VoxelCoord], VoxelPath] = {}
    route_points = (start, *gates)
    for source in route_points:
        for goal in gates:
            if source == goal:
                continue
            pair_cache[(source, goal)] = _dijkstra(voxel_map, source, goal, connectivity)

    best_path: VoxelPath | None = None
    for gate_order in permutations(gates):
        candidate_coords = [start]
        candidate_cost = 0.0
        current = start
        reachable = True
        for gate in gate_order:
            segment = pair_cache.get((current, gate))
            if segment is None or not segment.coords:
                reachable = False
                break
            candidate_cost += segment.cost
            candidate_coords.extend(segment.coords[1:])
            current = gate
        if not reachable:
            continue
        candidate = VoxelPath(tuple(candidate_coords), candidate_cost, gate_order)
        if best_path is None or candidate.cost < best_path.cost:
            best_path = candidate

    if best_path is None:
        raise ValueError("no route reaches all gates")
    return best_path


def _dijkstra(
    voxel_map: VoxelMap,
    start: VoxelCoord,
    goal: VoxelCoord,
    connectivity: Connectivity,
) -> VoxelPath:
    frontier: list[tuple[float, VoxelCoord]] = [(0.0, start)]
    came_from: dict[VoxelCoord, VoxelCoord | None] = {start: None}
    cost_so_far: dict[VoxelCoord, float] = {start: 0.0}

    while frontier:
        current_cost, current = heappop(frontier)
        if current == goal:
            return VoxelPath(_reconstruct_path(came_from, goal), current_cost, (goal,))
        if current_cost > cost_so_far[current]:
            continue

        for neighbor, step_cost in _neighbors(current, connectivity):
            if not voxel_map.traversable(neighbor):
                continue
            next_cost = current_cost + step_cost * (1.0 + voxel_map.traversal_weight(neighbor))
            if next_cost < cost_so_far.get(neighbor, inf):
                cost_so_far[neighbor] = next_cost
                came_from[neighbor] = current
                heappush(frontier, (next_cost, neighbor))

    return VoxelPath(coords=(), cost=inf, gate_order=())


def _neighbors(
    coord: VoxelCoord,
    connectivity: Connectivity,
) -> tuple[tuple[VoxelCoord, float], ...]:
    offsets = []
    for dx, dy, dz in product((-1, 0, 1), repeat=3):
        if dx == dy == dz == 0:
            continue
        changed_axes = abs(dx) + abs(dy) + abs(dz)
        if connectivity == 6 and changed_axes != 1:
            continue
        offsets.append(((coord[0] + dx, coord[1] + dy, coord[2] + dz), sqrt(changed_axes)))
    return tuple(offsets)


def _reconstruct_path(
    came_from: dict[VoxelCoord, VoxelCoord | None],
    goal: VoxelCoord,
) -> tuple[VoxelCoord, ...]:
    path = [goal]
    current = goal
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return tuple(path)
