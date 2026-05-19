from __future__ import annotations


def _horizontal_dims_m(gate: dict) -> tuple[float, float]:
    dims = gate["dimensions_m"]
    return float(dims[0]), float(dims[1])


def _opening_distance(gate: dict, opening: tuple[float, float]) -> float:
    width_m, height_m = _horizontal_dims_m(gate)
    return abs(width_m - opening[0]) + abs(height_m - opening[1])


def _within_opening_tolerance(
    gates: list[dict],
    opening: tuple[float, float],
    tolerance_m: float,
) -> list[dict]:
    matched: list[dict] = []
    for gate in gates:
        width_m, height_m = _horizontal_dims_m(gate)
        if (
            abs(width_m - opening[0]) <= tolerance_m
            and abs(height_m - opening[1]) <= tolerance_m
        ):
            matched.append(gate)
    return matched


def select_gates_for_reference(
    level_gates: list[dict],
    asset_gates: list[dict],
    opening: tuple[float, float],
    tolerance_m: float,
) -> list[dict]:
    for pool in (level_gates, asset_gates):
        matched = _within_opening_tolerance(pool, opening, tolerance_m)
        if matched:
            return matched

    all_gates = level_gates + asset_gates
    if not all_gates:
        return []

    scored = [(_opening_distance(gate, opening), gate) for gate in all_gates]
    best = min(score for score, _ in scored)
    return [gate for score, gate in scored if score == best]
