"""Official AI Grand Prix physical dimensions (VADR-TS-002).

Used for gate clearance planning, preflight checks against simulator snapshots,
and future navigation / control tuning. Values are stored in meters.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SPEC_PATH = ROOT / "docs" / "competition_specs.json"


@dataclass(frozen=True, slots=True)
class BoxDimensionsM:
    width_m: float
    length_m: float
    height_m: float

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.width_m, self.length_m, self.height_m)

    @property
    def horizontal_footprint_m(self) -> float:
        return max(self.width_m, self.length_m)


@dataclass(frozen=True, slots=True)
class CompetitionSpecs:
    document_id: str
    issue: str
    drone_chassis: BoxDimensionsM
    gate_outer: BoxDimensionsM
    gate_opening: BoxDimensionsM
    gate_frame_margin_m: float

    def gate_opening_clearance_m(self, *, centered: bool = True) -> tuple[float, float]:
        """Side clearance (m) for a centered drone through the gate opening (horizontal axes)."""
        if not centered:
            raise ValueError("only centered clearance is defined for official specs")
        footprint = self.drone_chassis.horizontal_footprint_m
        clearance_x = (self.gate_opening.width_m - footprint) / 2.0
        clearance_y = (self.gate_opening.length_m - footprint) / 2.0
        return clearance_x, clearance_y


def resolve_competition_spec_path(config: dict[str, Any]) -> Path | None:
    comp_cfg = config.get("competition", {})
    raw_path = str(comp_cfg.get("spec_path", "")).strip()
    if not raw_path:
        return None
    path = Path(raw_path)
    if not path.is_absolute():
        path = ROOT / path
    return path


def load_competition_specs(path: Path | str | None = None) -> CompetitionSpecs:
    spec_path = Path(path) if path is not None else DEFAULT_SPEC_PATH
    raw = json.loads(spec_path.read_text(encoding="utf-8"))
    meta = raw.get("metadata", {})
    drone_raw = raw["drone_chassis"]
    gate_raw = raw["gate"]
    return CompetitionSpecs(
        document_id=str(meta.get("document_id", "")),
        issue=str(meta.get("issue", "")),
        drone_chassis=BoxDimensionsM(
            width_m=float(drone_raw["width_m"]),
            length_m=float(drone_raw["length_m"]),
            height_m=float(drone_raw["height_m"]),
        ),
        gate_outer=BoxDimensionsM(
            width_m=float(gate_raw["outer"]["width_m"]),
            length_m=float(gate_raw["outer"]["height_m"]),
            height_m=float(gate_raw["outer"]["depth_m"]),
        ),
        gate_opening=BoxDimensionsM(
            width_m=float(gate_raw["opening"]["width_m"]),
            length_m=float(gate_raw["opening"]["height_m"]),
            height_m=float(gate_raw["opening"]["depth_m"]),
        ),
        gate_frame_margin_m=float(gate_raw.get("frame_margin_m", 0.0)),
    )


def dimensions_close(
    actual: list[float] | tuple[float, ...],
    expected: BoxDimensionsM,
    *,
    tolerance: float,
) -> bool:
    if len(actual) < 3:
        return False
    expected_tuple = expected.as_tuple()
    pairs = zip(actual, expected_tuple, strict=True)
    return all(abs(float(a) - float(e)) <= tolerance for a, e in pairs)


def competition_snapshot_validation(
    spec_snapshot: dict[str, Any],
    competition: CompetitionSpecs,
    *,
    tolerance_m: float = 0.15,
    validate_drone_chassis: bool = False,
) -> tuple[list[str], list[str], list[str]]:
    """Compare extracted simulator dimensions to official competition references."""
    errors: list[str] = []
    passes: list[str] = []
    warnings: list[str] = []

    gate_dims = spec_snapshot.get("gate_reference", {}).get("dimensions_m")
    if isinstance(gate_dims, list) and len(gate_dims) >= 2:
        opening = competition.gate_opening
        horizontal_ok = all(
            abs(float(gate_dims[i]) - expected) <= tolerance_m
            for i, expected in enumerate((opening.width_m, opening.length_m))
        )
        if horizontal_ok:
            passes.append(
                "Simulator gate_reference opening width x height matches official "
                f"({opening.width_m:.2f}x{opening.length_m:.2f} m) within ±{tolerance_m:.2f} m"
            )
        else:
            errors.append(
                "Simulator gate_reference opening "
                f"{gate_dims[0]:.3f}x{gate_dims[1]:.3f} m does not match official "
                f"{opening.width_m:.2f}x{opening.length_m:.2f} m within ±{tolerance_m:.2f} m"
            )
        if len(gate_dims) >= 3:
            depth_delta = abs(float(gate_dims[2]) - opening.height_m)
            if depth_delta <= tolerance_m:
                passes.append("Simulator gate_reference depth matches official opening depth")
            else:
                warnings.append(
                    "Simulator gate_reference depth "
                    f"{float(gate_dims[2]):.3f} m differs from official "
                    f"{opening.height_m:.3f} m (mesh thickness may differ in UE)"
                )
    else:
        warnings.append(
            "Simulator snapshot has no gate_reference.dimensions_m; "
            "skipped official gate opening check"
        )

    if validate_drone_chassis:
        drone_dims = spec_snapshot.get("drone", {}).get("dimensions_m")
        if isinstance(drone_dims, list) and len(drone_dims) >= 3:
            if dimensions_close(drone_dims, competition.drone_chassis, tolerance=tolerance_m):
                passes.append("Simulator drone mesh matches official chassis dimensions")
            else:
                warnings.append(
                    "Simulator drone mesh dimensions "
                    f"{drone_dims[:3]} differ from official chassis "
                    f"{list(competition.drone_chassis.as_tuple())} "
                    "(expected when the UE pawn is not 1:1 with competition drawings)"
                )

    cx, cy = competition.gate_opening_clearance_m()
    passes.append(
        f"Official centered gate clearance (chassis footprint): "
        f"{cx:.2f} m x {cy:.2f} m per horizontal axis"
    )
    return errors, passes, warnings


def competition_validation_from_config(
    config: dict[str, Any],
    spec_snapshot: dict[str, Any],
) -> tuple[list[str], list[str], list[str]]:
    comp_cfg = config.get("competition", {})
    if not bool(comp_cfg.get("validate_against_snapshot", True)):
        return [], [], []
    spec_path = resolve_competition_spec_path(config)
    if spec_path is None or not spec_path.is_file():
        return (
            [],
            [],
            [
                "competition.spec_path is not configured or missing; "
                "official dimension checks were skipped"
            ],
        )
    competition = load_competition_specs(spec_path)
    tolerance_m = float(comp_cfg.get("dimension_tolerance_m", 0.15))
    validate_drone = bool(comp_cfg.get("validate_drone_chassis_in_snapshot", False))
    return competition_snapshot_validation(
        spec_snapshot,
        competition,
        tolerance_m=tolerance_m,
        validate_drone_chassis=validate_drone,
    )
