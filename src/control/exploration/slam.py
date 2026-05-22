"""SLAM-inspired pose tracking, log-odds grid, landmarks, loop closure, export."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from src.control.utils import _clamp

_BEARING_BINS = 36
_LOG_ODDS_FREE = -0.45
_LOG_ODDS_OCC = 0.65
_LOG_ODDS_CLAMP = 4.0
_FREE_THRESH = -0.35
_OCC_THRESH = 0.45

@dataclass(frozen=True)
class SlamExplorationSettings:
    enabled: bool = True
    grid_cell_m: float = 1.0
    log_odds_enabled: bool = True
    frontier_enabled: bool = True
    loop_closure_enabled: bool = True
    loop_closure_radius_m: float = 2.5
    loop_closure_min_path_m: float = 8.0
    loop_closure_cooldown_s: float = 25.0
    active_exploration_enabled: bool = True
    active_exploration_gain_deg_s: float = 12.0
    landmark_enabled: bool = True
    landmark_default_range_m: float = 6.0
    landmark_max_range_m: float = 12.0
    imu_yaw_assist_gain: float = 0.85
    export_enabled: bool = True
    export_path: str = "logs/exploration_map.json"


@dataclass(frozen=True)
class SlamStatus:
    x_m: float
    y_m: float
    path_m: float
    known_cells: int
    free_cells: int
    occupied_cells: int
    frontier_cells: int
    landmark_count: int
    coverage_ratio: float
    loop_closures: int


def parse_slam_settings(raw: dict[str, Any] | None) -> SlamExplorationSettings:
    slam = raw or {}
    return SlamExplorationSettings(
        enabled=bool(slam.get("enabled", True)),
        grid_cell_m=_clamp(float(slam.get("grid_cell_m", 1.0)), 0.5, 5.0),
        log_odds_enabled=bool(slam.get("log_odds_enabled", True)),
        frontier_enabled=bool(slam.get("frontier_enabled", True)),
        loop_closure_enabled=bool(slam.get("loop_closure_enabled", True)),
        loop_closure_radius_m=_clamp(float(slam.get("loop_closure_radius_m", 2.5)), 0.5, 15.0),
        loop_closure_min_path_m=_clamp(float(slam.get("loop_closure_min_path_m", 8.0)), 2.0, 100.0),
        loop_closure_cooldown_s=_clamp(
            float(slam.get("loop_closure_cooldown_s", 25.0)), 5.0, 120.0
        ),
        active_exploration_enabled=bool(slam.get("active_exploration_enabled", True)),
        active_exploration_gain_deg_s=_clamp(
            float(slam.get("active_exploration_gain_deg_s", 12.0)), 0.0, 45.0
        ),
        landmark_enabled=bool(slam.get("landmark_enabled", True)),
        landmark_default_range_m=_clamp(
            float(slam.get("landmark_default_range_m", 6.0)), 1.0, 30.0
        ),
        landmark_max_range_m=_clamp(float(slam.get("landmark_max_range_m", 12.0)), 2.0, 40.0),
        imu_yaw_assist_gain=_clamp(float(slam.get("imu_yaw_assist_gain", 0.85)), 0.0, 2.0),
        export_enabled=bool(slam.get("export_enabled", True)),
        export_path=str(slam.get("export_path", "logs/exploration_map.json")),
    )


def _cell_index(x_m: float, y_m: float, cell_m: float) -> tuple[int, int]:
    return (math.floor(x_m / cell_m), math.floor(y_m / cell_m))


def _bearing_bin(rad: float) -> int:
    return int(math.degrees(rad) % 360.0 / (360.0 / _BEARING_BINS)) % _BEARING_BINS


def _cell_center(cell: tuple[int, int], cell_m: float) -> tuple[float, float]:
    return ((cell[0] + 0.5) * cell_m, (cell[1] + 0.5) * cell_m)


class ExplorationSlam:
    def __init__(
        self,
        settings: SlamExplorationSettings,
        *,
        spawn_x_m: float,
        spawn_y_m: float,
        spawn_yaw_rad: float,
    ) -> None:
        self._s = settings
        self._spawn = (spawn_x_m, spawn_y_m)
        self._x = spawn_x_m
        self._y = spawn_y_m
        self._yaw = spawn_yaw_rad
        self._prev_x = spawn_x_m
        self._prev_y = spawn_y_m
        self._path_m = 0.0
        self._log_odds: dict[tuple[int, int], float] = {}
        self._path_xy: list[tuple[float, float]] = [(spawn_x_m, spawn_y_m)]
        self._landmarks: list[dict[str, Any]] = []
        self._bearing_visits = [0] * _BEARING_BINS
        self._loop_closures = 0
        self._last_loop_closure_s = -1e9
        self._last_scores: list[float] = []
        self._last_n_cols = 0
        self._last_center_idx = 0.0
        self._last_inverse_depth = True
        spawn_cell = _cell_index(spawn_x_m, spawn_y_m, settings.grid_cell_m)
        self._touch_log_odds(spawn_cell, _LOG_ODDS_FREE)

    def _touch_log_odds(self, cell: tuple[int, int], delta: float) -> None:
        if not self._s.log_odds_enabled:
            return
        value = self._log_odds.get(cell, 0.0) + delta
        self._log_odds[cell] = _clamp(value, -_LOG_ODDS_CLAMP, _LOG_ODDS_CLAMP)

    def _cell_state(self, cell: tuple[int, int]) -> str:
        value = self._log_odds.get(cell)
        if value is None:
            return "unknown"
        if value >= _OCC_THRESH:
            return "occupied"
        if value <= _FREE_THRESH:
            return "free"
        return "unknown"

    def update_pose(self, x_m: float, y_m: float, yaw_rad: float) -> None:
        if not self._s.enabled:
            return
        step = math.hypot(x_m - self._prev_x, y_m - self._prev_y)
        self._path_m += step
        self._prev_x, self._prev_y = self._x, self._y
        self._x, self._y = x_m, y_m
        self._yaw = yaw_rad
        if step >= 0.15:
            self._path_xy.append((x_m, y_m))
        cell = _cell_index(x_m, y_m, self._s.grid_cell_m)
        self._touch_log_odds(cell, _LOG_ODDS_FREE)
        self._bearing_visits[_bearing_bin(yaw_rad)] += 1

    def _range_from_score(self, score: float, scores: list[float], *, inverse_depth: bool) -> float:
        lo, hi = min(scores), max(scores)
        span = max(1e-6, hi - lo)
        closeness = (score - lo) / span if inverse_depth else 1.0 - (score - lo) / span
        return _clamp(
            self._s.landmark_max_range_m * (1.0 - 0.8 * closeness) + 1.5,
            1.5,
            self._s.landmark_max_range_m,
        )

    def _column_index(self, nx: float) -> int:
        if self._last_n_cols < 1:
            return 0
        center = self._last_center_idx
        col = round((float(nx) * center) + center)
        return int(_clamp(col, 0, self._last_n_cols - 1))

    def integrate_depth_columns(
        self,
        n_cols: int,
        obstacle_score: Any,
        *,
        yaw_rad: float,
        half_fov_rad: float,
        inverse_depth: bool = True,
    ) -> None:
        if not self._s.enabled or n_cols < 1:
            return
        scores = [float(obstacle_score[i]) for i in range(n_cols)]
        self._last_scores = scores
        self._last_n_cols = n_cols
        self._last_center_idx = (n_cols - 1) / 2.0
        self._last_inverse_depth = inverse_depth
        ranked = sorted(scores)
        close_threshold = ranked[min(n_cols - 1, int(0.65 * (n_cols - 1)))]
        center = self._last_center_idx
        cell_m = self._s.grid_cell_m
        for i in range(n_cols):
            offset = (i - center) / max(1.0, center)
            bearing = yaw_rad + offset * half_fov_rad
            self._bearing_visits[_bearing_bin(bearing)] += 1
            range_m = self._range_from_score(scores[i], scores, inverse_depth=inverse_depth)
            steps = max(1, int(range_m / cell_m))
            for s in range(1, steps):
                frac = s / steps
                lx = self._x + frac * range_m * math.cos(bearing)
                ly = self._y + frac * range_m * math.sin(bearing)
                self._touch_log_odds(_cell_index(lx, ly, cell_m), _LOG_ODDS_FREE)
            lx = self._x + range_m * math.cos(bearing)
            ly = self._y + range_m * math.sin(bearing)
            end_cell = _cell_index(lx, ly, cell_m)
            if scores[i] >= close_threshold:
                self._touch_log_odds(end_cell, _LOG_ODDS_OCC)
            else:
                self._touch_log_odds(end_cell, _LOG_ODDS_FREE)

    def register_landmark(
        self,
        kind: str,
        nx: float,
        *,
        half_fov_rad: float,
        range_m: float | None = None,
    ) -> None:
        if not self._s.enabled or not self._s.landmark_enabled:
            return
        if range_m is None and self._last_scores:
            col = self._column_index(nx)
            range_m = self._range_from_score(
                self._last_scores[col],
                self._last_scores,
                inverse_depth=self._last_inverse_depth,
            )
        r = range_m if range_m is not None else self._s.landmark_default_range_m
        bearing = self._yaw + float(nx) * half_fov_rad
        lx = self._x + r * math.cos(bearing)
        ly = self._y + r * math.sin(bearing)
        self._landmarks.append({"kind": kind, "x_m": lx, "y_m": ly, "range_m": r, "nx": nx})
        self._touch_log_odds(_cell_index(lx, ly, self._s.grid_cell_m), _LOG_ODDS_OCC)

    def imu_yaw_assist_deg_s(self, zgyro_rad_s: float | None) -> float:
        if not self._s.enabled or zgyro_rad_s is None:
            return 0.0
        return _clamp(
            math.degrees(zgyro_rad_s) * self._s.imu_yaw_assist_gain,
            -45.0,
            45.0,
        )

    def _yaw_bias_toward_rad(self, target_yaw_rad: float) -> float:
        err = (math.degrees(target_yaw_rad - self._yaw) + 180.0) % 360.0 - 180.0
        g = self._s.active_exploration_gain_deg_s
        return _clamp(g * (err / 90.0), -g, g)

    def _bearing_yaw_bias_deg(self) -> float:
        front = range(_BEARING_BINS // 3, 2 * _BEARING_BINS // 3)
        min_visits = min(self._bearing_visits[i] for i in front)
        target_bin = next(i for i in front if self._bearing_visits[i] == min_visits)
        return self._yaw_bias_toward_rad(math.radians(target_bin * (360.0 / _BEARING_BINS)))

    def _frontier_yaw_bias_deg(self) -> float:
        if not self._s.frontier_enabled or not self._log_odds:
            return 0.0
        cell_m = self._s.grid_cell_m
        best: tuple[float, float] | None = None
        best_dist = float("inf")
        for cell, value in self._log_odds.items():
            if value > _FREE_THRESH:
                continue
            cx, cy = _cell_center(cell, cell_m)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (cell[0] + dx, cell[1] + dy)
                if self._cell_state(neighbor) == "unknown":
                    dist = math.hypot(cx - self._x, cy - self._y)
                    if dist < best_dist:
                        best_dist = dist
                        best = (cx, cy)
        if best is None:
            return 0.0
        return self._yaw_bias_toward_rad(math.atan2(best[1] - self._y, best[0] - self._x))

    def exploration_yaw_bias_deg(self) -> float:
        if not self._s.enabled or not self._s.active_exploration_enabled:
            return 0.0
        frontier = self._frontier_yaw_bias_deg()
        if abs(frontier) > 0.5:
            return frontier
        return self._bearing_yaw_bias_deg()

    def loop_closure_detected(self) -> bool:
        if not self._s.enabled or not self._s.loop_closure_enabled:
            return False
        if self._path_m < self._s.loop_closure_min_path_m:
            return False
        dist = math.hypot(self._x - self._spawn[0], self._y - self._spawn[1])
        return dist <= self._s.loop_closure_radius_m

    def consume_loop_closure(self, now_s: float) -> bool:
        if not self.loop_closure_detected():
            return False
        if now_s - self._last_loop_closure_s < self._s.loop_closure_cooldown_s:
            return False
        self._loop_closures += 1
        self._last_loop_closure_s = now_s
        return True

    def status(self) -> SlamStatus:
        free_n = sum(1 for v in self._log_odds.values() if v <= _FREE_THRESH)
        occ_n = sum(1 for v in self._log_odds.values() if v >= _OCC_THRESH)
        known = len(self._log_odds)
        frontier_n = 0
        if self._s.frontier_enabled:
            for cell, value in self._log_odds.items():
                if value > _FREE_THRESH:
                    continue
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if self._cell_state((cell[0] + dx, cell[1] + dy)) == "unknown":
                        frontier_n += 1
                        break
        coverage = free_n / max(1, known)
        return SlamStatus(
            x_m=self._x,
            y_m=self._y,
            path_m=self._path_m,
            known_cells=known,
            free_cells=free_n,
            occupied_cells=occ_n,
            frontier_cells=frontier_n,
            landmark_count=len(self._landmarks),
            coverage_ratio=coverage,
            loop_closures=self._loop_closures,
        )

    def export_map(
        self,
        path: str | Path | None = None,
        *,
        ned: dict[str, Any] | None = None,
    ) -> Path | None:
        if not self._s.enabled or not self._s.export_enabled:
            return None
        out = Path(path or self._s.export_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        st = self.status()
        cells = [
            {"ix": c[0], "iy": c[1], "log_odds": round(v, 3), "state": self._cell_state(c)}
            for c, v in self._log_odds.items()
        ]
        payload = {
            "spawn_m": {"x": self._spawn[0], "y": self._spawn[1]},
            "path_m": self._path_m,
            "path_xy": [[x, y] for x, y in self._path_xy],
            "landmarks": self._landmarks,
            "cells": cells,
            "metrics": {
                "known_cells": st.known_cells,
                "free_cells": st.free_cells,
                "occupied_cells": st.occupied_cells,
                "frontier_cells": st.frontier_cells,
                "coverage_ratio": round(st.coverage_ratio, 4),
                "loop_closures": st.loop_closures,
            },
        }
        if ned is not None:
            payload["ned"] = ned
        out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return out
