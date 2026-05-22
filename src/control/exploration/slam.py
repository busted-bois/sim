"""Lightweight SLAM-inspired helpers: pose, grid visits, landmarks, loop closure."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from src.control.utils import _clamp

_BEARING_BINS = 36


@dataclass(frozen=True)
class SlamExplorationSettings:
    enabled: bool = True
    grid_cell_m: float = 1.0
    loop_closure_enabled: bool = True
    loop_closure_radius_m: float = 2.5
    loop_closure_min_path_m: float = 8.0
    loop_closure_cooldown_s: float = 25.0
    active_exploration_enabled: bool = True
    active_exploration_gain_deg_s: float = 12.0
    landmark_enabled: bool = True
    landmark_default_range_m: float = 6.0


@dataclass(frozen=True)
class SlamStatus:
    x_m: float
    y_m: float
    path_m: float
    visited_cells: int
    landmark_count: int
    coverage_ratio: float
    loop_closures: int


def parse_slam_settings(raw: dict[str, Any] | None) -> SlamExplorationSettings:
    slam = raw or {}
    return SlamExplorationSettings(
        enabled=bool(slam.get("enabled", True)),
        grid_cell_m=_clamp(float(slam.get("grid_cell_m", 1.0)), 0.5, 5.0),
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
    )


def _cell_index(x_m: float, y_m: float, cell_m: float) -> tuple[int, int]:
    return (math.floor(x_m / cell_m), math.floor(y_m / cell_m))


def _bearing_bin(rad: float) -> int:
    deg = math.degrees(rad) % 360.0
    return int(deg / (360.0 / _BEARING_BINS)) % _BEARING_BINS


class ExplorationSlam:
    """Pose + coarse grid map for active exploration and loop closure."""

    def __init__(
        self,
        settings: SlamExplorationSettings,
        *,
        spawn_x_m: float,
        spawn_y_m: float,
        spawn_yaw_rad: float,
        start_s: float,
    ) -> None:
        self._s = settings
        self._spawn_x = spawn_x_m
        self._spawn_y = spawn_y_m
        self._x = spawn_x_m
        self._y = spawn_y_m
        self._yaw = spawn_yaw_rad
        self._prev_x = spawn_x_m
        self._prev_y = spawn_y_m
        self._path_m = 0.0
        self._start_s = start_s
        spawn_cell = _cell_index(spawn_x_m, spawn_y_m, settings.grid_cell_m)
        self._visited: set[tuple[int, int]] = {spawn_cell}
        self._obstacle: set[tuple[int, int]] = set()
        self._landmarks: list[tuple[float, float, str]] = []
        self._bearing_visits = [0] * _BEARING_BINS
        self._loop_closures = 0
        self._last_loop_closure_s = -1e9

    def update_pose(self, x_m: float, y_m: float, yaw_rad: float) -> None:
        if not self._s.enabled:
            return
        step = math.hypot(x_m - self._prev_x, y_m - self._prev_y)
        self._path_m += step
        self._prev_x, self._prev_y = self._x, self._y
        self._x, self._y = x_m, y_m
        self._yaw = yaw_rad
        self._visited.add(_cell_index(x_m, y_m, self._s.grid_cell_m))
        self._bearing_visits[_bearing_bin(yaw_rad)] += 1

    def integrate_depth_columns(
        self,
        n_cols: int,
        obstacle_score: Any,
        *,
        yaw_rad: float,
        half_fov_rad: float,
    ) -> None:
        if not self._s.enabled or n_cols < 1:
            return
        scores = [float(obstacle_score[i]) for i in range(n_cols)]
        ranked = sorted(scores)
        close_threshold = ranked[min(n_cols - 1, int(0.65 * (n_cols - 1)))]
        center = (n_cols - 1) / 2.0
        for i in range(n_cols):
            offset = (i - center) / max(1.0, center)
            bearing = yaw_rad + offset * half_fov_rad
            self._bearing_visits[_bearing_bin(bearing)] += 1
            if scores[i] < close_threshold:
                continue
            range_m = self._s.landmark_default_range_m * (1.0 - abs(offset) * 0.35)
            lx = self._x + range_m * math.cos(bearing)
            ly = self._y + range_m * math.sin(bearing)
            self._obstacle.add(_cell_index(lx, ly, self._s.grid_cell_m))

    def register_landmark(self, kind: str, nx: float, *, half_fov_rad: float) -> None:
        if not self._s.enabled or not self._s.landmark_enabled:
            return
        bearing = self._yaw + float(nx) * half_fov_rad
        r = self._s.landmark_default_range_m
        lx = self._x + r * math.cos(bearing)
        ly = self._y + r * math.sin(bearing)
        self._landmarks.append((lx, ly, kind))
        self._visited.add(_cell_index(lx, ly, self._s.grid_cell_m))

    def active_exploration_yaw_bias_deg(self) -> float:
        if not self._s.enabled or not self._s.active_exploration_enabled:
            return 0.0
        front_bins = range(_BEARING_BINS // 3, 2 * _BEARING_BINS // 3)
        min_visits = min(self._bearing_visits[i] for i in front_bins)
        candidates = [i for i in front_bins if self._bearing_visits[i] == min_visits]
        target_bin = candidates[len(candidates) // 2]
        target_yaw = math.radians(target_bin * (360.0 / _BEARING_BINS))
        err = math.degrees(target_yaw - self._yaw)
        err = (err + 180.0) % 360.0 - 180.0
        return _clamp(
            self._s.active_exploration_gain_deg_s * (err / 90.0),
            -self._s.active_exploration_gain_deg_s,
            self._s.active_exploration_gain_deg_s,
        )

    def loop_closure_detected(self) -> bool:
        if not self._s.enabled or not self._s.loop_closure_enabled:
            return False
        if self._path_m < self._s.loop_closure_min_path_m:
            return False
        dist = math.hypot(self._x - self._spawn_x, self._y - self._spawn_y)
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
        total = max(1, len(self._visited) + len(self._obstacle))
        coverage = len(self._visited) / total
        return SlamStatus(
            x_m=self._x,
            y_m=self._y,
            path_m=self._path_m,
            visited_cells=len(self._visited),
            landmark_count=len(self._landmarks),
            coverage_ratio=coverage,
            loop_closures=self._loop_closures,
        )
