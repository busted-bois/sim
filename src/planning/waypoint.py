"""Placeholder waypoint following algorithm."""

import math
from dataclasses import dataclass


@dataclass
class VelocityCommand:
    """Velocity command in NED frame (m/s)."""

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0


class WaypointFollower:
    """Follows a list of waypoints sequentially using simple proportional control.

    Waypoints are (x, y, z) tuples in NED frame.
    Advances to next waypoint when within acceptance_radius meters.
    """

    def __init__(
        self,
        waypoints: list[tuple[float, float, float]],
        speed_ms: float = 2.0,
        acceptance_radius: float = 1.0,
    ):
        self._waypoints = waypoints
        self._speed = speed_ms
        self._acceptance_radius = acceptance_radius
        self._current_index = 0

    @property
    def current_waypoint(self) -> tuple[float, float, float] | None:
        """Returns current target waypoint or None if all reached."""
        if self._current_index < len(self._waypoints):
            return self._waypoints[self._current_index]
        return None

    @property
    def all_reached(self) -> bool:
        """Returns True when all waypoints have been reached."""
        return self._current_index >= len(self._waypoints)

    def update(self, x: float, y: float, z: float) -> VelocityCommand:
        """Compute velocity command toward current waypoint.

        Args:
            x, y, z: Current position in NED frame.

        Returns:
            VelocityCommand with desired velocities.
        """
        wp = self.current_waypoint
        if wp is None:
            return VelocityCommand()

        dx = wp[0] - x
        dy = wp[1] - y
        dz = wp[2] - z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self._acceptance_radius:
            self._current_index += 1
            wp = self.current_waypoint
            if wp is None:
                return VelocityCommand()
            dx = wp[0] - x
            dy = wp[1] - y
            dz = wp[2] - z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-6:
            return VelocityCommand()

        vx = dx / dist * self._speed
        vy = dy / dist * self._speed
        vz = dz / dist * self._speed
        yaw = math.atan2(dy, dx)

        return VelocityCommand(vx=vx, vy=vy, vz=vz, yaw=yaw)

    def reset(self) -> None:
        """Reset to first waypoint."""
        self._current_index = 0
