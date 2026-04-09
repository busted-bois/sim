"""Six-direction test algorithm: moves +X, -X, +Y, -Y, +Z, -Z for 5s each at 2 m/s."""

import time

from src.control.algorithms import Algorithm, register
from src.planning.waypoint import VelocityCommand

DIRECTIONS = [
    ("+X", 2.0, 0.0, 0.0),
    ("-X", -2.0, 0.0, 0.0),
    ("+Y", 0.0, 2.0, 0.0),
    ("-Y", 0.0, -2.0, 0.0),
    ("+Z", 0.0, 0.0, 2.0),
    ("-Z", 0.0, 0.0, -2.0),
]
DURATION_S = 5.0


@register("six_directions")
class SixDirections(Algorithm):
    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self._start_time: float | None = None
        self._last_direction: str = ""

    def compute(self, state) -> VelocityCommand | None:
        if self._start_time is None:
            self._start_time = time.monotonic()

        elapsed = time.monotonic() - self._start_time
        total = len(DIRECTIONS) * DURATION_S

        if elapsed >= total:
            if self._last_direction != "hover":
                print("[six_directions] Complete — hovering")
                self._last_direction = "hover"
            return VelocityCommand(0, 0, 0)

        idx = min(int(elapsed / DURATION_S), len(DIRECTIONS) - 1)
        label, vx, vy, vz = DIRECTIONS[idx]

        if label != self._last_direction:
            print(f"[six_directions] Moving {label}")
            self._last_direction = label

        return VelocityCommand(vx, vy, vz)
