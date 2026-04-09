"""Drone state representation using NED coordinate frame."""

from copy import deepcopy
from dataclasses import dataclass


@dataclass
class DroneState:
    """Current drone state. NED frame: negative z = above ground."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    qw: float = 1.0
    xacc: float = 0.0
    yacc: float = 0.0
    zacc: float = 0.0
    armed: bool = False
    system_status: int = 0
    timestamp_ms: int = 0

    def update(self, **kwargs) -> None:
        """Merge partial updates into current state."""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)

    def copy(self) -> "DroneState":
        """Return an independent deep copy."""
        return deepcopy(self)
