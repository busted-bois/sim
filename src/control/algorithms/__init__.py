"""Algorithm registry — base class, decorator, and factory for flight algorithms."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.comms.state import DroneState
    from src.planning.waypoint import VelocityCommand

_registry: dict[str, type[Algorithm]] = {}


class Algorithm:
    """Base class for flight control algorithms."""

    name: str = "base"

    def __init__(self, config: dict) -> None:
        self._config = config

    def compute(self, state: DroneState) -> VelocityCommand | None:
        raise NotImplementedError


def register(name: str):
    """Decorator to register an algorithm class by name."""

    def decorator(cls: type[Algorithm]) -> type[Algorithm]:
        cls.name = name
        _registry[name] = cls
        return cls

    return decorator


def get_algorithm(name: str, config: dict) -> Algorithm:
    """Instantiate an algorithm by name from the registry."""
    if name not in _registry:
        available = ", ".join(_registry.keys()) or "(none)"
        raise ValueError(f"Unknown algorithm '{name}'. Available: {available}")
    return _registry[name](config)


def list_algorithms() -> list[str]:
    """Return sorted list of registered algorithm names."""
    return sorted(_registry.keys())


# Import built-in algorithms to trigger registration
from src.control.algorithms import six_directions  # noqa: E402, F401
