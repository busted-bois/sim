"""Algorithm registry — base class, decorator, and factory for flight algorithms."""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import airsim
    from src.vision import VisionFeed, VisionFrame, VisionStats

_registry: dict[str, type[Algorithm]] = {}


class Algorithm:
    """Base class for flight control algorithms."""

    name: str = "base"

    def __init__(self, config: dict) -> None:
        self._config = config
        self._vision_feed: VisionFeed | None = None

    def run(self, client: airsim.MultirotorClient) -> None:
        """Execute the algorithm with full control over the AirSim client."""
        raise NotImplementedError

    def set_vision_feed(self, vision_feed: VisionFeed | None) -> None:
        self._vision_feed = vision_feed

    def latest_frame(self) -> VisionFrame | None:
        if self._vision_feed is None:
            return None
        return self._vision_feed.get_latest()

    def vision_stats(self) -> VisionStats | None:
        if self._vision_feed is None:
            return None
        return self._vision_feed.get_stats()


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


# Import built-in algorithms to trigger registration.
importlib.import_module("src.control.algorithms.six_directions")
importlib.import_module("src.control.algorithms.attitude_four_motion")
importlib.import_module("src.control.algorithms.maze_straight_collision")
