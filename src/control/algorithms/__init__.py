"""Algorithm registry — base class, decorator, and factory for flight algorithms."""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import TYPE_CHECKING, Any

from src.control.flight_client import FlightClient
from src.control.highres_imu import HighresImuHealth, HighresImuSample, SensorSnapshot

if TYPE_CHECKING:
    from src.config import Config
    from src.vision import VisionFeed, VisionFrame, VisionStats

_registry: dict[str, type[Algorithm]] = {}


class Algorithm:
    """Base class for flight control algorithms."""

    name: str = "base"
    config_section: str | None = None

    def __init__(self, config: Config | dict[str, Any]) -> None:
        self._config = config
        self._vision_feed: VisionFeed | None = None

    def run(self, client: FlightClient) -> None:
        """Execute the algorithm with full control over the flight client."""
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

    def latest_highres_imu(self, client: FlightClient) -> HighresImuSample | None:
        getter = getattr(client, "getHighresImu", None)
        if not callable(getter):
            return None
        return getter()

    def highres_imu_health(self, client: FlightClient) -> HighresImuHealth | None:
        getter = getattr(client, "getHighresImuHealth", None)
        if not callable(getter):
            return None
        return getter()

    def latest_sensor_snapshot(self, client: FlightClient) -> SensorSnapshot | None:
        getter = getattr(client, "getSensorSnapshot", None)
        if not callable(getter):
            return None
        return getter()


def register(name: str):
    """Decorator to register an algorithm class by name."""

    def decorator(cls: type[Algorithm]) -> type[Algorithm]:
        cls.name = name
        _registry[name] = cls
        return cls

    return decorator


def get_algorithm(name: str, config: Config | dict[str, Any]) -> Algorithm:
    """Instantiate an algorithm by name from the registry."""
    if name not in _registry:
        available = ", ".join(_registry.keys()) or "(none)"
        raise ValueError(f"Unknown algorithm '{name}'. Available: {available}")
    algo = _registry[name](config)
    if algo.config_section and algo.config_section not in config:
        raise ValueError(
            f"Algorithm '{name}' requires config section '{algo.config_section}' "
            f"but it was not found in the config."
        )
    return algo


def list_algorithms() -> list[str]:
    """Return sorted list of registered algorithm names."""
    return sorted(_registry.keys())


_algorithms_dir = Path(__file__).parent
for module_path in _algorithms_dir.glob("*.py"):
    if module_path.name.startswith("_") or module_path.name == "__init__.py":
        continue
    module_name = f"src.control.algorithms.{module_path.stem}"
    importlib.import_module(module_name)
