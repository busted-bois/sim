from __future__ import annotations

import os
import sys
import time
from typing import Any


def is_maze_mode() -> bool:
    return os.environ.get("AIGP_MAZE", "").strip() == "1"


def choose_algorithm_name(config: dict[str, Any]) -> str:
    """Resolve active algorithm with explicit env override support.

    Priority:
    1) AIGP_ALGORITHM (always wins)
    2) AIGP_MAZE_ALGORITHM (maze mode only)
    3) config["maze_algorithm"] (maze mode only)
    4) config["algorithm"]
    """
    explicit = os.environ.get("AIGP_ALGORITHM", "").strip()
    if explicit:
        return explicit

    if is_maze_mode():
        maze_explicit = os.environ.get("AIGP_MAZE_ALGORITHM", "").strip()
        if maze_explicit:
            return maze_explicit
        maze_cfg = str(config.get("maze_algorithm", "")).strip()
        if maze_cfg:
            return maze_cfg

    return str(config.get("algorithm", "attitude_four_motion")).strip()


def takeoff_with_retries(
    client: Any,
    *,
    timeout_sec: float,
    attempts: int = 4,
    retry_backoff_s: float = 1.5,
    log_prefix: str = "flight",
) -> None:
    for attempt in range(1, max(1, attempts) + 1):
        try:
            client.takeoffAsync(timeout_sec=timeout_sec).join()
            return
        except Exception as exc:
            if attempt == attempts:
                raise
            print(
                f"[{log_prefix}] takeoff attempt {attempt}/{attempts} failed "
                f"({type(exc).__name__}: {exc}); retrying...",
                file=sys.stderr,
            )
            time.sleep(retry_backoff_s * attempt)


def move_planar(
    client: Any, *, vx: float, vy: float, duration: float, z_ref: float | None = None
) -> None:
    move_planar_async = getattr(client, "movePlanarAsync", None)
    if callable(move_planar_async):
        move_planar_async(vx, vy, duration, z_ref).join()
        return
    client.moveByVelocityAsync(vx, vy, 0.0, duration).join()
