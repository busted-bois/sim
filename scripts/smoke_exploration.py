#!/usr/bin/env python3
"""Smoke: exploration scheduler + autonomous_explore registration."""

from __future__ import annotations

import sys

from src.control.algorithms import get_algorithm, list_algorithms
from src.control.exploration import (
    ExplorationScheduler,
    parse_exploration_settings,
)


def main() -> int:
    if "autonomous_explore" not in list_algorithms():
        print("FAIL: autonomous_explore not registered")
        return 1
    algo = get_algorithm("autonomous_explore", {"autonomous_explore": {}})
    if not hasattr(algo, "latest_sensor_snapshot"):
        print("FAIL: missing latest_sensor_snapshot")
        return 1
    settings = parse_exploration_settings(
        {"panorama_enabled": True, "panorama_interval_s": 15.0},
        hold_altitude_m=5.0,
        max_altitude_m=50.0,
    )
    sched = ExplorationScheduler(settings, start_s=0.0, initial_yaw_rad=0.0, initial_z_ned=-5.0)
    if sched.z_hold_ned != -5.0:
        print(f"FAIL: z_hold_ned={sched.z_hold_ned}")
        return 1
    print("OK exploration smoke")
    return 0


if __name__ == "__main__":
    sys.exit(main())
