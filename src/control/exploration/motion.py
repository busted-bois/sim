"""Apply exploration scheduler ticks as velocity commands."""

from __future__ import annotations

from typing import TYPE_CHECKING

import airsim
from src.control.exploration.scheduler import (
    ExplorationScheduler,
    WanderTickInput,
    WanderTickOutput,
)

if TYPE_CHECKING:
    from src.control.flight_client import FlightClient


def apply_wander_move(
    client: FlightClient,
    scheduler: ExplorationScheduler,
    inp: WanderTickInput,
    *,
    cos_yaw: float,
    sin_yaw: float,
    dt: float,
) -> WanderTickOutput:
    out = scheduler.tick(inp)
    client.moveByVelocityAsync(
        out.fwd_speed * cos_yaw,
        out.fwd_speed * sin_yaw,
        out.vz,
        dt,
        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(out.yaw_rate_deg_s)),
    ).join()
    return out
