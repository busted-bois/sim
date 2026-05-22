"""Apply exploration scheduler ticks as velocity commands."""

from __future__ import annotations

from typing import TYPE_CHECKING

import airsim
from src.control.exploration.scheduler import (
    ExplorationScheduler,
    WanderTickInput,
    WanderTickOutput,
)
from src.control.ned_environment import local_velocity_forward

if TYPE_CHECKING:
    from src.control.flight_client import FlightClient
    from src.control.ned_environment import NedEnvironmentMap


def apply_wander_move(
    client: FlightClient,
    scheduler: ExplorationScheduler,
    inp: WanderTickInput,
    *,
    cos_yaw: float,
    sin_yaw: float,
    dt: float,
    ned: NedEnvironmentMap | None = None,
) -> WanderTickOutput:
    out = scheduler.tick(inp)
    if ned is not None:
        vel = local_velocity_forward(ned, out.fwd_speed, out.vz, yaw_rad=inp.yaw_rad)
        vx, vy, vz = vel.vx, vel.vy, vel.vz
    else:
        vx = out.fwd_speed * cos_yaw
        vy = out.fwd_speed * sin_yaw
        vz = out.vz
    client.moveByVelocityAsync(
        vx,
        vy,
        vz,
        dt,
        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(out.yaw_rate_deg_s)),
    ).join()
    return out
