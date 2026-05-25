from __future__ import annotations

import math
import time
from itertools import pairwise

from src.control.algorithms import Algorithm, register
from src.control.flight_client import (
    SET_POSITION_FRAME_LOCAL_NED,
    FlightClient,
    SetPositionTargetLocalNedCommand,
    build_position_type_mask,
)
from src.control.primitives import takeoff_with_settle
from src.pathfinding import create_placeholder_voxel_map, plan_route_through_gates
from src.pathfinding.voxel_dijkstra import Connectivity, VoxelCoord


@register("pathfinding_explore")
class PathfindingExplore(Algorithm):
    config_section = "pathfinding_explore"

    def run(self, client: FlightClient) -> None:
        cfg = self._config.get("pathfinding_explore", {})
        control_cfg = self._config.get("control", {})
        speed_ms = max(0.2, float(cfg.get("speed_ms", control_cfg.get("max_speed_ms", 2.0))))
        waypoint_tolerance_m = max(0.1, float(cfg.get("waypoint_tolerance_m", 0.5)))
        connectivity = _connectivity(cfg.get("connectivity", 26))

        takeoff_with_settle(client, max_attempts=4, label="pathfinding_explore")

        start = _start_voxel(client, cfg)
        voxel_map = create_placeholder_voxel_map()
        path = plan_route_through_gates(voxel_map, start, connectivity=connectivity)
        print(
            "[pathfinding_explore] "
            f"start={start} gates={path.gate_order} "
            f"waypoints={len(path.coords)} cost={path.cost:.2f}"
        )

        for waypoint in _compress_collinear(path.coords)[1:]:
            print(f"[pathfinding_explore] waypoint={waypoint}")
            _fly_to_waypoint(client, waypoint, speed_ms, waypoint_tolerance_m)

        client.hoverAsync().join()


def _connectivity(raw: object) -> Connectivity:
    value = int(raw)
    if value not in {6, 26}:
        raise ValueError("pathfinding_explore.connectivity must be 6 or 26")
    return value  # type: ignore[return-value]


def _start_voxel(client: FlightClient, cfg: dict) -> VoxelCoord:
    if "start" in cfg:
        raw = cfg["start"]
        return (int(raw[0]), int(raw[1]), int(raw[2]))
    pos = client.getMultirotorState().kinematics_estimated.position
    return (round(float(pos.x_val)), round(float(pos.y_val)), round(float(pos.z_val)))


def _compress_collinear(path: tuple[VoxelCoord, ...]) -> tuple[VoxelCoord, ...]:
    if len(path) <= 2:
        return path
    compressed = [path[0]]
    last_direction = _direction(path[0], path[1])
    for prev_coord, coord in pairwise(path[1:]):
        direction = _direction(prev_coord, coord)
        if direction != last_direction:
            compressed.append(prev_coord)
        last_direction = direction
    compressed.append(path[-1])
    return tuple(compressed)


def _direction(a: VoxelCoord, b: VoxelCoord) -> VoxelCoord:
    return (b[0] - a[0], b[1] - a[1], b[2] - a[2])


def _fly_to_waypoint(
    client: FlightClient,
    waypoint: VoxelCoord,
    speed_ms: float,
    waypoint_tolerance_m: float,
) -> None:
    duration_s = _segment_duration(client, waypoint, speed_ms, waypoint_tolerance_m)
    stream_position = getattr(client, "streamSetPositionTargetLocalNedAsync", None)
    if callable(stream_position):
        command = SetPositionTargetLocalNedCommand(
            frame=SET_POSITION_FRAME_LOCAL_NED,
            type_mask=build_position_type_mask(),
            x=float(waypoint[0]),
            y=float(waypoint[1]),
            z=float(waypoint[2]),
        )
        stream_position(command, duration_s).join()
        return

    submit_position = getattr(client, "submitPositionLocalNed", None)
    if callable(submit_position):
        deadline = time.perf_counter() + duration_s
        while time.perf_counter() < deadline:
            submit_position(float(waypoint[0]), float(waypoint[1]), float(waypoint[2]))
            time.sleep(0.05)
        return

    vx, vy, vz = _velocity_toward(client, waypoint, speed_ms, waypoint_tolerance_m)
    client.moveByVelocityAsync(vx, vy, vz, duration_s).join()


def _segment_duration(
    client: FlightClient,
    waypoint: VoxelCoord,
    speed_ms: float,
    waypoint_tolerance_m: float,
) -> float:
    pos = client.getMultirotorState().kinematics_estimated.position
    distance = math.dist(
        (float(pos.x_val), float(pos.y_val), float(pos.z_val)),
        (float(waypoint[0]), float(waypoint[1]), float(waypoint[2])),
    )
    return max(0.2, max(0.0, distance - waypoint_tolerance_m) / speed_ms)


def _velocity_toward(
    client: FlightClient,
    waypoint: VoxelCoord,
    speed_ms: float,
    waypoint_tolerance_m: float,
) -> tuple[float, float, float]:
    pos = client.getMultirotorState().kinematics_estimated.position
    delta = (
        float(waypoint[0]) - float(pos.x_val),
        float(waypoint[1]) - float(pos.y_val),
        float(waypoint[2]) - float(pos.z_val),
    )
    distance = max(waypoint_tolerance_m, math.sqrt(sum(axis * axis for axis in delta)))
    scale = speed_ms / distance
    return (delta[0] * scale, delta[1] * scale, delta[2] * scale)
