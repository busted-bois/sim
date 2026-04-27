from __future__ import annotations

import sys
import time

from src.control.algorithms import Algorithm, register


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@register("maze_straight_collision")
class MazeStraightCollision(Algorithm):
    """Maze behavior: fly straight until collision, then finish for landing."""

    def run(self, client) -> None:
        cfg = self._config.get("maze_straight_collision", {})
        control_cfg = self._config.get("control", {})
        max_speed_ms = max(0.5, float(control_cfg.get("max_speed_ms", 10.0)))
        speed_ms = _clamp(float(cfg.get("speed_ms", 1.8)), 0.3, max_speed_ms)
        command_chunk_s = _clamp(float(cfg.get("command_chunk_s", 0.20)), 0.05, 1.0)
        probe_duration_s = _clamp(float(cfg.get("probe_duration_s", 0.5)), 0.15, 1.5)
        max_flight_s = max(3.0, float(cfg.get("max_flight_s", 60.0)))
        target_altitude_m = max(1.0, float(cfg.get("target_altitude_m", 2.5)))
        target_z = -target_altitude_m
        collision_confirm_ticks = max(1, int(cfg.get("collision_confirm_ticks", 2)))
        stall_confirm_ticks = max(2, int(cfg.get("stall_confirm_ticks", 6)))
        min_progress_m = max(0.02, float(cfg.get("min_progress_m", 0.05)))
        takeoff_timeout_sec = max(10.0, float(cfg.get("takeoff_timeout_sec", 60.0)))
        collision_rpc_available = True

        print(
            "[maze_straight_collision] init "
            f"speed_ms={speed_ms:.2f} chunk_s={command_chunk_s:.2f} "
            f"probe_s={probe_duration_s:.2f} max_flight_s={max_flight_s:.1f} "
            f"target_z={target_z:.2f}"
        )

        if hasattr(client, "movement_backend"):
            backend = client.movement_backend()
            print(f"[maze_straight_collision] movement_backend={backend}")

        takeoff_attempts = 4
        for attempt in range(1, takeoff_attempts + 1):
            try:
                client.takeoffAsync(timeout_sec=takeoff_timeout_sec).join()
                break
            except Exception as exc:
                if attempt == takeoff_attempts:
                    raise
                print(
                    f"[maze_straight_collision] takeoff attempt {attempt}/{takeoff_attempts} "
                    f"failed ({type(exc).__name__}: {exc}); retrying...",
                    file=sys.stderr,
                )
                time.sleep(1.5 * attempt)

        fwd_vx, fwd_vy, direction_label = self._probe_forward_direction(
            client=client, speed_ms=speed_ms, probe_duration_s=probe_duration_s
        )
        print(
            "[maze_straight_collision] selected_forward "
            f"label={direction_label} command(vx,vy)=({fwd_vx:.2f},{fwd_vy:.2f})"
        )

        started_s = time.perf_counter()
        steps = 0
        collision_streak = 0
        stall_streak = 0
        prev_state = client.getMultirotorState().kinematics_estimated
        prev_x = float(prev_state.position.x_val)
        prev_y = float(prev_state.position.y_val)
        while time.perf_counter() - started_s < max_flight_s:
            steps += 1
            self._move_planar(
                client, vx=fwd_vx, vy=fwd_vy, duration=command_chunk_s, z_ref=target_z
            )

            collision = None
            if collision_rpc_available:
                try:
                    collision = client.simGetCollisionInfo()
                    if bool(getattr(collision, "has_collided", False)):
                        collision_streak += 1
                    else:
                        collision_streak = 0
                except Exception as exc:
                    collision_rpc_available = False
                    collision_streak = 0
                    print(
                        "[maze_straight_collision] collision RPC unavailable; "
                        f"using stall-only wall detection ({type(exc).__name__}: {exc})"
                    )

            state = client.getMultirotorState().kinematics_estimated
            x = float(state.position.x_val)
            y = float(state.position.y_val)
            progress = ((x - prev_x) ** 2 + (y - prev_y) ** 2) ** 0.5
            prev_x, prev_y = x, y
            if progress < min_progress_m:
                stall_streak += 1
            else:
                stall_streak = 0

            if steps == 1 or steps % 10 == 0:
                print(
                    "[maze_straight_collision] progress "
                    f"step={steps} pos=({x:.2f},{y:.2f},{float(state.position.z_val):.2f}) "
                    f"delta_m={progress:.3f} coll_streak={collision_streak} "
                    f"stall_streak={stall_streak}"
                )

            if collision is not None and collision_streak >= collision_confirm_ticks:
                print(
                    "[maze_straight_collision] collision detected "
                    f"step={steps} object={getattr(collision, 'object_name', '')!r} "
                    f"impact={getattr(collision, 'impact_point', None)!r}"
                )
                break
            if stall_streak >= stall_confirm_ticks:
                print(
                    "[maze_straight_collision] stall detected "
                    f"step={steps} progress<{min_progress_m:.3f}m for {stall_streak} ticks; ending."
                )
                break
        else:
            print(
                "[maze_straight_collision] max_flight_s reached without collision; "
                "ending run and landing."
            )

        client.hoverAsync().join()
        print("[maze_straight_collision] complete — ready for landing sequence")

    def _probe_forward_direction(self, *, client, speed_ms: float, probe_duration_s: float):
        # Probe lateral axis/sign in this map by measuring world-frame displacement.
        probes = [
            ("+X", speed_ms, 0.0),
            ("-X", -speed_ms, 0.0),
            ("+Y", 0.0, speed_ms),
            ("-Y", 0.0, -speed_ms),
        ]
        baseline = client.getMultirotorState().kinematics_estimated
        x0 = float(baseline.position.x_val)
        y0 = float(baseline.position.y_val)
        best = ("+X", speed_ms, 0.0, 0.0)
        print(
            "[maze_straight_collision] direction_probe start "
            f"from=({x0:.2f},{y0:.2f})"
        )

        for label, vx, vy in probes:
            try:
                self._move_planar(client, vx=vx, vy=vy, duration=probe_duration_s, z_ref=None)
                state = client.getMultirotorState().kinematics_estimated
                dx = float(state.position.x_val) - x0
                dy = float(state.position.y_val) - y0
                score = (dx * dx + dy * dy) ** 0.5
                print(
                    "[maze_straight_collision] direction_probe "
                    f"label={label} delta=({dx:.2f},{dy:.2f}) score={score:.2f}"
                )
                if score > best[3]:
                    best = (label, vx, vy, score)
                client.hoverAsync().join()
                time.sleep(0.15)
            except Exception as exc:
                print(
                    f"[maze_straight_collision] direction_probe label={label} failed "
                    f"({type(exc).__name__}: {exc})",
                    file=sys.stderr,
                )

        if best[3] <= 0.05:
            print(
                "[maze_straight_collision] direction_probe weak movement; "
                "defaulting to +X command.",
                file=sys.stderr,
            )
            return speed_ms, 0.0, "+X(default)"
        return best[1], best[2], best[0]

    @staticmethod
    def _move_planar(client, *, vx: float, vy: float, duration: float, z_ref: float | None) -> None:
        move_planar = getattr(client, "movePlanarAsync", None)
        if callable(move_planar):
            move_planar(vx, vy, duration, z_ref).join()
            return
        client.moveByVelocityAsync(vx, vy, 0.0, duration).join()
