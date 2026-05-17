"""Haphazard movement algorithm for MAVLink integration testing."""

from __future__ import annotations

import random
import time

from src.control.algorithms import Algorithm, register
from src.control.flight_client import FlightClient

_JITTER_MOVES = [
    ("forward", 1.0, 0.0, 0.0),
    ("backward", -1.0, 0.0, 0.0),
    ("left", 0.0, -1.0, 0.0),
    ("right", 0.0, 1.0, 0.0),
    ("up", 0.0, 0.0, -0.5),
    ("down", 0.0, 0.0, 0.5),
    ("diag-nw", 0.7, -0.7, 0.0),
    ("diag-ne", 0.7, 0.7, 0.0),
    ("diag-sw", -0.7, -0.7, 0.0),
    ("diag-se", -0.7, 0.7, 0.0),
    ("spin-nfwd", 0.5, 0.0, -0.3),
    ("spin-nback", -0.5, 0.0, -0.3),
    ("strafe-up", 0.0, 1.0, -0.5),
    ("strafe-dn", 0.0, -1.0, 0.5),
]


@register("mavlink_jitter")
class MavlinkJitter(Algorithm):
    """Haphazard movement to stress-test MAVLink command pipeline."""

    name = "mavlink_jitter"
    config_section = "mavlink_jitter"

    def run(self, client: FlightClient) -> None:
        cfg = self._config.get("mavlink_jitter", {})
        control_cfg = self._config.get("control", {})
        max_speed = max(0.1, float(control_cfg.get("max_speed_ms", 2.0)))
        duration_s = max(5.0, float(cfg.get("duration_s", 60.0)))
        move_dur = max(0.5, float(cfg.get("move_duration_s", 2.0)))
        yaw_rate = float(cfg.get("yaw_rate_dps", 30.0))
        altitude_jitter = float(cfg.get("altitude_jitter_m", 0.5))
        hover_between = bool(cfg.get("hover_between_moves", True))
        hover_dur = max(0.1, float(cfg.get("hover_duration_s", 0.5)))
        seed = cfg.get("random_seed", 42)

        rng = random.Random(seed)

        print("[mavlink_jitter] Taking off...")
        client.takeoffAsync().join()
        self._log_state(client, "post_takeoff")
        time.sleep(1.0)

        start = time.perf_counter()
        move_count = 0

        while time.perf_counter() - start < duration_s:
            remaining = duration_s - (time.perf_counter() - start)
            if remaining < 0.5:
                break

            label, vx, vy, vz = rng.choice(_JITTER_MOVES)
            speed_factor = 0.3 + rng.random() * 0.7
            vx *= max_speed * speed_factor
            vy *= max_speed * speed_factor
            vz *= max_speed * speed_factor

            if rng.random() < 0.3:
                vz += rng.uniform(-altitude_jitter, altitude_jitter)

            this_dur = min(move_dur, remaining - 0.1)
            if this_dur < 0.3:
                break

            move_count += 1
            print(
                f"[mavlink_jitter] Move #{move_count}: {label} "
                f"vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} "
                f"dur={this_dur:.1f}s ({remaining:.1f}s remaining)"
            )

            if move_count % 5 == 0:
                rate = rng.choice([-1, 1]) * yaw_rate
                print(
                    f"[mavlink_jitter]   -> yaw rotation "
                    f"{rate:+.0f} deg/s for {this_dur:.1f}s"
                )
                client.rotateByYawRateAsync(rate, this_dur).join()
            else:
                client.moveByVelocityAsync(vx, vy, vz, this_dur).join()

            self._log_state(client, f"after_move_{move_count}")

            if hover_between and remaining > 1.0:
                client.hoverAsync().join()
                time.sleep(hover_dur)

        print(
            f"[mavlink_jitter] Done -- {move_count} moves in "
            f"{duration_s:.0f}s. Hovering."
        )
        client.hoverAsync().join()
        time.sleep(1.0)

    def _log_state(self, client: FlightClient, label: str) -> None:
        try:
            state = client.getMultirotorState()
            kin = state.kinematics_estimated
            pos = kin.position
            vel = kin.linear_velocity
            speed = (vel.x_val**2 + vel.y_val**2 + vel.z_val**2) ** 0.5
            print(
                f"[mavlink_jitter]   state@{label}: "
                f"pos=({pos.x_val:.2f}, {pos.y_val:.2f}, {pos.z_val:.2f}) "
                f"speed={speed:.2f} m/s"
            )
        except Exception as exc:
            print(f"[mavlink_jitter]   state@{label}: error: {exc}")
