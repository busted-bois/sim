"""Haphazard movement algorithm for MAVLink integration testing."""

from __future__ import annotations

import random
import time

from src.control.algorithms import Algorithm, register
from src.control.flight_client import FlightClient
from src.control.main_loop import VehicleState
from src.control.setpoints import apply_velocity_ned
from src.vision import VisionFrame

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
    name = "mavlink_jitter"
    config_section = "mavlink_jitter"
    uses_control_loop = True

    def __init__(self, config) -> None:
        super().__init__(config)
        self._tick_initialized = False
        self._rng: random.Random | None = None
        self._deadline_s = 0.0
        self._move_deadline_s = 0.0
        self._move_count = 0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._yaw_rate_dps = 0.0
        self._hover_between = True
        self._hover_dur = 0.5
        self._max_speed = 2.0
        self._move_dur = 2.0
        self._altitude_jitter = 0.5
        self._yaw_rate_cfg = 30.0
        self._period_s = 0.02

    def run(self, client: FlightClient) -> None:
        print("[mavlink_jitter] Taking off...")
        client.takeoffAsync().join()
        time.sleep(1.0)

    def run_tick(
        self,
        client: FlightClient,
        state: VehicleState,
        frame: VisionFrame | None,
    ) -> None:
        _ = state, frame
        if not self._tick_initialized:
            self._init_tick_loop()
            return

        now = time.perf_counter()
        if now >= self._deadline_s:
            print(f"[mavlink_jitter] Done -- {self._move_count} moves. Hovering.")
            client.hoverAsync().join()
            self.flight_complete = True
            return

        if now >= self._move_deadline_s:
            if self._hover_between and (self._deadline_s - now) > 1.0:
                client.hoverAsync().join()
                time.sleep(self._hover_dur)
            self._start_move(now)
            return

        apply_velocity_ned(
            client,
            self._vx,
            self._vy,
            self._vz,
            self._period_s,
            yaw_rate_dps=self._yaw_rate_dps,
        )

    def _init_tick_loop(self) -> None:
        cfg = self._config.get("mavlink_jitter", {})
        control_cfg = self._config.get("control", {})
        self._max_speed = max(0.1, float(control_cfg.get("max_speed_ms", 2.0)))
        duration_s = max(5.0, float(cfg.get("duration_s", 60.0)))
        self._move_dur = max(0.5, float(cfg.get("move_duration_s", 2.0)))
        self._yaw_rate_cfg = float(cfg.get("yaw_rate_dps", 30.0))
        self._altitude_jitter = float(cfg.get("altitude_jitter_m", 0.5))
        self._hover_between = bool(cfg.get("hover_between_moves", True))
        self._hover_dur = max(0.1, float(cfg.get("hover_duration_s", 0.5)))
        seed = cfg.get("random_seed", 42)
        rate_hz = max(5.0, float(control_cfg.get("command_rate_hz", 50.0)))
        self._period_s = 1.0 / rate_hz
        self._rng = random.Random(seed)
        self._deadline_s = time.perf_counter() + duration_s
        self._move_count = 0
        self._tick_initialized = True
        self._start_move(time.perf_counter())

    def _start_move(self, now: float) -> None:
        assert self._rng is not None
        remaining = self._deadline_s - now
        if remaining < 0.5:
            self._move_deadline_s = now
            return

        label, vx, vy, vz = self._rng.choice(_JITTER_MOVES)
        speed_factor = 0.3 + self._rng.random() * 0.7
        vx *= self._max_speed * speed_factor
        vy *= self._max_speed * speed_factor
        vz *= self._max_speed * speed_factor
        if self._rng.random() < 0.3:
            vz += self._rng.uniform(-self._altitude_jitter, self._altitude_jitter)

        this_dur = min(self._move_dur, remaining - 0.1)
        if this_dur < 0.3:
            self._move_deadline_s = now
            return

        self._move_count += 1
        self._vx = vx
        self._vy = vy
        self._vz = vz
        self._move_deadline_s = now + this_dur
        if self._move_count % 5 == 0:
            self._yaw_rate_dps = float(self._rng.choice([-1, 1]) * self._yaw_rate_cfg)
            print(
                f"[mavlink_jitter] Move #{self._move_count}: yaw "
                f"{self._yaw_rate_dps:+.0f} deg/s for {this_dur:.1f}s"
            )
        else:
            self._yaw_rate_dps = 0.0
            print(
                f"[mavlink_jitter] Move #{self._move_count}: {label} "
                f"vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} dur={this_dur:.1f}s"
            )
