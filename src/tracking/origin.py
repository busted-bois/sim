"""Arm-gated LOCAL_NED origin at [0, 0, 0]."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class OriginState:
    set: bool = False
    x0: float = 0.0
    y0: float = 0.0
    z0: float = 0.0
    t0_boot_ms: int = 0
    t0_sim_ns: int = 0


class OriginManager:
    def __init__(self, *, enabled: bool = True) -> None:
        self._enabled = enabled
        self._origin = OriginState()
        self._was_armed = False

    @property
    def origin(self) -> OriginState:
        return self._origin

    def on_arm_transition(self, armed: bool, *, raw_x: float, raw_y: float, raw_z: float,
                          time_boot_ms: int, sim_time_ns: int) -> bool:
        """Reset origin on rising edge of armed. Returns True if origin was (re)set."""
        if not self._enabled:
            self._was_armed = armed
            return False
        rising = armed and not self._was_armed
        self._was_armed = armed
        if rising:
            self._origin = OriginState(
                set=True,
                x0=float(raw_x),
                y0=float(raw_y),
                z0=float(raw_z),
                t0_boot_ms=int(time_boot_ms),
                t0_sim_ns=int(sim_time_ns),
            )
            return True
        return False

    def to_relative(
        self, raw_x: float, raw_y: float, raw_z: float, time_boot_ms: int
    ) -> tuple[float, float, float, float]:
        if not self._origin.set:
            return raw_x, raw_y, raw_z, time_boot_ms / 1000.0
        t_s = (int(time_boot_ms) - self._origin.t0_boot_ms) / 1000.0
        return (
            raw_x - self._origin.x0,
            raw_y - self._origin.y0,
            raw_z - self._origin.z0,
            t_s,
        )

    def sim_time_ns_from_boot_ms(self, time_boot_ms: int) -> int:
        if not self._origin.set:
            return int(time_boot_ms) * 1_000_000
        delta_ms = int(time_boot_ms) - self._origin.t0_boot_ms
        return self._origin.t0_sim_ns + delta_ms * 1_000_000
