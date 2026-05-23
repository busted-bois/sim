"""Strapdown IMU propagation in LOCAL_NED (gyro attitude, accel velocity/position)."""

from __future__ import annotations

import math
from dataclasses import dataclass

GRAVITY_MS2 = 9.80665


@dataclass
class PropagatedState:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    last_time_usec: int | None = None


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rotate_body_to_ned(
    ax: float, ay: float, az: float, roll: float, pitch: float, yaw: float
) -> tuple[float, float, float]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll); body accel to NED
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr
    nx = r00 * ax + r01 * ay + r02 * az
    ny = r10 * ax + r11 * ay + r12 * az
    nz = r20 * ax + r21 * ay + r22 * az
    return nx, ny, nz


class ImuPropagator:
    def __init__(
        self,
        *,
        max_speed_ms: float = 10.0,
        plausible_step_slack: float = 1.5,
    ) -> None:
        self._max_speed_ms = max(0.1, float(max_speed_ms))
        self._plausible_step_slack = max(1.0, float(plausible_step_slack))
        self._state = PropagatedState()

    @property
    def state(self) -> PropagatedState:
        return self._state

    def reset(self, *, x: float = 0.0, y: float = 0.0, z: float = 0.0,
              roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        self._state = PropagatedState(
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, last_time_usec=None
        )

    def set_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        self._state.roll = float(roll)
        self._state.pitch = float(pitch)
        self._state.yaw = _wrap_pi(float(yaw))

    def set_position_velocity(
        self,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        self._state.x = float(x)
        self._state.y = float(y)
        self._state.z = float(z)
        self._state.vx = float(vx)
        self._state.vy = float(vy)
        self._state.vz = float(vz)

    def step(
        self,
        time_usec: int,
        xacc: float | None,
        yacc: float | None,
        zacc: float | None,
        xgyro: float | None,
        ygyro: float | None,
        zgyro: float | None,
    ) -> bool:
        if None in (xacc, yacc, zacc, xgyro, ygyro, zgyro):
            return False
        t_us = int(time_usec)
        if self._state.last_time_usec is None:
            self._state.last_time_usec = t_us
            return True
        dt = (t_us - self._state.last_time_usec) / 1_000_000.0
        if dt <= 0.0 or dt > 0.5:
            self._state.last_time_usec = t_us
            return False

        gx, gy, gz = float(xgyro), float(ygyro), float(zgyro)
        self._state.roll += gx * dt
        self._state.pitch += gy * dt
        self._state.yaw = _wrap_pi(self._state.yaw + gz * dt)

        ax, ay, az = float(xacc), float(yacc), float(zacc)
        st = self._state
        nx, ny, nz = rotate_body_to_ned(ax, ay, az, st.roll, st.pitch, st.yaw)
        nx, ny, nz = nx, ny, nz + GRAVITY_MS2  # NED: +g on Z (down)

        self._state.vx += nx * dt
        self._state.vy += ny * dt
        self._state.vz += nz * dt

        step_m = math.sqrt(
            (self._state.vx * dt) ** 2 + (self._state.vy * dt) ** 2 + (self._state.vz * dt) ** 2
        )
        max_step = self._max_speed_ms * dt * self._plausible_step_slack
        if step_m > max_step:
            scale = max_step / max(step_m, 1e-9)
            self._state.vx *= scale
            self._state.vy *= scale
            self._state.vz *= scale

        self._state.x += self._state.vx * dt
        self._state.y += self._state.vy * dt
        self._state.z += self._state.vz * dt
        self._state.last_time_usec = t_us
        return True
