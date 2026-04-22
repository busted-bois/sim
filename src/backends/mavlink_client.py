from __future__ import annotations

import math
import time
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

from pymavlink import mavutil


@dataclass
class _CompletedAction:
    def join(self) -> None:
        return None


class MavlinkClient:
    """Minimal AirSim-like client surface backed by MAVLink."""

    def __init__(self, connection: str, *, source_system: int = 255, source_component: int = 0):
        self._master = mavutil.mavlink_connection(
            connection,
            source_system=source_system,
            source_component=source_component,
        )
        self._target_system = 0
        self._target_component = 0
        self._last_local_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def confirmConnection(self) -> None:
        hb = self._master.wait_heartbeat(timeout=15)
        if hb is None:
            raise TimeoutError("MAVLink heartbeat timeout (15s).")
        self._target_system = int(self._master.target_system or hb.get_srcSystem() or 1)
        self._target_component = int(self._master.target_component or hb.get_srcComponent() or 1)
        print(
            "Connected! MAVLink "
            f"target={self._target_system}:{self._target_component}"
        )

    def enableApiControl(self, is_enabled: bool, vehicle_name: str = "") -> None:
        # MAVLink has no direct API-control equivalent; noop for parity.
        _ = (is_enabled, vehicle_name)

    def armDisarm(self, arm: bool, vehicle_name: str = "") -> bool:
        _ = vehicle_name
        self._master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return True

    def takeoffAsync(self, timeout_sec: float = 20, vehicle_name: str = "") -> _CompletedAction:
        _ = (timeout_sec, vehicle_name)
        self._master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            math.nan,
            0,
            0,
            3.0,
        )
        return _CompletedAction()

    def landAsync(self, timeout_sec: float = 60, vehicle_name: str = "") -> _CompletedAction:
        _ = (timeout_sec, vehicle_name)
        self._master.mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            math.nan,
            0,
            0,
            0,
        )
        return _CompletedAction()

    def hoverAsync(self, vehicle_name: str = "") -> _CompletedAction:
        _ = vehicle_name
        return self.moveByVelocityAsync(0.0, 0.0, 0.0, 0.2)

    def moveByVelocityAsync(
        self, vx: float, vy: float, vz: float, duration: float, vehicle_name: str = ""
    ) -> _CompletedAction:
        _ = vehicle_name
        self._master.mav.set_position_target_local_ned_send(
            0,
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity-only setpoint
            0,
            0,
            0,
            float(vx),
            float(vy),
            float(vz),
            0,
            0,
            0,
            0,
            0,
        )
        if duration > 0:
            time.sleep(float(duration))
        return _CompletedAction()

    def getMultirotorState(self) -> Any:
        msg = self._master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg is not None:
            self._last_local_pos = (float(msg.x), float(msg.y), float(msg.z))
        x, y, z = self._last_local_pos
        return SimpleNamespace(
            kinematics_estimated=SimpleNamespace(
                position=SimpleNamespace(x_val=x, y_val=y, z_val=z)
            )
        )

    def simSetCameraPose(self, camera_name: str, pose: Any, vehicle_name: str = "") -> None:
        _ = (camera_name, pose, vehicle_name)

    def simSetTraceLine(self, color_rgba: list[float], thickness: float, vehicle_name: str = "") -> None:
        _ = (color_rgba, thickness, vehicle_name)

