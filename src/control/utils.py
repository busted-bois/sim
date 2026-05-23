import math
from dataclasses import dataclass


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _yaw_from_orientation(orientation) -> float:
    x = float(orientation.x_val)
    y = float(orientation.y_val)
    z = float(orientation.z_val)
    w = float(orientation.w_val)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True, slots=True)
class Quaternionr:
    x_val: float = 0.0
    y_val: float = 0.0
    z_val: float = 0.0
    w_val: float = 1.0


def orientation_from_yaw(yaw_rad: float) -> Quaternionr:
    """Quaternion (x,y,z,w) for pure yaw about NED down axis."""
    half = float(yaw_rad) * 0.5
    return Quaternionr(0.0, 0.0, math.sin(half), math.cos(half))


def orientation_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternionr:
    """Quaternion from MAVLink NED roll/pitch/yaw (rad)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternionr(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_orientation(orientation) -> tuple[float, float, float]:
    x = float(orientation.x_val)
    y = float(orientation.y_val)
    z = float(orientation.z_val)
    w = float(orientation.w_val)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = _yaw_from_orientation(orientation)
    return roll, pitch, yaw


def make_vz_trim(client, z_hold: float):
    def vz_trim() -> float:
        z = float(client.getMultirotorState().kinematics_estimated.position.z_val)
        err = z - z_hold
        if err < -0.3:
            return 0.35
        if err > 0.3:
            return -0.35
        return 0.0

    return vz_trim
