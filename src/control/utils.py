import math


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _yaw_from_orientation(orientation) -> float:
    x = float(orientation.x_val)
    y = float(orientation.y_val)
    z = float(orientation.z_val)
    w = float(orientation.w_val)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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
