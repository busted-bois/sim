"""MAVLink message id constants and human-readable names."""

from __future__ import annotations

MAVLINK_MSG_ID_ATTITUDE = 30

MSGID_NAMES: dict[int, str] = {
    0: "HEARTBEAT",
    1: "SYS_STATUS",
    2: "SYSTEM_TIME",
    24: "GPS_RAW_INT",
    27: "RAW_IMU",
    29: "SCALED_PRESSURE",
    30: "ATTITUDE",
    31: "ATTITUDE_QUATERNION",
    32: "LOCAL_POSITION_NED",
    33: "GLOBAL_POSITION_INT",
    36: "SERVO_OUTPUT_RAW",
    65: "RC_CHANNELS",
    74: "VFR_HUD",
    76: "COMMAND_LONG",
    77: "COMMAND_ACK",
    82: "SET_ATTITUDE_TARGET",
    83: "ATTITUDE_TARGET",
    84: "SET_POSITION_TARGET_LOCAL_NED",
    85: "POSITION_TARGET_LOCAL_NED",
    87: "POSITION_TARGET_GLOBAL_INT",
    93: "HIL_STATE_QUATERNION",
    105: "HIGHRES_IMU",
    111: "TIMESYNC",
    140: "ACTUATOR_CONTROL_TARGET",
    141: "ALTITUDE",
    147: "BATTERY_STATUS",
    230: "ESTIMATOR_STATUS",
    231: "WIND_COV",
    241: "VIBRATION",
    242: "HOME_POSITION",
    245: "EXTENDED_SYS_STATE",
    253: "STATUSTEXT",
    331: "ATTITUDE_QUATERNION_COV",
    340: "UTM_GLOBAL_POSITION",
    410: "ESC_STATUS",
}


def fmt_msgid(msgid: int) -> str:
    return f"{msgid}({MSGID_NAMES.get(msgid, '?')})"
