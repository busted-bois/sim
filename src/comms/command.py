"""MAVLink SET_POSITION_TARGET_LOCAL_NED velocity command sender over UDP."""

import socket
import struct

from src.comms.mavlink_parser import MAVLINK_STX_V2, mavlink_crc

# SET_POSITION_TARGET_LOCAL_NED (msg 83)
SET_POSITION_TARGET_LOCAL_NED_MSG_ID = 83
SET_POSITION_TARGET_LOCAL_NED_CRC_EXTRA = 0x4F  # 79
MAV_FRAME_LOCAL_NED = 1

# Payload: time_boot_ms(I) target_sys(B) target_comp(B) coord_frame(B) type_mask(I)
#           x(f) y(f) z(f) vx(f) vy(f) vz(f) afx(f) afy(f) afz(f) yaw(f) yaw_rate(f)
PAYLOAD_FMT = "<I3BI11f"
PAYLOAD_LEN = struct.calcsize(PAYLOAD_FMT)  # 55 bytes

# Type mask: ignore pos(0,1,2), accel(6,7,8), force(9), yaw(10); use vel(3,4,5), yaw_rate(11)
VELOCITY_YAW_RATE_TYPE_MASK = (
    (1 << 0)
    | (1 << 1)
    | (1 << 2)  # ignore x,y,z position
    | (1 << 6)
    | (1 << 7)
    | (1 << 8)  # ignore x,y,z acceleration
    | (1 << 9)  # ignore force
    | (1 << 10)  # ignore yaw
)

# COMMAND_LONG (msg 76)
COMMAND_LONG_MSG_ID = 76
COMMAND_LONG_CRC_EXTRA = 155  # 0x9B
COMMAND_LONG_FMT = "<BBHB7f"  # 33 bytes

MAV_CMD_COMPONENT_ARM_DISARM = 519
MAV_CMD_DO_SET_MODE = 176

_seq_counter = 0


def send_command_long(
    command: int,
    param1: float = 0.0,
    param2: float = 0.0,
    param3: float = 0.0,
    param4: float = 0.0,
    param5: float = 0.0,
    param6: float = 0.0,
    param7: float = 0.0,
    target_system: int = 1,
    target_component: int = 1,
    target_address: str = "127.0.0.1",
    port: int = 14540,
    sock: socket.socket | None = None,
) -> None:
    """Encode and send a MAVLink COMMAND_LONG message."""
    global _seq_counter

    payload = struct.pack(
        COMMAND_LONG_FMT,
        target_system,
        target_component,
        command,
        0,  # confirmation
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7,
    )

    header = struct.pack(
        "<BBBBBBBHB",
        MAVLINK_STX_V2,
        len(payload),
        0,
        0,
        _seq_counter & 0xFF,
        1,
        1,
        COMMAND_LONG_MSG_ID & 0xFFFF,
        (COMMAND_LONG_MSG_ID >> 16) & 0xFF,
    )
    _seq_counter = (_seq_counter + 1) & 0xFF

    crc = mavlink_crc(header[1:] + payload, COMMAND_LONG_CRC_EXTRA)
    frame = header + payload + struct.pack("<H", crc)

    own_sock = sock is None
    if own_sock:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(frame, (target_address, port))
    finally:
        if own_sock:
            sock.close()


def arm_drone(
    target_address: str = "127.0.0.1",
    port: int = 14540,
    sock: socket.socket | None = None,
) -> None:
    """Send MAV_CMD_COMPONENT_ARM_DISARM with force arm (param2=21196 for SITL)."""
    send_command_long(
        command=MAV_CMD_COMPONENT_ARM_DISARM,
        param1=1.0,
        param2=21196.0,
        target_address=target_address,
        port=port,
        sock=sock,
    )


def set_offboard_mode(
    target_address: str = "127.0.0.1",
    port: int = 14540,
    sock: socket.socket | None = None,
) -> None:
    """Send MAV_CMD_DO_SET_MODE to switch to PX4 OFFBOARD mode (custom_mode=6)."""
    send_command_long(
        command=MAV_CMD_DO_SET_MODE,
        param1=1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        param2=6.0,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        target_address=target_address,
        port=port,
        sock=sock,
    )


def send_velocity(
    vx: float,
    vy: float,
    vz: float,
    yaw_rate: float = 0.0,
    target_system: int = 1,
    target_component: int = 1,
    target_address: str = "127.0.0.1",
    port: int = 14540,
    sock: socket.socket | None = None,
) -> None:
    """Encode and send a MAVLink SET_POSITION_TARGET_LOCAL_NED velocity command."""
    global _seq_counter

    payload = struct.pack(
        PAYLOAD_FMT,
        0,  # time_boot_ms (not used by most simulators)
        target_system,
        target_component,
        MAV_FRAME_LOCAL_NED,
        VELOCITY_YAW_RATE_TYPE_MASK,
        0.0,
        0.0,
        0.0,  # x, y, z position (ignored)
        vx,
        vy,
        vz,
        0.0,
        0.0,
        0.0,  # afx, afy, afz (ignored)
        0.0,  # yaw (ignored)
        yaw_rate,
    )

    header = struct.pack(
        "<BBBBBBBHB",
        MAVLINK_STX_V2,
        len(payload),
        0,  # incompat_flags
        0,  # compat_flags
        _seq_counter & 0xFF,
        1,  # sysid
        1,  # compid
        SET_POSITION_TARGET_LOCAL_NED_MSG_ID & 0xFFFF,
        (SET_POSITION_TARGET_LOCAL_NED_MSG_ID >> 16) & 0xFF,
    )
    _seq_counter = (_seq_counter + 1) & 0xFF

    crc = mavlink_crc(header[1:] + payload, SET_POSITION_TARGET_LOCAL_NED_CRC_EXTRA)
    frame = header + payload + struct.pack("<H", crc)

    own_sock = sock is None
    if own_sock:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(frame, (target_address, port))
    finally:
        if own_sock:
            sock.close()
