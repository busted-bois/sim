"""
MAVLink message parser for AIGP drone challenge.

Encodes and decodes the following MAVLink message types:
- HEARTBEAT (msgid 0)
- ATTITUDE (msgid 30)
- HIGHRES_IMU (msgid 105)
- ODOMETRY (msgid 331)
- TIMESYNC (msgid 111)

Uses raw struct packing to avoid external MAVLink library dependencies for parsing.
Wire format: MAVLink v2 compatible payload layout.
"""

import struct
from dataclasses import dataclass
from enum import IntEnum

# --- MAVLink v2 constants ---
MAVLINK_STX_V2 = 0xFD
MAVLINK_HEADER_LEN = 10  # v2 header
MAVLINK_CHECKSUM_LEN = 2


class MsgId(IntEnum):
    HEARTBEAT = 0
    ATTITUDE = 30
    HIGHRES_IMU = 105
    TIMESYNC = 111
    ODOMETRY = 331


# CRC_EXTRA seeds per message type (from MAVLink spec)
CRC_EXTRA = {
    MsgId.HEARTBEAT: 50,
    MsgId.ATTITUDE: 39,
    MsgId.HIGHRES_IMU: 93,
    MsgId.TIMESYNC: 34,
    MsgId.ODOMETRY: 91,
}


# --- Data classes for each message ---


@dataclass
class Heartbeat:
    """MAVLink HEARTBEAT message."""

    custom_mode: int = 0  # uint32
    type: int = 0  # uint8  (MAV_TYPE)
    autopilot: int = 0  # uint8  (MAV_AUTOPILOT)
    base_mode: int = 0  # uint8
    system_status: int = 0  # uint8  (MAV_STATE)
    mavlink_version: int = 3  # uint8

    MSG_ID = MsgId.HEARTBEAT
    PAYLOAD_FMT = "<IBBBBB"
    PAYLOAD_LEN = struct.calcsize("<IBBBBB")  # 9 bytes


@dataclass
class Attitude:
    """MAVLink ATTITUDE message."""

    time_boot_ms: int = 0  # uint32
    roll: float = 0.0  # float rad
    pitch: float = 0.0  # float rad
    yaw: float = 0.0  # float rad
    rollspeed: float = 0.0  # float rad/s
    pitchspeed: float = 0.0  # float rad/s
    yawspeed: float = 0.0  # float rad/s

    MSG_ID = MsgId.ATTITUDE
    PAYLOAD_FMT = "<Iffffff"
    PAYLOAD_LEN = struct.calcsize("<Iffffff")  # 28 bytes


@dataclass
class HighresImu:
    """MAVLink HIGHRES_IMU message."""

    time_usec: int = 0  # uint64
    xacc: float = 0.0  # float m/s^2
    yacc: float = 0.0
    zacc: float = 0.0
    xgyro: float = 0.0  # float rad/s
    ygyro: float = 0.0
    zgyro: float = 0.0
    xmag: float = 0.0  # float gauss
    ymag: float = 0.0
    zmag: float = 0.0
    abs_pressure: float = 0.0  # float millibar
    diff_pressure: float = 0.0  # float millibar
    pressure_alt: float = 0.0  # float m
    temperature: float = 0.0  # float degC
    fields_updated: int = 0  # uint16 bitmask

    MSG_ID = MsgId.HIGHRES_IMU
    PAYLOAD_FMT = "<QfffffffffffffH"
    PAYLOAD_LEN = struct.calcsize("<QfffffffffffffH")  # 62 bytes


@dataclass
class Timesync:
    """MAVLink TIMESYNC message."""

    tc1: int = 0  # int64 ns
    ts1: int = 0  # int64 ns

    MSG_ID = MsgId.TIMESYNC
    PAYLOAD_FMT = "<qq"
    PAYLOAD_LEN = struct.calcsize("<qq")  # 16 bytes


@dataclass
class Odometry:
    """MAVLink ODOMETRY message (simplified core fields)."""

    time_usec: int = 0  # uint64
    frame_id: int = 0  # uint8  MAV_FRAME
    child_frame_id: int = 0  # uint8  MAV_FRAME
    x: float = 0.0  # float m  (NED)
    y: float = 0.0
    z: float = 0.0
    q: tuple = (1.0, 0.0, 0.0, 0.0)  # quaternion (w, x, y, z)
    vx: float = 0.0  # float m/s
    vy: float = 0.0
    vz: float = 0.0
    rollspeed: float = 0.0  # float rad/s
    pitchspeed: float = 0.0
    yawspeed: float = 0.0

    MSG_ID = MsgId.ODOMETRY
    # Layout: uint64 + 3f(pos) + 4f(quat) + 3f(vel) + 3f(angvel) + ... + 2 uint8
    # Full MAVLink ODOMETRY has covariance arrays; we pack the core fields only.
    PAYLOAD_FMT = "<Q fff ffff fff fff BB"
    PAYLOAD_LEN = struct.calcsize("<QffffffffffffffBB")  # 58 bytes


# Mapping from message ID to dataclass
MSG_TYPES = {
    MsgId.HEARTBEAT: Heartbeat,
    MsgId.ATTITUDE: Attitude,
    MsgId.HIGHRES_IMU: HighresImu,
    MsgId.TIMESYNC: Timesync,
    MsgId.ODOMETRY: Odometry,
}


# --- CRC (X.25) ---


def _crc_accumulate(byte: int, crc: int) -> int:
    tmp = byte ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def mavlink_crc(data: bytes, extra: int) -> int:
    """Compute MAVLink X.25 CRC with CRC_EXTRA seed."""
    crc = 0xFFFF
    for b in data:
        crc = _crc_accumulate(b, crc)
    crc = _crc_accumulate(extra, crc)
    return crc


# --- Encoder ---


def _pack_payload(msg) -> bytes:
    """Pack a message dataclass into its wire payload bytes."""
    if isinstance(msg, Heartbeat):
        return struct.pack(
            Heartbeat.PAYLOAD_FMT,
            msg.custom_mode,
            msg.type,
            msg.autopilot,
            msg.base_mode,
            msg.system_status,
            msg.mavlink_version,
        )
    elif isinstance(msg, Attitude):
        return struct.pack(
            Attitude.PAYLOAD_FMT,
            msg.time_boot_ms,
            msg.roll,
            msg.pitch,
            msg.yaw,
            msg.rollspeed,
            msg.pitchspeed,
            msg.yawspeed,
        )
    elif isinstance(msg, HighresImu):
        return struct.pack(
            HighresImu.PAYLOAD_FMT.replace(" ", ""),
            msg.time_usec,
            msg.xacc,
            msg.yacc,
            msg.zacc,
            msg.xgyro,
            msg.ygyro,
            msg.zgyro,
            msg.xmag,
            msg.ymag,
            msg.zmag,
            msg.abs_pressure,
            msg.diff_pressure,
            msg.pressure_alt,
            msg.temperature,
            msg.fields_updated,
        )
    elif isinstance(msg, Timesync):
        return struct.pack(Timesync.PAYLOAD_FMT, msg.tc1, msg.ts1)
    elif isinstance(msg, Odometry):
        return struct.pack(
            Odometry.PAYLOAD_FMT.replace(" ", ""),
            msg.time_usec,
            msg.x,
            msg.y,
            msg.z,
            msg.q[0],
            msg.q[1],
            msg.q[2],
            msg.q[3],
            msg.vx,
            msg.vy,
            msg.vz,
            msg.rollspeed,
            msg.pitchspeed,
            msg.yawspeed,
            msg.frame_id,
            msg.child_frame_id,
        )
    else:
        raise ValueError(f"Unknown message type: {type(msg)}")


_seq_counter = 0


def encode(msg, system_id: int = 1, component_id: int = 1) -> bytes:
    """
    Encode a message dataclass into a full MAVLink v2 frame.

    Returns the complete byte sequence ready to send over UDP.
    """
    global _seq_counter

    payload = _pack_payload(msg)
    msg_id = msg.MSG_ID

    # MAVLink v2 header
    header = struct.pack(
        "<BBBBBBBHB",
        MAVLINK_STX_V2,  # STX
        len(payload),  # payload length
        0,  # incompat_flags
        0,  # compat_flags
        _seq_counter & 0xFF,  # sequence
        system_id,
        component_id,
        msg_id & 0xFFFF,  # msg_id low 16 bits
        (msg_id >> 16) & 0xFF,  # msg_id high 8 bits
    )

    _seq_counter = (_seq_counter + 1) & 0xFF

    # CRC covers header[1:] + payload
    crc = mavlink_crc(header[1:] + payload, CRC_EXTRA[msg_id])
    crc_bytes = struct.pack("<H", crc)

    return header + payload + crc_bytes


# --- Decoder ---


def decode(data: bytes) -> object | None:
    """
    Decode a MAVLink v2 frame from bytes.

    Returns the appropriate message dataclass, or None if invalid.
    """
    if len(data) < MAVLINK_HEADER_LEN + MAVLINK_CHECKSUM_LEN:
        return None

    if data[0] != MAVLINK_STX_V2:
        return None

    payload_len = data[1]
    expected_len = MAVLINK_HEADER_LEN + payload_len + MAVLINK_CHECKSUM_LEN
    if len(data) < expected_len:
        return None

    # Parse header
    (stx, plen, incompat, compat, seq, sys_id, comp_id, msg_id_low, msg_id_high) = struct.unpack(
        "<BBBBBBBHB", data[:MAVLINK_HEADER_LEN]
    )
    msg_id = msg_id_low | (msg_id_high << 16)

    if msg_id not in MSG_TYPES:
        return None

    # Verify CRC
    payload = data[MAVLINK_HEADER_LEN : MAVLINK_HEADER_LEN + payload_len]
    received_crc = struct.unpack(
        "<H", data[MAVLINK_HEADER_LEN + payload_len : MAVLINK_HEADER_LEN + payload_len + 2]
    )[0]
    computed_crc = mavlink_crc(data[1:MAVLINK_HEADER_LEN] + payload, CRC_EXTRA[msg_id])

    if received_crc != computed_crc:
        return None

    # Unpack payload into dataclass
    msg_id_enum = MsgId(msg_id)

    if msg_id_enum == MsgId.HEARTBEAT:
        vals = struct.unpack(Heartbeat.PAYLOAD_FMT, payload)
        return Heartbeat(
            custom_mode=vals[0],
            type=vals[1],
            autopilot=vals[2],
            base_mode=vals[3],
            system_status=vals[4],
            mavlink_version=vals[5],
        )
    elif msg_id_enum == MsgId.ATTITUDE:
        vals = struct.unpack(Attitude.PAYLOAD_FMT, payload)
        return Attitude(
            time_boot_ms=vals[0],
            roll=vals[1],
            pitch=vals[2],
            yaw=vals[3],
            rollspeed=vals[4],
            pitchspeed=vals[5],
            yawspeed=vals[6],
        )
    elif msg_id_enum == MsgId.HIGHRES_IMU:
        fmt = HighresImu.PAYLOAD_FMT.replace(" ", "")
        vals = struct.unpack(fmt, payload)
        return HighresImu(
            time_usec=vals[0],
            xacc=vals[1],
            yacc=vals[2],
            zacc=vals[3],
            xgyro=vals[4],
            ygyro=vals[5],
            zgyro=vals[6],
            xmag=vals[7],
            ymag=vals[8],
            zmag=vals[9],
            abs_pressure=vals[10],
            diff_pressure=vals[11],
            pressure_alt=vals[12],
            temperature=vals[13],
            fields_updated=vals[14],
        )
    elif msg_id_enum == MsgId.TIMESYNC:
        vals = struct.unpack(Timesync.PAYLOAD_FMT, payload)
        return Timesync(tc1=vals[0], ts1=vals[1])
    elif msg_id_enum == MsgId.ODOMETRY:
        fmt = Odometry.PAYLOAD_FMT.replace(" ", "")
        vals = struct.unpack(fmt, payload)
        return Odometry(
            time_usec=vals[0],
            x=vals[1],
            y=vals[2],
            z=vals[3],
            q=(vals[4], vals[5], vals[6], vals[7]),
            vx=vals[8],
            vy=vals[9],
            vz=vals[10],
            rollspeed=vals[11],
            pitchspeed=vals[12],
            yawspeed=vals[13],
            frame_id=vals[14],
            child_frame_id=vals[15],
        )

    return None
