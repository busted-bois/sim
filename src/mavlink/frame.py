"""Lightweight MAVLink v1/v2 frame parsing (no pymavlink)."""

from __future__ import annotations

from typing import NamedTuple

MAVLINK_V1_STX = 0xFE
MAVLINK_V2_STX = 0xFD


class Frame(NamedTuple):
    version: int
    sysid: int
    compid: int
    msgid: int
    payload_len: int


def parse_mavlink(buf: bytes) -> Frame | None:
    if not buf:
        return None
    stx = buf[0]
    if stx == MAVLINK_V1_STX and len(buf) >= 8:
        plen = buf[1]
        if len(buf) >= 6 + plen + 2:
            return Frame(1, buf[3], buf[4], buf[5], plen)
    if stx == MAVLINK_V2_STX and len(buf) >= 12:
        plen = buf[1]
        if len(buf) >= 10 + plen + 2:
            msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            return Frame(2, buf[5], buf[6], msgid, plen)
    return None


def frame_payload(buf: bytes, frame: Frame) -> bytes | None:
    if frame.version == 1:
        start = 6
    else:
        start = 10
    end = start + frame.payload_len
    if len(buf) < end:
        return None
    return buf[start:end]
