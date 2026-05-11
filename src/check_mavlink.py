"""Probe whether the running simulator emits MAVLink over UDP.

Passive listener: start the simulator first, then run `uv run check-mavlink`.
Sniffs MAVLink v1 (0xFE) and v2 (0xFD) frames on standard ports (14540, 14550),
logs each packet's source/sysid/msgid, and exits with PASS/FAIL.

Sends nothing. Does not change AirSim settings.
"""

from __future__ import annotations

import argparse
import select
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import NamedTuple

DEFAULT_PORTS: tuple[int, ...] = (14540, 14550)
DEFAULT_DURATION_S: float = 10.0
RECV_BUFSIZE = 4096
MAVLINK_V1_STX = 0xFE
MAVLINK_V2_STX = 0xFD

# Spec section 4.3 messages plus the common PX4 SITL ones, so output is readable
# without pulling in pymavlink as a dep.
MSGID_NAMES = {
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


class Frame(NamedTuple):
    version: int
    sysid: int
    compid: int
    msgid: int
    payload_len: int


@dataclass
class PortStats:
    port: int
    sock: socket.socket
    bytes_total: int = 0
    pkts_total: int = 0
    pkts_v1: int = 0
    pkts_v2: int = 0
    pkts_other: int = 0
    sysids: set[int] = field(default_factory=set)
    msgids: set[int] = field(default_factory=set)
    sources: set[tuple[str, int]] = field(default_factory=set)
    first_pkt_t: float | None = None
    last_pkt_t: float | None = None


def parse_mavlink(buf: bytes) -> Frame | None:
    if not buf:
        return None
    stx = buf[0]
    # MAVLink v1 header: STX, LEN, SEQ, SYSID, COMPID, MSGID, payload, CK1, CK2
    if stx == MAVLINK_V1_STX and len(buf) >= 8:
        plen = buf[1]
        if len(buf) >= 6 + plen + 2:
            return Frame(1, buf[3], buf[4], buf[5], plen)
    # MAVLink v2 header: STX, LEN, INCOMPAT, COMPAT, SEQ, SYSID, COMPID, MSGID(3B LE)
    if stx == MAVLINK_V2_STX and len(buf) >= 12:
        plen = buf[1]
        if len(buf) >= 10 + plen + 2:
            msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            return Frame(2, buf[5], buf[6], msgid, plen)
    return None


def make_listener(port: int) -> socket.socket | None:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        s.setblocking(False)
        return s
    except OSError as e:
        print(f"[check-mavlink] WARN cannot bind UDP :{port} ({e}); skipping.")
        return None


def fmt_msgid(msgid: int) -> str:
    return f"{msgid}({MSGID_NAMES.get(msgid, '?')})"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Probe simulator for MAVLink UDP output.")
    p.add_argument(
        "--port", type=int, action="append", default=[],
        help="Extra UDP port to listen on (repeatable).",
    )
    p.add_argument(
        "--duration", type=float, default=DEFAULT_DURATION_S,
        help=f"Listen seconds (default {DEFAULT_DURATION_S}).",
    )
    p.add_argument("--quiet", action="store_true", help="Suppress per-packet log.")
    args = p.parse_args(argv)

    ports = list(dict.fromkeys([*DEFAULT_PORTS, *args.port]))
    print(f"[check-mavlink] probing UDP ports {ports} for {args.duration:.1f}s")
    print("[check-mavlink] passive listener -- start the simulator first; sends nothing")

    stats: list[PortStats] = []
    for port in ports:
        sock = make_listener(port)
        if sock is not None:
            stats.append(PortStats(port=port, sock=sock))

    if not stats:
        print("[check-mavlink] FAIL: could not bind any UDP port.")
        return 2

    socks = [st.sock for st in stats]
    by_fd = {st.sock.fileno(): st for st in stats}
    deadline = time.monotonic() + args.duration
    last_tick = time.monotonic()

    interrupted = False
    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        timeout = min(0.5, deadline - now)
        try:
            ready, _, _ = select.select(socks, [], [], timeout)
        except KeyboardInterrupt:
            interrupted = True
            print("\n[check-mavlink] interrupted -- emitting summary so far.")
            break
        for s in ready:
            st = by_fd[s.fileno()]
            try:
                buf, addr = s.recvfrom(RECV_BUFSIZE)
            except OSError:
                continue
            t = time.monotonic()
            st.bytes_total += len(buf)
            st.pkts_total += 1
            st.sources.add(addr)
            if st.first_pkt_t is None:
                st.first_pkt_t = t
            st.last_pkt_t = t
            frame = parse_mavlink(buf)
            if frame is None:
                st.pkts_other += 1
                if not args.quiet:
                    head = buf[:8].hex(" ")
                    print(
                        f"[:{st.port}] non-mavlink {len(buf)}B "
                        f"from {addr[0]}:{addr[1]} head={head}"
                    )
                continue
            if frame.version == 1:
                st.pkts_v1 += 1
            else:
                st.pkts_v2 += 1
            st.sysids.add(frame.sysid)
            st.msgids.add(frame.msgid)
            if not args.quiet:
                print(
                    f"[:{st.port}] mavlink v{frame.version} {len(buf)}B "
                    f"from {addr[0]}:{addr[1]} sys={frame.sysid} comp={frame.compid} "
                    f"msg={fmt_msgid(frame.msgid)} payload={frame.payload_len}B"
                )
        if now - last_tick >= 1.0:
            last_tick = now
            tot = sum(st.pkts_total for st in stats)
            mav = sum(st.pkts_v1 + st.pkts_v2 for st in stats)
            remaining = max(0.0, deadline - now)
            print(f"[check-mavlink] tick: {tot} pkts, {mav} mavlink, {remaining:.1f}s left")

    print("\n[check-mavlink] === summary ===")
    saw_mavlink = False
    for st in stats:
        if st.first_pkt_t and st.last_pkt_t and st.last_pkt_t > st.first_pkt_t:
            rate = st.pkts_total / (st.last_pkt_t - st.first_pkt_t)
        else:
            rate = 0.0
        srcs = ", ".join(f"{ip}:{p}" for ip, p in sorted(st.sources)) or "(none)"
        msgid_strs = [fmt_msgid(m) for m in sorted(st.msgids)] or ["[]"]
        print(
            f"  :{st.port:<5}  pkts={st.pkts_total:<4} "
            f"v1={st.pkts_v1} v2={st.pkts_v2} other={st.pkts_other} "
            f"sysids={sorted(st.sysids) or '[]'} "
            f"msgids={msgid_strs} "
            f"rate={rate:.1f}Hz src={srcs}"
        )
        if st.pkts_v1 + st.pkts_v2 > 0:
            saw_mavlink = True

    print()
    if saw_mavlink:
        print("[check-mavlink] PASS: simulator is emitting MAVLink over UDP.")
        return 130 if interrupted else 0

    print("[check-mavlink] FAIL: no MAVLink frames observed.")
    print("  Common causes:")
    print(
        "    - AirSim settings.json has VehicleType=SimpleFlight (RPC-only, no MAVLink). "
        "Run `uv run sim-mavlink` to switch to PX4Multirotor."
    )
    print(
        "    - PX4-SITL is running with the SIH simulator (look for 'INFO [init] SIH "
        "simulator' in PX4 log). Relaunch PX4 with PX4_SIM_MODEL=iris."
    )
    print(
        "    - PX4 connected to AirSim but settings.json is missing QgcHostIp/QgcPort, "
        "so AirSim has no GCS endpoint to forward MAVLink to."
    )
    print(
        "    - Windows Defender Firewall is dropping the inbound UDP from AirSim "
        "(less likely on 127.0.0.1 loopback, but possible)."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
