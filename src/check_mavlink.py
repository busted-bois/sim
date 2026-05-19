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

from src.mavlink.attitude import decode_attitude_payload, roll_pitch_yaw_deg
from src.mavlink.frame import frame_payload, parse_mavlink
from src.mavlink.messages import MAVLINK_MSG_ID_ATTITUDE, fmt_msgid
from src.mavlink.udp import DEFAULT_PORTS, RECV_BUFSIZE, make_listener

DEFAULT_DURATION_S: float = 10.0
_LOG = "check-mavlink"


@dataclass
class PortStats:
    port: int
    sock: socket.socket
    bytes_total: int = 0
    pkts_total: int = 0
    pkts_v1: int = 0
    pkts_v2: int = 0
    pkts_other: int = 0
    attitude_decoded: int = 0
    sysids: set[int] = field(default_factory=set)
    msgids: set[int] = field(default_factory=set)
    sources: set[tuple[str, int]] = field(default_factory=set)
    first_pkt_t: float | None = None
    last_pkt_t: float | None = None


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Probe simulator for MAVLink UDP output.")
    p.add_argument("--port", type=int, action="append", default=[], help="Extra UDP port.")
    p.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="Listen seconds (0=forever).",
    )
    p.add_argument("--quiet", action="store_true", help="Suppress per-packet log.")
    p.add_argument("--decode-attitude", action="store_true", help="Decode ATTITUDE (#30) payloads.")
    args = p.parse_args(argv)

    forever = args.duration <= 0
    ports = list(dict.fromkeys([*DEFAULT_PORTS, *args.port]))
    if forever:
        print(f"[{_LOG}] probing UDP ports {ports} forever (Ctrl+C to stop)")
    else:
        print(f"[{_LOG}] probing UDP ports {ports} for {args.duration:.1f}s")
    print(f"[{_LOG}] passive listener -- start the simulator first; sends nothing")
    if args.decode_attitude:
        print(f"[{_LOG}] ATTITUDE decode enabled")

    stats: list[PortStats] = []
    for port in ports:
        sock = make_listener(port, log_prefix=_LOG)
        if sock is not None:
            stats.append(PortStats(port=port, sock=sock))

    if not stats:
        print(f"[{_LOG}] FAIL: could not bind any UDP port.")
        return 2

    socks = [st.sock for st in stats]
    by_fd = {st.sock.fileno(): st for st in stats}
    deadline = None if forever else time.monotonic() + args.duration
    last_tick = time.monotonic()
    total_attitude = 0
    interrupted = False

    while True:
        now = time.monotonic()
        if deadline is not None and now >= deadline:
            break
        timeout = 0.5 if deadline is None else min(0.5, deadline - now)
        try:
            ready, _, _ = select.select(socks, [], [], timeout)
        except KeyboardInterrupt:
            interrupted = True
            print(f"\n[{_LOG}] interrupted -- emitting summary so far.")
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
            attitude_extra = ""
            if args.decode_attitude and frame.msgid == MAVLINK_MSG_ID_ATTITUDE:
                payload = frame_payload(buf, frame)
                if payload is not None:
                    sample = decode_attitude_payload(payload)
                    if sample is not None:
                        st.attitude_decoded += 1
                        total_attitude += 1
                        roll_d, pitch_d, yaw_d = roll_pitch_yaw_deg(sample)
                        attitude_extra = (
                            f" rpy_deg=({roll_d:.2f},{pitch_d:.2f},{yaw_d:.2f}) "
                            f"rates=({sample.rollspeed:.3f},{sample.pitchspeed:.3f},{sample.yawspeed:.3f})"
                        )
            if not args.quiet:
                print(
                    f"[:{st.port}] mavlink v{frame.version} {len(buf)}B from {addr[0]}:{addr[1]} "
                    f"sys={frame.sysid} comp={frame.compid} msg={fmt_msgid(frame.msgid)} "
                    f"payload={frame.payload_len}B{attitude_extra}"
                )
        if now - last_tick >= 1.0:
            last_tick = now
            tot = sum(st.pkts_total for st in stats)
            mav = sum(st.pkts_v1 + st.pkts_v2 for st in stats)
            att = sum(st.attitude_decoded for st in stats)
            if deadline is None:
                print(f"[{_LOG}] tick: {tot} pkts, {mav} mavlink, {att} ATTITUDE decoded")
            else:
                remaining = max(0.0, deadline - now)
                print(
                    f"[{_LOG}] tick: {tot} pkts, {mav} mavlink, "
                    f"{att} ATTITUDE decoded, {remaining:.1f}s left"
                )

    print(f"\n[{_LOG}] === summary ===")
    saw_mavlink = False
    for st in stats:
        if st.first_pkt_t and st.last_pkt_t and st.last_pkt_t > st.first_pkt_t:
            rate = st.pkts_total / (st.last_pkt_t - st.first_pkt_t)
        else:
            rate = 0.0
        srcs = ", ".join(f"{ip}:{p}" for ip, p in sorted(st.sources)) or "(none)"
        msgid_strs = [fmt_msgid(m) for m in sorted(st.msgids)] or ["[]"]
        print(
            f"  :{st.port:<5} pkts={st.pkts_total:<4} "
            f"v1={st.pkts_v1} v2={st.pkts_v2} other={st.pkts_other} "
            f"sysids={sorted(st.sysids) or '[]'} msgids={msgid_strs} rate={rate:.1f}Hz src={srcs}"
        )
        if args.decode_attitude:
            print(f"           ATTITUDE decoded={st.attitude_decoded}")
        if st.pkts_v1 + st.pkts_v2 > 0:
            saw_mavlink = True
    if args.decode_attitude:
        print(f"  ATTITUDE total decoded: {total_attitude}")
    print()
    if saw_mavlink:
        print(f"[{_LOG}] PASS: simulator is emitting MAVLink over UDP.")
        return 130 if interrupted else 0
    print(f"[{_LOG}] FAIL: no MAVLink frames observed.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
