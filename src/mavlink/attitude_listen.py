"""Listen for MAVLink ATTITUDE on UDP without pymavlink."""

from __future__ import annotations

import argparse
import select
import socket
import sys
import time
from dataclasses import replace

from src.mavlink.attitude import decode_attitude_payload, format_attitude_sample, roll_pitch_yaw_deg
from src.mavlink.frame import frame_payload, parse_mavlink
from src.mavlink.messages import MAVLINK_MSG_ID_ATTITUDE
from src.mavlink.udp import DEFAULT_PORTS, RECV_BUFSIZE, make_listener

DEFAULT_DURATION_S = 10.0
_LOG = "attitude-listen"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Listen for MAVLink ATTITUDE on UDP.")
    p.add_argument("--port", type=int, action="append", default=[], help="Extra UDP port.")
    p.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"Listen seconds (default {DEFAULT_DURATION_S}).",
    )
    args = p.parse_args(argv)

    ports = list(dict.fromkeys([*DEFAULT_PORTS, *args.port]))
    socks: list[socket.socket] = []
    for port in ports:
        sock = make_listener(port, log_prefix=_LOG)
        if sock is not None:
            socks.append(sock)

    if not socks:
        print(f"[{_LOG}] FAIL: could not bind any UDP port.")
        return 2

    print(f"[{_LOG}] listening on {ports} for {args.duration:.1f}s")
    deadline = time.monotonic() + args.duration
    last_tick = time.monotonic()
    attitude_count = 0
    latest = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        timeout = min(0.5, deadline - now)
        ready, _, _ = select.select(socks, [], [], timeout)
        for s in ready:
            try:
                buf, addr = s.recvfrom(RECV_BUFSIZE)
            except OSError:
                continue
            frame = parse_mavlink(buf)
            if frame is None or frame.msgid != MAVLINK_MSG_ID_ATTITUDE:
                continue
            payload = frame_payload(buf, frame)
            if payload is None:
                continue
            decoded = decode_attitude_payload(payload)
            if decoded is None:
                continue
            sample = replace(
                decoded,
                local_received_monotonic_ns=time.monotonic_ns(),
                transport=f"udp:{addr[0]}:{addr[1]}",
            )
            attitude_count += 1
            latest = sample

        if now - last_tick >= 1.0:
            last_tick = now
            if latest is not None:
                roll_d, pitch_d, yaw_d = roll_pitch_yaw_deg(latest)
                print(
                    f"[{_LOG}] tick: {attitude_count} ATTITUDE samples "
                    f"latest rpy_deg=({roll_d:.2f},{pitch_d:.2f},{yaw_d:.2f})"
                )
            else:
                print(f"[{_LOG}] tick: {attitude_count} ATTITUDE samples")

    print(f"\n[{_LOG}] === summary === ATTITUDE samples={attitude_count}")
    if latest is not None:
        print(f"[{_LOG}] latest: {format_attitude_sample(latest)}")
    if attitude_count > 0:
        print(f"[{_LOG}] PASS: received MAVLink ATTITUDE.")
        return 0
    print(f"[{_LOG}] FAIL: no ATTITUDE frames decoded.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
