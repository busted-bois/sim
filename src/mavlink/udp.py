"""UDP bind helpers for passive MAVLink sniffers."""

from __future__ import annotations

import socket

DEFAULT_PORTS: tuple[int, ...] = (14540, 14550)
RECV_BUFSIZE = 4096


def make_listener(port: int, *, log_prefix: str) -> socket.socket | None:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        s.setblocking(False)
        return s
    except OSError as e:
        print(f"[{log_prefix}] WARN cannot bind UDP :{port} ({e}); skipping.")
        return None
