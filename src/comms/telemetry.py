"""Threaded UDP MAVLink telemetry listener."""

import copy
import socket
import threading
import time
from dataclasses import dataclass, field

from src.comms.mavlink_parser import (
    Attitude,
    Heartbeat,
    HighresImu,
    Odometry,
    decode,
)
from src.comms.state import DroneState


@dataclass
class HeartbeatStats:
    received: int = 0
    missed: int = 0
    max_gap_s: float = 0.0
    gaps: list = field(default_factory=list)  # list of (timestamp, gap_seconds) tuples


class TelemetryClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 14550, timeout: float = 1.0):
        self._host = host
        self._port = port
        self._timeout = timeout

        self._state = DroneState()
        self._lock = threading.Lock()

        self._running = False
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None

        self._last_heartbeat_time: float | None = None
        self._prev_heartbeat_time: float | None = None
        self._heartbeat_stats = HeartbeatStats()

        self._packets_received = 0
        self._packets_failed = 0
        self._packets_by_type: dict = {}

    def start(self) -> None:
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.settimeout(0.1)
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)

    @property
    def is_connected(self) -> bool:
        with self._lock:
            if self._last_heartbeat_time is None:
                return False
            return time.monotonic() - self._last_heartbeat_time < self._timeout

    def get_state(self) -> DroneState:
        with self._lock:
            return copy.deepcopy(self._state)

    def get_heartbeat_stats(self) -> HeartbeatStats:
        with self._lock:
            return HeartbeatStats(
                received=self._heartbeat_stats.received,
                missed=self._heartbeat_stats.missed,
                max_gap_s=self._heartbeat_stats.max_gap_s,
                gaps=list(self._heartbeat_stats.gaps),
            )

    def get_packet_stats(self) -> dict:
        with self._lock:
            return {
                "received": self._packets_received,
                "failed_decode": self._packets_failed,
                "by_type": dict(self._packets_by_type),
            }

    def _listen(self) -> None:
        assert self._sock is not None
        while self._running:
            try:
                data, _ = self._sock.recvfrom(4096)
            except (TimeoutError, OSError):
                continue

            msg = decode(data)

            with self._lock:
                if msg is None:
                    self._packets_failed += 1
                    continue

                self._packets_received += 1
                type_name = type(msg).__name__
                self._packets_by_type[type_name] = self._packets_by_type.get(type_name, 0) + 1

                if isinstance(msg, Heartbeat):
                    self._handle_heartbeat(msg)
                elif isinstance(msg, Attitude):
                    self._state.update(
                        roll=msg.roll,
                        pitch=msg.pitch,
                        yaw=msg.yaw,
                        timestamp_ms=msg.time_boot_ms,
                    )
                elif isinstance(msg, Odometry):
                    self._state.update(
                        x=msg.x,
                        y=msg.y,
                        z=msg.z,
                        vx=msg.vx,
                        vy=msg.vy,
                        vz=msg.vz,
                        qw=msg.q[0],
                        timestamp_ms=msg.time_usec // 1000,
                    )
                elif isinstance(msg, HighresImu):
                    self._state.update(
                        xacc=msg.xacc,
                        yacc=msg.yacc,
                        zacc=msg.zacc,
                        timestamp_ms=msg.time_usec // 1000,
                    )

    def _handle_heartbeat(self, msg: Heartbeat) -> None:
        now = time.monotonic()

        if self._prev_heartbeat_time is not None:
            gap = now - self._prev_heartbeat_time
            if gap > 1.5 * self._timeout:
                self._heartbeat_stats.missed += 1
                self._heartbeat_stats.gaps.append((now, gap))
                if gap > self._heartbeat_stats.max_gap_s:
                    self._heartbeat_stats.max_gap_s = gap

        self._prev_heartbeat_time = self._last_heartbeat_time
        self._last_heartbeat_time = now
        self._heartbeat_stats.received += 1

        self._state.update(
            armed=bool(msg.base_mode & 128),
            system_status=msg.system_status,
            timestamp_ms=int(now * 1000),
        )
