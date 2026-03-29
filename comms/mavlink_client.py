"""
MAVLink UDP client — receives telemetry and exposes a clean API.

Runs a background receiver thread that continuously reads MAVLink frames
from UDP and updates shared drone state. The algo team consumes state via:

    client = TelemetryClient()
    client.start()
    state = client.get_state()
    # { position, velocity, attitude, angular_velocity, timestamp_ms, ... }
"""

import math
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from comms.mavlink_parser import (
    Attitude,
    Heartbeat,
    HighresImu,
    MsgId,
    Odometry,
    Timesync,
    decode,
    MAVLINK_HEADER_LEN,
    MAVLINK_CHECKSUM_LEN,
    MAVLINK_STX_V2,
)


@dataclass
class DroneState:
    """Snapshot of the drone's current state — the interface algo team consumes."""
    # Position (NED frame, meters)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # Velocity (NED frame, m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Attitude (radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular velocity (rad/s)
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0

    # Quaternion orientation (w, x, y, z)
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0

    # IMU
    xacc: float = 0.0
    yacc: float = 0.0
    zacc: float = 0.0

    # Timestamps
    timestamp_ms: int = 0        # boot time in ms (from ATTITUDE)
    timestamp_us: int = 0        # boot time in us (from ODOMETRY/IMU)
    last_update: float = 0.0     # wall clock (time.monotonic)

    # Heartbeat info
    armed: bool = False
    system_status: int = 0
    last_heartbeat: float = 0.0  # wall clock


@dataclass
class HeartbeatStats:
    """Tracks heartbeat reliability metrics."""
    received: int = 0
    expected: int = 0
    missed: int = 0
    max_gap_s: float = 0.0
    gaps: list = field(default_factory=list)  # list of (timestamp, gap_duration)


class TelemetryClient:
    """
    UDP MAVLink telemetry receiver with a clean state API.

    Usage:
        client = TelemetryClient(port=14550)
        client.start()
        state = client.get_state()
        client.stop()
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 14550, timeout: float = 2.0):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._state = DroneState()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None

        # Stats
        self._heartbeat_stats = HeartbeatStats()
        self._packets_received = 0
        self._packets_failed = 0
        self._msg_counts: dict[int, int] = {}

    def start(self):
        """Start receiving telemetry in a background thread."""
        if self._running:
            return

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.settimeout(self._timeout)

        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the receiver thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._sock:
            self._sock.close()
            self._sock = None

    def get_state(self) -> DroneState:
        """
        Get the latest drone state snapshot.

        This is the primary API for the algo team.
        Returns a copy so callers can read without holding the lock.
        """
        with self._lock:
            # Return a shallow copy
            s = self._state
            return DroneState(
                x=s.x, y=s.y, z=s.z,
                vx=s.vx, vy=s.vy, vz=s.vz,
                roll=s.roll, pitch=s.pitch, yaw=s.yaw,
                rollspeed=s.rollspeed, pitchspeed=s.pitchspeed, yawspeed=s.yawspeed,
                qw=s.qw, qx=s.qx, qy=s.qy, qz=s.qz,
                xacc=s.xacc, yacc=s.yacc, zacc=s.zacc,
                timestamp_ms=s.timestamp_ms, timestamp_us=s.timestamp_us,
                last_update=s.last_update,
                armed=s.armed, system_status=s.system_status,
                last_heartbeat=s.last_heartbeat,
            )

    def get_heartbeat_stats(self) -> HeartbeatStats:
        """Get heartbeat reliability stats."""
        with self._lock:
            return HeartbeatStats(
                received=self._heartbeat_stats.received,
                expected=self._heartbeat_stats.expected,
                missed=self._heartbeat_stats.missed,
                max_gap_s=self._heartbeat_stats.max_gap_s,
                gaps=list(self._heartbeat_stats.gaps),
            )

    def get_packet_stats(self) -> dict:
        """Get packet reception statistics."""
        with self._lock:
            return {
                "received": self._packets_received,
                "failed_decode": self._packets_failed,
                "by_type": dict(self._msg_counts),
            }

    @property
    def is_connected(self) -> bool:
        """True if we've received a heartbeat in the last 3 seconds."""
        with self._lock:
            if self._state.last_heartbeat == 0:
                return False
            return (time.monotonic() - self._state.last_heartbeat) < 3.0

    def _receive_loop(self):
        """Background loop: read UDP datagrams and decode MAVLink messages."""
        buf = bytearray()

        while self._running:
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            # Process each received datagram (mock sim sends one message per packet)
            msg = decode(data)
            if msg is None:
                with self._lock:
                    self._packets_failed += 1
                continue

            with self._lock:
                self._packets_received += 1
                self._msg_counts[msg.MSG_ID] = self._msg_counts.get(msg.MSG_ID, 0) + 1
                self._apply_message(msg)

    def _apply_message(self, msg):
        """Apply a decoded message to the drone state. Must hold self._lock."""
        now = time.monotonic()

        if isinstance(msg, Heartbeat):
            prev = self._state.last_heartbeat
            self._state.last_heartbeat = now
            self._state.armed = bool(msg.base_mode & 128)  # MAV_MODE_FLAG_SAFETY_ARMED
            self._state.system_status = msg.system_status

            # Track heartbeat gaps
            self._heartbeat_stats.received += 1
            if prev > 0:
                gap = now - prev
                if gap > 1.5:  # expected 1 Hz, flag gaps > 1.5s
                    self._heartbeat_stats.missed += 1
                    self._heartbeat_stats.gaps.append((now, gap))
                if gap > self._heartbeat_stats.max_gap_s:
                    self._heartbeat_stats.max_gap_s = gap

        elif isinstance(msg, Attitude):
            self._state.roll = msg.roll
            self._state.pitch = msg.pitch
            self._state.yaw = msg.yaw
            self._state.rollspeed = msg.rollspeed
            self._state.pitchspeed = msg.pitchspeed
            self._state.yawspeed = msg.yawspeed
            self._state.timestamp_ms = msg.time_boot_ms
            self._state.last_update = now

        elif isinstance(msg, Odometry):
            self._state.x = msg.x
            self._state.y = msg.y
            self._state.z = msg.z
            self._state.vx = msg.vx
            self._state.vy = msg.vy
            self._state.vz = msg.vz
            self._state.qw = msg.q[0]
            self._state.qx = msg.q[1]
            self._state.qy = msg.q[2]
            self._state.qz = msg.q[3]
            self._state.timestamp_us = msg.time_usec
            self._state.last_update = now

        elif isinstance(msg, HighresImu):
            self._state.xacc = msg.xacc
            self._state.yacc = msg.yacc
            self._state.zacc = msg.zacc
            self._state.timestamp_us = msg.time_usec
            self._state.last_update = now

        elif isinstance(msg, Timesync):
            pass  # Could implement clock sync here
