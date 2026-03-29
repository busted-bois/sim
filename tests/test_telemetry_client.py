"""Tests for the TelemetryClient against the mock simulator (integration) and unit tests."""

import socket
import threading
import time

import pytest

from comms.mavlink_client import TelemetryClient, DroneState
from comms.mavlink_parser import (
    Attitude,
    Heartbeat,
    HighresImu,
    Odometry,
    Timesync,
    encode,
)


def _find_free_port() -> int:
    """Find a free UDP port for testing."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class TestDroneState:
    def test_defaults(self):
        state = DroneState()
        assert state.x == 0.0
        assert state.armed is False
        assert state.qw == 1.0

    def test_altitude_convention(self):
        """NED frame: negative z = above ground."""
        state = DroneState(z=-10.0)
        assert state.z < 0  # above ground in NED


class TestTelemetryClient:
    def test_receives_heartbeat(self):
        port = _find_free_port()
        client = TelemetryClient(host="127.0.0.1", port=port, timeout=1.0)
        client.start()

        # Send a heartbeat
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        hb = Heartbeat(type=2, autopilot=12, base_mode=209, system_status=4)
        sock.sendto(encode(hb), ("127.0.0.1", port))
        time.sleep(0.2)

        state = client.get_state()
        assert state.armed is True  # base_mode 209 has armed flag
        assert state.system_status == 4
        assert client.is_connected

        sock.close()
        client.stop()

    def test_receives_attitude(self):
        port = _find_free_port()
        client = TelemetryClient(host="127.0.0.1", port=port, timeout=1.0)
        client.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        att = Attitude(time_boot_ms=1000, roll=0.1, pitch=-0.2, yaw=1.5)
        sock.sendto(encode(att), ("127.0.0.1", port))
        time.sleep(0.2)

        state = client.get_state()
        assert state.roll == pytest.approx(0.1, abs=1e-5)
        assert state.pitch == pytest.approx(-0.2, abs=1e-5)
        assert state.yaw == pytest.approx(1.5, abs=1e-5)
        assert state.timestamp_ms == 1000

        sock.close()
        client.stop()

    def test_receives_odometry(self):
        port = _find_free_port()
        client = TelemetryClient(host="127.0.0.1", port=port, timeout=1.0)
        client.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        odom = Odometry(
            time_usec=500000, x=10.0, y=20.0, z=-5.0,
            vx=1.0, vy=2.0, vz=-0.5,
            q=(0.707, 0.0, 0.707, 0.0),
        )
        sock.sendto(encode(odom), ("127.0.0.1", port))
        time.sleep(0.2)

        state = client.get_state()
        assert state.x == pytest.approx(10.0, abs=1e-4)
        assert state.y == pytest.approx(20.0, abs=1e-4)
        assert state.z == pytest.approx(-5.0, abs=1e-4)
        assert state.vx == pytest.approx(1.0, abs=1e-4)

        sock.close()
        client.stop()

    def test_receives_imu(self):
        port = _find_free_port()
        client = TelemetryClient(host="127.0.0.1", port=port, timeout=1.0)
        client.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        imu = HighresImu(time_usec=100000, xacc=0.5, yacc=-1.0, zacc=-9.81)
        sock.sendto(encode(imu), ("127.0.0.1", port))
        time.sleep(0.2)

        state = client.get_state()
        assert state.xacc == pytest.approx(0.5, abs=1e-5)
        assert state.zacc == pytest.approx(-9.81, abs=1e-4)

        sock.close()
        client.stop()

    def test_packet_stats(self):
        port = _find_free_port()
        client = TelemetryClient(host="127.0.0.1", port=port, timeout=1.0)
        client.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for _ in range(5):
            sock.sendto(encode(Heartbeat()), ("127.0.0.1", port))
        # Send garbage too
        sock.sendto(b"\x00\x01\x02", ("127.0.0.1", port))
        time.sleep(0.3)

        stats = client.get_packet_stats()
        assert stats["received"] == 5
        assert stats["failed_decode"] == 1

        sock.close()
        client.stop()

    def test_not_connected_without_heartbeat(self):
        port = _find_free_port()
        client = TelemetryClient(host="127.0.0.1", port=port, timeout=0.5)
        client.start()
        assert not client.is_connected
        client.stop()

    def test_get_state_returns_copy(self):
        port = _find_free_port()
        client = TelemetryClient(host="127.0.0.1", port=port, timeout=0.5)
        client.start()

        s1 = client.get_state()
        s2 = client.get_state()
        assert s1 is not s2  # different objects

        client.stop()


class TestMiniIntegration:
    """Quick integration: send a burst of mixed messages, verify state converges."""

    def test_mixed_message_burst(self):
        port = _find_free_port()
        client = TelemetryClient(host="127.0.0.1", port=port, timeout=1.0)
        client.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target = ("127.0.0.1", port)

        # Send a mix of messages
        sock.sendto(encode(Heartbeat(base_mode=209, system_status=4)), target)
        sock.sendto(encode(Attitude(time_boot_ms=100, roll=0.1, yaw=1.0)), target)
        sock.sendto(encode(Odometry(x=5.0, y=10.0, z=-3.0, vx=2.0)), target)
        sock.sendto(encode(HighresImu(xacc=1.0, zacc=-9.81)), target)
        sock.sendto(encode(Timesync(ts1=123456789)), target)
        time.sleep(0.3)

        state = client.get_state()
        assert state.armed is True
        assert state.roll == pytest.approx(0.1, abs=1e-5)
        assert state.x == pytest.approx(5.0, abs=1e-4)
        assert state.xacc == pytest.approx(1.0, abs=1e-5)

        stats = client.get_packet_stats()
        assert stats["received"] == 5

        sock.close()
        client.stop()
