import unittest
from unittest import mock

from pymavlink import mavutil

from src.control.mavlink_client import PymavlinkFlightClient
from tests.mavlink_fakes import FakeMavConnection, FakeMessage


def _client(connection: FakeMavConnection) -> PymavlinkFlightClient:
    return PymavlinkFlightClient(
        endpoint="udpin:0.0.0.0:14550",
        send_timesync_requests=False,
        prepare_for_flight_on_connect=False,
        request_state_messages_on_connect=False,
        highres_imu_enabled=False,
        guided_custom_mode=6,
        log_commands=False,
        connection_factory=lambda *a, **k: connection,
    )


class MavlinkClientArmOffboardTests(unittest.TestCase):
    def test_force_arm_uses_px4_magic_param(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=3, source_component=1)
        connection = FakeMavConnection(heartbeat, [])
        client = _client(connection)
        try:
            client.confirmConnection()
            client._send_arm_disarm_command(True, force=True)
            arm_cmds = [
                c
                for c in connection.mav.command_long_calls
                if c[2] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            ]
            self.assertEqual(len(arm_cmds), 1)
            self.assertEqual(arm_cmds[0][4], 1)
            self.assertEqual(arm_cmds[0][5], 21196)
        finally:
            client.close()

    def test_arm_raises_when_vehicle_stays_disarmed(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=3, source_component=1)
        connection = FakeMavConnection(heartbeat, [])
        client = _client(connection)
        try:
            client.confirmConnection()
            clock = {"t": 0.0}

            def monotonic() -> float:
                clock["t"] += 1.0
                return clock["t"]

            with mock.patch.object(client, "is_armed", return_value=False):
                with mock.patch("src.control.mavlink_client.time.monotonic", monotonic):
                    with mock.patch("src.control.mavlink_client.time.sleep", return_value=None):
                        with self.assertRaises(RuntimeError):
                            client.armDisarm(True)
            arm_cmds = [
                c
                for c in connection.mav.command_long_calls
                if c[2] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            ]
            self.assertEqual(len(arm_cmds), 2)
        finally:
            client.close()

    def test_offboard_mode_primes_setpoints_before_do_set_mode(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=3, source_component=1)
        connection = FakeMavConnection(heartbeat, [])
        client = _client(connection)
        try:
            client.confirmConnection()
            client._set_guided_mode(force=True)
            self.assertGreaterEqual(len(connection.mav.position_target_calls), 10)
            mode_cmds = [
                c
                for c in connection.mav.command_long_calls
                if c[2] == mavutil.mavlink.MAV_CMD_DO_SET_MODE
            ]
            self.assertTrue(mode_cmds)
            self.assertEqual(mode_cmds[-1][5], 6)
        finally:
            client.close()


if __name__ == "__main__":
    unittest.main()
