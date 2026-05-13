import unittest

from pymavlink import mavutil

from src.control.flight_client import (
    SET_POSITION_FRAME_BODY_NED,
    SET_POSITION_FRAME_LOCAL_NED,
    SetPositionTargetLocalNedCommand,
    build_position_target_type_mask,
    build_position_type_mask,
    build_velocity_type_mask,
)
from src.control.mavlink_client import PymavlinkFlightClient
from tests.mavlink_fakes import FakeMavConnection, FakeMessage


class PymavlinkSetPositionTargetLocalNedTests(unittest.TestCase):
    def test_submit_supports_body_ned_and_full_payload(self) -> None:
        client, connection = self._make_client()
        try:
            client.submitSetPositionTargetLocalNed(
                SetPositionTargetLocalNedCommand(
                    frame="body_ned",
                    type_mask=0x0123,
                    x=1.0,
                    y=2.0,
                    z=-3.0,
                    vx=0.4,
                    vy=0.5,
                    vz=0.6,
                    afx=0.7,
                    afy=0.8,
                    afz=0.9,
                    yaw=1.1,
                    yaw_rate=1.2,
                )
            )
        finally:
            client.close()

        self.assertEqual(len(connection.mav.position_target_calls), 1)
        call = connection.mav.position_target_calls[0]
        self.assertEqual(call[3], mavutil.mavlink.MAV_FRAME_BODY_NED)
        self.assertEqual(call[4], 0x0123)
        self.assertEqual(call[5:], (1.0, 2.0, -3.0, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.1, 1.2))

    def test_submit_rejects_unknown_frame(self) -> None:
        client, _ = self._make_client()
        try:
            with self.assertRaises(ValueError):
                client.submitSetPositionTargetLocalNed(
                    SetPositionTargetLocalNedCommand(
                        frame="bad_frame",  # type: ignore[arg-type]
                        type_mask=0,
                    )
                )
        finally:
            client.close()

    def test_move_by_velocity_uses_velocity_only_type_mask(self) -> None:
        client, connection = self._make_client()
        try:
            client.moveByVelocityAsync(1.0, 2.0, 3.0, 0.05).join()
        finally:
            client.close()

        self.assertGreaterEqual(len(connection.mav.position_target_calls), 1)
        call = connection.mav.position_target_calls[0]
        expected_mask = (
            int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        )
        self.assertEqual(call[3], mavutil.mavlink.MAV_FRAME_LOCAL_NED)
        self.assertEqual(call[4], expected_mask)
        self.assertEqual(call[5:8], (0.0, 0.0, 0.0))
        self.assertEqual(call[8:11], (1.0, 2.0, 3.0))

    def test_type_mask_builder_supports_mixed_fields(self) -> None:
        mask = build_position_target_type_mask(
            use_position=True,
            use_velocity=True,
            use_acceleration=False,
            use_yaw=True,
            use_yaw_rate=False,
            force_set=False,
        )
        ignored_mask = (
            int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
            | int(mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        )
        self.assertEqual(mask, ignored_mask)

    def test_convenience_submitters_use_expected_frames_and_masks(self) -> None:
        client, connection = self._make_client()
        try:
            client.submitVelocityLocalNed(0.1, 0.2, 0.3)
            client.submitVelocityBodyNed(1.1, 1.2, 1.3)
            client.submitPositionLocalNed(3.0, 4.0, -5.0)
        finally:
            client.close()

        self.assertEqual(len(connection.mav.position_target_calls), 3)
        velocity_mask = build_velocity_type_mask()
        position_mask = build_position_type_mask()

        local_velocity_call = connection.mav.position_target_calls[0]
        body_velocity_call = connection.mav.position_target_calls[1]
        local_position_call = connection.mav.position_target_calls[2]

        self.assertEqual(local_velocity_call[3], mavutil.mavlink.MAV_FRAME_LOCAL_NED)
        self.assertEqual(local_velocity_call[4], velocity_mask)
        self.assertEqual(local_velocity_call[8:11], (0.1, 0.2, 0.3))

        self.assertEqual(body_velocity_call[3], mavutil.mavlink.MAV_FRAME_BODY_NED)
        self.assertEqual(body_velocity_call[4], velocity_mask)
        self.assertEqual(body_velocity_call[8:11], (1.1, 1.2, 1.3))

        self.assertEqual(local_position_call[3], mavutil.mavlink.MAV_FRAME_LOCAL_NED)
        self.assertEqual(local_position_call[4], position_mask)
        self.assertEqual(local_position_call[5:8], (3.0, 4.0, -5.0))

    def test_short_mask_helpers_match_general_builder(self) -> None:
        self.assertEqual(
            build_velocity_type_mask(),
            build_position_target_type_mask(use_velocity=True, force_set=True),
        )
        self.assertEqual(
            build_position_type_mask(),
            build_position_target_type_mask(use_position=True),
        )

    def test_full_payload_mixed_fields_passthrough(self) -> None:
        client, connection = self._make_client()
        try:
            mixed_mask = build_position_target_type_mask(
                use_position=True,
                use_velocity=False,
                use_acceleration=False,
                use_yaw=True,
                use_yaw_rate=False,
            )
            client.submitSetPositionTargetLocalNed(
                SetPositionTargetLocalNedCommand(
                    frame=SET_POSITION_FRAME_BODY_NED,
                    type_mask=mixed_mask,
                    x=7.0,
                    y=8.0,
                    z=-9.0,
                    yaw=0.25,
                )
            )
        finally:
            client.close()

        call = connection.mav.position_target_calls[0]
        self.assertEqual(call[3], mavutil.mavlink.MAV_FRAME_BODY_NED)
        self.assertEqual(call[4], mixed_mask)
        self.assertEqual(call[5:8], (7.0, 8.0, -9.0))
        self.assertEqual(call[14], 0.25)
        self.assertEqual(call[15], 0.0)

    def test_stream_set_position_repeats_setpoint_at_command_rate(self) -> None:
        client, connection = self._make_client(command_rate_hz=20.0)
        try:
            client.streamSetPositionTargetLocalNedAsync(
                SetPositionTargetLocalNedCommand(
                    frame=SET_POSITION_FRAME_LOCAL_NED,
                    type_mask=build_velocity_type_mask(),
                    vx=0.5,
                    vy=0.0,
                    vz=0.0,
                ),
                0.25,
            ).join()
        finally:
            client.close()

        self.assertGreaterEqual(len(connection.mav.position_target_calls), 3)

    @staticmethod
    def _make_client(
        *,
        command_rate_hz: float = 50.0,
    ) -> tuple[PymavlinkFlightClient, FakeMavConnection]:
        heartbeat = FakeMessage(
            "HEARTBEAT",
            base_mode=0,
            source_system=42,
            source_component=24,
        )
        connection = FakeMavConnection(heartbeat, [])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            command_rate_hz=command_rate_hz,
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            request_state_messages_on_connect=False,
            connection_factory=lambda *args, **kwargs: connection,
        )
        client.confirmConnection()
        return client, connection


if __name__ == "__main__":
    unittest.main()
