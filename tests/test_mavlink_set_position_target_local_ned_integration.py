import socket
import threading
import time
import unittest

from pymavlink import mavutil

from src.control.mavlink_client import PymavlinkFlightClient


class SetPositionTargetLocalNedIntegrationTests(unittest.TestCase):
    def test_udp_loopback_emits_local_and_body_velocity_setpoints(self) -> None:
        port = self._reserve_udp_port()
        vehicle = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{port}",
            source_system=42,
            source_component=24,
        )
        stop_evt = threading.Event()
        heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            args=(vehicle, stop_evt),
            daemon=True,
        )
        heartbeat_thread.start()

        client = PymavlinkFlightClient(
            endpoint=f"udpin:0.0.0.0:{port}",
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            request_state_messages_on_connect=False,
        )
        try:
            client.confirmConnection()
            client.submitVelocityLocalNed(0.3, 0.2, -0.1)
            client.submitVelocityBodyNed(1.3, -0.2, 0.4)

            messages = []
            deadline = time.time() + 3.0
            while len(messages) < 2 and time.time() < deadline:
                msg = vehicle.recv_match(
                    type="SET_POSITION_TARGET_LOCAL_NED",
                    blocking=True,
                    timeout=0.5,
                )
                if msg is not None:
                    messages.append(msg)

            self.assertEqual(len(messages), 2)
            self.assertEqual(messages[0].coordinate_frame, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
            self.assertEqual(messages[1].coordinate_frame, mavutil.mavlink.MAV_FRAME_BODY_NED)
            self.assertAlmostEqual(float(messages[0].vx), 0.3, places=3)
            self.assertAlmostEqual(float(messages[1].vx), 1.3, places=3)
        finally:
            stop_evt.set()
            heartbeat_thread.join(timeout=1.0)
            client.close()
            vehicle.close()

    @staticmethod
    def _heartbeat_loop(connection, stop_evt: threading.Event) -> None:
        while not stop_evt.is_set():
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_PX4,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            time.sleep(0.1)

    @staticmethod
    def _reserve_udp_port() -> int:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        port = int(sock.getsockname()[1])
        sock.close()
        return port


if __name__ == "__main__":
    unittest.main()
