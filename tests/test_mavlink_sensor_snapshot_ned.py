import time
import unittest

from src.control.algorithms import Algorithm
from src.control.mavlink_client import PymavlinkFlightClient
from tests.mavlink_fakes import FakeMavConnection, FakeMessage


class MavlinkSensorSnapshotNedTests(unittest.TestCase):
    def test_latest_sensor_snapshot_includes_ned(self) -> None:
        heartbeat = FakeMessage("HEARTBEAT", base_mode=0, source_system=42, source_component=24)
        local = FakeMessage(
            "LOCAL_POSITION_NED",
            x=1.0,
            y=2.0,
            z=-3.0,
            vx=0.0,
            vy=0.0,
            vz=0.0,
        )
        attitude = FakeMessage("ATTITUDE", roll=0.0, pitch=0.0, yaw=0.25)
        connection = FakeMavConnection(heartbeat, [local, attitude])
        client = PymavlinkFlightClient(
            endpoint="udpin:0.0.0.0:14550",
            send_timesync_requests=False,
            prepare_for_flight_on_connect=False,
            connection_factory=lambda *args, **kwargs: connection,
        )
        try:
            client.confirmConnection()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                snap = Algorithm({}).latest_sensor_snapshot(client)
                if snap.ned is not None and snap.ned.transform_ready:
                    self.assertAlmostEqual(snap.ned.position.x, 1.0, places=3)
                    self.assertIsNotNone(snap.ned_health)
                    return
                time.sleep(0.02)
            self.fail("NED snapshot never became transform_ready")
        finally:
            client.close()


if __name__ == "__main__":
    unittest.main()
