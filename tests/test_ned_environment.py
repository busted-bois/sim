import math
import unittest

from src.control.ned_environment import (
    MavlinkNedFrame,
    MavlinkNedIngest,
    NedEnvironmentMap,
    NedEnvironmentSettings,
    NedTransformError,
    NedVector3,
)
from tests.mavlink_fakes import FakeMessage


class NedEnvironmentMapTests(unittest.TestCase):
    def test_body_forward_at_yaw_zero_maps_to_north(self) -> None:
        mapper = NedEnvironmentMap()
        mapper.update_attitude(roll=0.0, pitch=0.0, yaw=0.0)
        out = mapper.transform_vector(
            NedVector3(1.0, 0.0, 0.0),
            from_frame=MavlinkNedFrame.BODY_NED,
            to_frame=MavlinkNedFrame.LOCAL_NED,
        )
        self.assertAlmostEqual(out.x, 1.0, places=5)
        self.assertAlmostEqual(out.y, 0.0, places=5)

    def test_body_forward_at_yaw_90_maps_to_east(self) -> None:
        mapper = NedEnvironmentMap()
        mapper.update_attitude(roll=0.0, pitch=0.0, yaw=math.pi / 2)
        out = mapper.transform_vector(
            NedVector3(1.0, 0.0, 0.0),
            from_frame=MavlinkNedFrame.BODY_NED,
            to_frame=MavlinkNedFrame.LOCAL_NED,
        )
        self.assertAlmostEqual(out.x, 0.0, places=5)
        self.assertAlmostEqual(out.y, 1.0, places=5)

    def test_round_trip_body_local_body(self) -> None:
        mapper = NedEnvironmentMap()
        mapper.update_attitude(roll=0.0, pitch=0.0, yaw=0.7)
        original = NedVector3(2.0, -1.0, -0.5)
        local = mapper.transform_vector(
            original,
            from_frame=MavlinkNedFrame.BODY_NED,
            to_frame=MavlinkNedFrame.LOCAL_NED,
        )
        back = mapper.transform_vector(
            local,
            from_frame=MavlinkNedFrame.LOCAL_NED,
            to_frame=MavlinkNedFrame.BODY_NED,
        )
        self.assertAlmostEqual(back.x, original.x, places=5)
        self.assertAlmostEqual(back.y, original.y, places=5)
        self.assertAlmostEqual(back.z, original.z, places=5)

    def test_spawn_relative_xy(self) -> None:
        mapper = NedEnvironmentMap()
        mapper.update_local_position_ned(x=10.0, y=4.0, z=-5.0, vx=0, vy=0, vz=0)
        mapper.set_spawn_origin(8.0, 1.0)
        rel = mapper.spawn_relative_xy()
        self.assertIsNotNone(rel)
        assert rel is not None
        self.assertAlmostEqual(rel[0], 2.0, places=5)
        self.assertAlmostEqual(rel[1], 3.0, places=5)

    def test_transform_without_attitude_raises(self) -> None:
        mapper = NedEnvironmentMap()
        with self.assertRaises(NedTransformError):
            mapper.body_velocity_to_local(1.0, 0.0, 0.0)

    def test_mavlink_ingest_updates_snapshot(self) -> None:
        mapper = NedEnvironmentMap()
        ingest = MavlinkNedIngest(mapper)
        ingest.on_local_position_ned(
            FakeMessage("LOCAL_POSITION_NED", x=1.0, y=2.0, z=-3.0, vx=0.1, vy=0.2, vz=0.3)
        )
        ingest.on_attitude(
            FakeMessage("ATTITUDE", roll=0.0, pitch=0.0, yaw=1.5, time_boot_ms=100)
        )
        snap = mapper.snapshot()
        self.assertTrue(snap.has_position)
        self.assertTrue(snap.has_attitude)
        self.assertAlmostEqual(snap.position.z, -3.0, places=5)
        self.assertAlmostEqual(snap.attitude.yaw if snap.attitude else 0.0, 1.5, places=5)

    def test_disabled_mapper_ignores_updates(self) -> None:
        mapper = NedEnvironmentMap(NedEnvironmentSettings(enabled=False))
        mapper.update_local_position_ned(x=1.0, y=2.0, z=3.0, vx=0, vy=0, vz=0)
        snap = mapper.snapshot()
        self.assertFalse(snap.has_position)


if __name__ == "__main__":
    unittest.main()
