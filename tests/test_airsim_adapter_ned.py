import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from src.control.flight_client import AirSimAdapter


class AirSimAdapterNedTests(unittest.TestCase):
    def test_get_ned_environment_refreshes_from_rpc_state(self) -> None:
        rpc = MagicMock()
        rpc.getMultirotorState.return_value = SimpleNamespace(
            kinematics_estimated=SimpleNamespace(
                position=SimpleNamespace(x_val=3.0, y_val=-1.0, z_val=-5.0),
                linear_velocity=SimpleNamespace(x_val=0.1, y_val=0.2, z_val=0.0),
                orientation=SimpleNamespace(x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0),
            )
        )
        adapter = AirSimAdapter(rpc)
        ned = adapter.get_ned_environment()
        snap = ned.snapshot()
        self.assertAlmostEqual(snap.position.x, 3.0, places=4)
        self.assertTrue(snap.has_position)


if __name__ == "__main__":
    unittest.main()
