"""Regression tests for Algorithm.latest_sensor_snapshot."""

from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from src.control.algorithms import Algorithm
from src.control.algorithms.autonomous_explore import AutonomousExplore
from src.control.highres_imu import HighresImuHealth, SensorSnapshot


class _StubKinematics:
    position = SimpleNamespace(z_val=-5.0)
    orientation = SimpleNamespace(w_val=1.0, x_val=0.0, y_val=0.0, z_val=0.0)


class AlgorithmSensorSnapshotTests(unittest.TestCase):
    def test_autonomous_explore_has_latest_sensor_snapshot(self) -> None:
        self.assertTrue(callable(getattr(Algorithm, "latest_sensor_snapshot", None)))

    def test_latest_sensor_snapshot_returns_health(self) -> None:
        health = HighresImuHealth(
            status="ok",
            reason="stub",
            enabled=True,
            sample_count=1,
            stream_rate_hz=None,
            update_age_ms=0.0,
            max_staleness_ms=1000.0,
            sensor_count=0,
            active_sensor_ids=(),
            expected_rate_hz=None,
        )
        client = MagicMock()
        client.getMultirotorState.return_value = SimpleNamespace(
            kinematics_estimated=_StubKinematics()
        )
        client.getHighresImu.return_value = None
        client.getHighresImuHealth.return_value = health

        algo = AutonomousExplore({})
        snapshot = algo.latest_sensor_snapshot(client)

        self.assertIsInstance(snapshot, SensorSnapshot)
        self.assertIs(snapshot.highres_imu_health, health)
        self.assertEqual(snapshot.transport, "unknown")
