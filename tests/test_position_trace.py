"""Unit tests for MAVLink position trace store."""

from __future__ import annotations

import csv
import tempfile
import threading
import unittest
from pathlib import Path

from src.position_trace import CSV_HEADER, PositionTraceStore


class PositionTraceStoreTests(unittest.TestCase):
    def test_anchor_and_relative_second_sample(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            store = PositionTraceStore(Path(tmp) / "trace.csv", max_speed_ms=20.0)
            self.assertTrue(
                store.record(1000, 1.0, 2.0, -5.0, 0.0, 0.0, 0.0, 0)
            )
            self.assertTrue(
                store.record(1050, 1.5, 2.0, -5.0, 0.1, 0.0, 0.0, 0)
            )
            rows = store.rows()
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(rows[0][0], 0.0)
        self.assertAlmostEqual(rows[0][2], 0.0)
        self.assertAlmostEqual(rows[0][3], 0.0)
        self.assertAlmostEqual(rows[0][4], 0.0)
        self.assertAlmostEqual(rows[1][0], 0.05)
        self.assertAlmostEqual(rows[1][2], 0.5)
        self.assertAlmostEqual(rows[1][3], 0.0)
        self.assertAlmostEqual(rows[1][4], 0.0)

    def test_stationary_samples_do_not_integrate_drift(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            store = PositionTraceStore(Path(tmp) / "trace.csv", max_speed_ms=10.0)
            store.record(0, 1.0, 2.0, -5.0, 0.0, 0.0, 0.0, 0)
            for index in range(1, 101):
                noise = (index % 5) * 0.001
                store.record(
                    index * 50,
                    1.0 + noise,
                    2.0,
                    -5.0,
                    0.0,
                    0.0,
                    0.0,
                    0,
                )
            rows = store.rows()
        for row in rows:
            self.assertLess(abs(row[2]), 0.01)
            self.assertAlmostEqual(row[3], 0.0)
            self.assertAlmostEqual(row[4], 0.0)

    def test_reject_duplicate_time_boot_ms(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            store = PositionTraceStore(Path(tmp) / "trace.csv")
            store.record(1000, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0, 0)
            self.assertFalse(store.record(1000, 1.0, 0.0, -5.0, 0.0, 0.0, 0.0, 0))
            health = store.health()
        self.assertEqual(health.accepted_count, 1)
        self.assertEqual(health.rejected_count, 1)
        self.assertEqual(health.last_reject_reason, "duplicate time_boot_ms")

    def test_reject_time_regression(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            store = PositionTraceStore(Path(tmp) / "trace.csv")
            store.record(2000, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0, 0)
            self.assertFalse(store.record(1500, 0.1, 0.0, -5.0, 0.0, 0.0, 0.0, 0))
            health = store.health()
        self.assertEqual(health.rejected_count, 1)
        self.assertEqual(health.last_reject_reason, "time_boot_ms regression")

    def test_reject_outlier_step(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            store = PositionTraceStore(
                Path(tmp) / "trace.csv",
                max_speed_ms=10.0,
                plausible_step_slack=1.5,
            )
            store.record(1000, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0, 0)
            self.assertFalse(store.record(1050, 100.0, 0.0, -5.0, 0.0, 0.0, 0.0, 0))
            health = store.health()
        self.assertEqual(health.accepted_count, 1)
        self.assertEqual(health.rejected_count, 1)
        self.assertIn("exceeds plausible", health.last_reject_reason or "")

    def test_anchor_not_reset_after_rejected_outlier(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            store = PositionTraceStore(Path(tmp) / "trace.csv", max_speed_ms=10.0)
            store.record(1000, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0, 0)
            store.record(1050, 100.0, 0.0, -5.0, 0.0, 0.0, 0.0, 0)
            store.record(1100, 0.1, 0.0, -5.0, 0.0, 0.0, 0.0, 0)
            rows = store.rows()
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(rows[1][2], 0.1)

    def test_flush_writes_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trace.csv"
            store = PositionTraceStore(path)
            store.record(0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0)
            store.flush()
            store.flush()
            with path.open(encoding="utf-8") as handle:
                rows = list(csv.reader(handle))
        self.assertEqual(rows[0], list(CSV_HEADER))
        self.assertEqual(len(rows), 2)

    def test_concurrent_record(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            store = PositionTraceStore(Path(tmp) / "trace.csv", max_speed_ms=50.0)

            def worker(start_ms: int) -> None:
                for offset in range(10):
                    store.record(
                        start_ms + offset * 50,
                        float(start_ms + offset),
                        0.0,
                        -5.0,
                        0.0,
                        0.0,
                        0.0,
                        0,
                    )

            bases = (0, 10_000, 20_000)
            threads = [threading.Thread(target=worker, args=(base,)) for base in bases]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            health = store.health()
        self.assertGreaterEqual(health.accepted_count, 1)


class RpcPositionSnapshotProviderTests(unittest.TestCase):
    def test_builds_trace_snapshot_from_multirotor_state(self) -> None:
        from dataclasses import dataclass

        from src.position_trace import RpcPositionSnapshotProvider

        @dataclass
        class _Vec:
            x_val: float
            y_val: float
            z_val: float

        @dataclass
        class _Kin:
            position: _Vec
            linear_velocity: _Vec

        @dataclass
        class _State:
            timestamp: int
            kinematics_estimated: _Kin

        class _Client:
            def getMultirotorState(self) -> _State:
                return _State(
                    timestamp=42,
                    kinematics_estimated=_Kin(
                        position=_Vec(1.0, -2.0, -5.0),
                        linear_velocity=_Vec(0.1, 0.2, 0.0),
                    ),
                )

        provider = RpcPositionSnapshotProvider(_Client())
        snap = provider()
        self.assertIsNotNone(snap.latest)
        assert snap.latest is not None
        self.assertAlmostEqual(snap.latest[2], 1.0)
        self.assertAlmostEqual(snap.latest[8], 5.0)


if __name__ == "__main__":
    unittest.main()
