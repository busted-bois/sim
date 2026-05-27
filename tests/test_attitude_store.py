import threading
import time
import unittest

from src.mavlink.attitude import AttitudeSample
from src.mavlink.attitude_store import AttitudeStore


class AttitudeStoreTests(unittest.TestCase):
    def test_health_missing_when_no_samples(self) -> None:
        store = AttitudeStore()
        health = store.get_health(enabled=True, max_staleness_ms=500.0)
        self.assertEqual(health.status, "missing")

    def test_health_ok_for_fresh_sample(self) -> None:
        store = AttitudeStore()
        store.update(
            AttitudeSample(
                time_boot_ms=1,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                rollspeed=0.0,
                pitchspeed=0.0,
                yawspeed=0.0,
                local_received_monotonic_ns=time.monotonic_ns(),
            )
        )
        health = store.get_health(enabled=True, max_staleness_ms=5000.0)
        self.assertEqual(health.status, "ok")
        self.assertTrue(health.is_healthy())

    def test_health_stale_after_age(self) -> None:
        store = AttitudeStore()
        old_ns = time.monotonic_ns() - 2_000_000_000
        store.update(
            AttitudeSample(
                time_boot_ms=1,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                rollspeed=0.0,
                pitchspeed=0.0,
                yawspeed=0.0,
                local_received_monotonic_ns=old_ns,
            )
        )
        health = store.get_health(enabled=True, max_staleness_ms=500.0)
        self.assertEqual(health.status, "stale")

    def test_concurrent_updates(self) -> None:
        store = AttitudeStore()
        sample = AttitudeSample(
            time_boot_ms=1,
            roll=0.1,
            pitch=0.2,
            yaw=0.3,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
            local_received_monotonic_ns=time.monotonic_ns(),
        )

        def worker() -> None:
            for _ in range(100):
                store.update(sample)

        threads = [threading.Thread(target=worker) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(store.sample_count(), 400)


if __name__ == "__main__":
    unittest.main()
