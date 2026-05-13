"""Sample altitude, vertical velocity, command, and optional IMU data during landing."""

from __future__ import annotations

import csv
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.control.flight_client import FlightClient


class LandingTelemetrySampler:
    """Background sampler; call set_command() from the main thread as the phase changes."""

    def __init__(
        self,
        client: FlightClient,
        out_path: Path,
        sample_hz: float,
    ) -> None:
        self._client = client
        self.out_path = out_path
        self._period = 1.0 / max(1.0, sample_hz)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._command = "init"
        self._t0 = 0.0
        self._rows: list[tuple[float, float, float, str, float, float]] = []

    def set_command(self, cmd: str) -> None:
        with self._lock:
            self._command = cmd

    def _get_command(self) -> str:
        with self._lock:
            return self._command

    def start(self) -> None:
        self._t0 = time.monotonic()

        def loop() -> None:
            while not self._stop.is_set():
                self._sample()
                if self._stop.wait(self._period):
                    break

        self._thread = threading.Thread(target=loop, name="landing_telemetry", daemon=True)
        self._thread.start()

    def _sample(self) -> None:
        cmd = self._get_command()
        t = time.monotonic() - self._t0
        try:
            s = self._client.getMultirotorState().kinematics_estimated
            z = float(s.position.z_val)
            vz = float(s.linear_velocity.z_val)
            alt_m = max(0.0, -z)
            imu = self._client.getHighresImu()
            imu_zacc = float("nan") if imu is None or imu.zacc is None else float(imu.zacc)
            imu_zgyro = float("nan") if imu is None or imu.zgyro is None else float(imu.zgyro)
            self._rows.append((t, alt_m, vz, cmd, imu_zacc, imu_zgyro))
        except Exception:
            self._rows.append(
                (t, float("nan"), float("nan"), f"{cmd}|sample_error", float("nan"), float("nan"))
            )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._sample()
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        with self.out_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "altitude_m", "vz_ms", "command", "imu_zacc_ms2", "imu_zgyro_rads"])
            w.writerows(self._rows)
