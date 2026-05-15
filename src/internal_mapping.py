"""NED pose / velocity log from AirSim RPC for local trajectory mapping (MAVLink later).

Samples ``getMultirotorState()`` on a background thread and writes CSV rows with flush
per sample so the file reflects the latest pose if the process stops abruptly.
"""

from __future__ import annotations

import csv
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, TextIO

from src.control.utils import _yaw_from_orientation

if TYPE_CHECKING:
    from src.control.flight_client import FlightClient


class InternalMappingLogger:
    """Threaded CSV writer; AirSim NED frame (x North, y East, z Down)."""

    def __init__(self, client: FlightClient, cfg: dict) -> None:
        self._client = client
        self._enabled = bool(cfg.get("enabled", False))
        self._out_path = Path(str(cfg.get("path", "logs/internal_mapping.csv")).strip())
        self._period = 1.0 / max(1.0, float(cfg.get("sample_hz", 20.0)))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fh: TextIO | None = None
        self._writer: csv.writer | None = None
        self._t0 = 0.0

    def start(self) -> None:
        if not self._enabled:
            return
        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        self._t0 = time.monotonic()
        self._fh = self._out_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(
            [
                "session_t_s",
                "airsim_timestamp",
                "pos_n_m",
                "pos_e_m",
                "pos_d_m",
                "yaw_rad",
                "vn_ms",
                "ve_ms",
                "vd_ms",
            ]
        )
        self._fh.flush()

        def loop() -> None:
            while not self._stop.is_set():
                self._sample()
                if self._stop.wait(self._period):
                    break
            self._sample()

        self._thread = threading.Thread(target=loop, name="internal_mapping", daemon=True)
        self._thread.start()

    def _sample(self) -> None:
        if self._writer is None or self._fh is None:
            return
        session_t = time.monotonic() - self._t0
        try:
            state = self._client.getMultirotorState()
            k = state.kinematics_estimated
            p = k.position
            v = k.linear_velocity
            yaw = _yaw_from_orientation(k.orientation)
            ts = int(state.timestamp)
            self._writer.writerow(
                [
                    f"{session_t:.6f}",
                    ts,
                    f"{float(p.x_val):.6f}",
                    f"{float(p.y_val):.6f}",
                    f"{float(p.z_val):.6f}",
                    f"{yaw:.6f}",
                    f"{float(v.x_val):.6f}",
                    f"{float(v.y_val):.6f}",
                    f"{float(v.z_val):.6f}",
                ]
            )
            self._fh.flush()
        except Exception as exc:
            self._writer.writerow(
                [
                    f"{session_t:.6f}",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    f"error:{exc}",
                ]
            )
            self._fh.flush()

    def stop(self) -> None:
        if not self._enabled:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        self._writer = None
