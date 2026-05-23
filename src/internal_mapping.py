"""NED pose / velocity CSV from flight client RPC (AirSim now; MAVLink NED later).

Separate from ``exploration.slam`` map export. Samples on a background thread and
flushes each row so Ctrl+C still leaves a usable trace.
"""

from __future__ import annotations

import csv
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, TextIO

from src.control.utils import _yaw_from_orientation
from src.log_paths import resolve_log_csv_path

if TYPE_CHECKING:
    from src.control.flight_client import FlightClient

CSV_HEADER = (
    "session_t_s",
    "airsim_timestamp",
    "pos_n_m",
    "pos_e_m",
    "pos_d_m",
    "yaw_rad",
    "vn_ms",
    "ve_ms",
    "vd_ms",
)


def internal_mapping_config(config: dict[str, Any]) -> dict[str, Any]:
    """``autonomous_explore.exploration.internal_mapping`` or legacy top-level key."""
    explore = config.get("autonomous_explore", {}).get("exploration", {}).get(
        "internal_mapping", {}
    )
    if explore:
        return dict(explore)
    return dict(config.get("internal_mapping", {}))


def internal_mapping_logger_from_config(
    config: dict[str, Any],
    project_root: Path,
    client: FlightClient,
) -> InternalMappingLogger | None:
    cfg = internal_mapping_config(config)
    if not bool(cfg.get("enabled", False)):
        return None
    out_path = resolve_log_csv_path(
        str(cfg.get("path", "logs/internal_mapping_{timestamp}.csv")),
        project_root,
        default="logs/internal_mapping_{timestamp}.csv",
    )
    return InternalMappingLogger(
        client,
        out_path=out_path,
        sample_hz=float(cfg.get("sample_hz", 20.0)),
    )


class InternalMappingLogger:
    """Threaded CSV writer; NED frame (x North, y East, z Down)."""

    def __init__(
        self,
        client: FlightClient,
        *,
        out_path: Path,
        sample_hz: float = 20.0,
    ) -> None:
        self._client = client
        self.out_path = out_path
        self._period = 1.0 / max(1.0, float(sample_hz))
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fh: TextIO | None = None
        self._writer: csv.writer | None = None
        self._t0 = 0.0
        self._row_count = 0

    @property
    def row_count(self) -> int:
        return self._row_count

    def start(self) -> None:
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self._t0 = time.monotonic()
        self._fh = self.out_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(CSV_HEADER)
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
            self._row_count += 1
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
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        self._writer = None
