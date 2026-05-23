"""User-visible session log output paths (CSV / SLAM export)."""

from __future__ import annotations

import sys
from pathlib import Path


def print_session_log_plan(
    *,
    transport: str,
    env_transport: str,
    landing_csv_path: Path | None,
    position_trace_path: Path | None,
    local_tracker_path: Path | None,
    slam_export_path: str,
    slam_export_enabled: bool,
    internal_mapping_enabled: bool,
    internal_mapping_path: str | None,
) -> None:
    print("[logs] Outputs for this session:")
    transport_line = f"  transport={transport!r}"
    if env_transport:
        transport_line += f" (env AIGP_CONTROL_TRANSPORT={env_transport!r})"
    print(transport_line)
    if internal_mapping_enabled and internal_mapping_path:
        print(
            f"  internal_mapping CSV (fused tracking or RPC fallback, not SLAM): "
            f"{internal_mapping_path}"
        )
    else:
        print("  internal_mapping CSV: disabled (exploration.internal_mapping.enabled=false)")
    if transport != "mavlink":
        print(
            "  MAVLink CSVs (position_trace, tracking): OFF — only active when transport is "
            "'mavlink' (PX4-SITL)."
        )
    else:
        if position_trace_path is not None:
            print(f"  position_trace CSV (on shutdown): {position_trace_path}")
        if local_tracker_path is not None:
            print(f"  tracking_state CSV (on shutdown): {local_tracker_path}")
    if landing_csv_path is not None:
        print(f"  landing_telemetry CSV (after landing phase): {landing_csv_path}")
    if slam_export_enabled:
        print(f"  SLAM map JSON (separate from internal_mapping): {slam_export_path}")


def warn_missing_session_logs(
    *,
    transport: str,
    landing_csv_path: Path | None,
    position_trace_path: Path | None,
    internal_mapping_path: Path | None,
    logs_dir: Path,
) -> None:
    if landing_csv_path is not None and not landing_csv_path.is_file():
        print(
            f"Warning: landing telemetry CSV not written: {landing_csv_path} "
            "(landing phase did not run — e.g. Ctrl+C during algorithm).",
            file=sys.stderr,
        )
    if internal_mapping_path is not None and not internal_mapping_path.is_file():
        print(
            f"Warning: internal_mapping CSV not written: {internal_mapping_path}",
            file=sys.stderr,
        )
    if transport != "mavlink":
        print(
            "Warning: no MAVLink position_trace/tracking CSV this run (transport is not mavlink).",
            file=sys.stderr,
        )
    elif position_trace_path is not None and not position_trace_path.is_file():
        print(
            f"Warning: position_trace CSV not written: {position_trace_path}",
            file=sys.stderr,
        )
    if logs_dir.is_dir():
        newest = max(logs_dir.glob("*.csv"), key=lambda p: p.stat().st_mtime, default=None)
        if newest is not None:
            print(f"[logs] Newest CSV in {logs_dir}: {newest.name}")
        slam_dir = logs_dir / "slam"
        if slam_dir.is_dir():
            newest_slam = max(
                slam_dir.glob("*.json"),
                key=lambda p: p.stat().st_mtime,
                default=None,
            )
            if newest_slam is not None:
                print(f"[logs] Newest SLAM JSON in {slam_dir}: {newest_slam.name}")
