"""Preflight checks for simulator/drone runtime configuration."""

from __future__ import annotations

import json
import os
import socket
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from src.config import load_config, resolve_config_path, simulator_endpoint
from src.control.algorithms import list_algorithms
from src.mavlink_endpoints import candidate_mavlink_endpoints, resolve_control_transport
from src.simulator_specs import conformity_fingerprint, resolve_specification_path

ROOT = Path(__file__).resolve().parent.parent


def _load_env_local() -> None:
    path = ROOT / ".env.local"
    if not path.is_file():
        return
    text = path.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        line = line.strip().lstrip("\ufeff")
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            os.environ[key] = value


def _has_nested_key(data: Any, key_path: str) -> bool:
    current = data
    parts = key_path.split(".")
    for part in parts[:-1]:
        if not isinstance(current, Mapping) or part not in current:
            return False
        current = current[part]
    return isinstance(current, Mapping) and parts[-1] in current


def _airsim_reachable(host: str, port: int) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.5)
    try:
        return sock.connect_ex((host, port)) == 0
    finally:
        sock.close()


def _mavlink_heartbeat(config: dict) -> tuple[bool, str, list[str]]:
    from pymavlink import mavutil as _mavutil

    endpoints = candidate_mavlink_endpoints(config)
    last_err = ""
    for endpoint in endpoints:
        connection = None
        try:
            connection = _mavutil.mavlink_connection(endpoint, autoreconnect=False)
            heartbeat = connection.wait_heartbeat(timeout=2.0)
            if heartbeat is not None:
                return True, endpoint, endpoints
        except Exception as exc:
            last_err = str(exc)
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
    return False, last_err, endpoints


def _mavlink_highres_imu(config: dict) -> tuple[bool, str, list[str]]:
    from pymavlink import mavutil as _mavutil

    endpoints = candidate_mavlink_endpoints(config)
    mav_cfg = config.get("control", {}).get("mavlink", {})
    imu_cfg = mav_cfg.get("highres_imu", {})
    interval_us = int(1e6 / max(1.0, float(imu_cfg.get("request_hz", 20.0))))
    last_err = ""
    for endpoint in endpoints:
        connection = None
        try:
            connection = _mavutil.mavlink_connection(endpoint, autoreconnect=False)
            heartbeat = connection.wait_heartbeat(timeout=2.0)
            if heartbeat is None:
                continue
            message_id = getattr(_mavutil.mavlink, "MAVLINK_MSG_ID_HIGHRES_IMU", None)
            if message_id is not None:
                connection.mav.message_interval_send(int(message_id), interval_us)
            deadline = time.monotonic() + 2.5
            while time.monotonic() < deadline:
                message = connection.recv_match(
                    type=["HIGHRES_IMU"],
                    blocking=True,
                    timeout=0.5,
                )
                if message is not None:
                    return True, endpoint, endpoints
            last_err = "timed out waiting for HIGHRES_IMU after requesting stream"
        except Exception as exc:
            last_err = str(exc)
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
    return False, last_err, endpoints


def _normalized_resolution(vision_cfg: dict[str, Any]) -> list[int]:
    resolution = vision_cfg.get("resolution", [640, 360])
    if isinstance(resolution, (list, tuple)) and len(resolution) == 2:
        return [int(resolution[0]), int(resolution[1])]
    return [int(vision_cfg.get("width", 640)), int(vision_cfg.get("height", 360))]


def run_preflight() -> int:
    _load_env_local()
    config = load_config()
    transport = resolve_control_transport(config)

    errors: list[str] = []
    warnings: list[str] = []
    passes: list[str] = []

    required_keys = [
        "algorithm",
        "simulator.colosseum_path",
        "simulator.airsim_port",
        "simulator.rpc_ready_timeout_seconds",
        "control.command_rate_hz",
        "control.max_speed_ms",
    ]
    missing = [key for key in required_keys if not _has_nested_key(config, key)]
    if missing:
        errors.append(f"Missing required config keys: {', '.join(missing)}")
    else:
        passes.append("Required config keys present")

    algo = str(config.get("algorithm", "")).strip()
    if algo:
        registered = list_algorithms()
        if algo not in registered:
            errors.append(
                f"Unknown algorithm {algo!r}; registered: {', '.join(registered) or '(none)'}"
            )
        else:
            passes.append(f"Algorithm {algo!r} is registered")

    sim_cfg = config.get("simulator", {})
    vision_cfg = config.get("vision", {})
    camera_cfg = config.get("camera", {})
    control_cfg = config.get("control", {})
    colosseum_path = str(sim_cfg.get("colosseum_path", "")).strip()
    if not colosseum_path:
        errors.append("simulator.colosseum_path is empty")
    elif not Path(colosseum_path).exists():
        errors.append(f"Unreal executable not found: {colosseum_path}")
    else:
        passes.append("Unreal executable path exists")

    project_path = str(os.environ.get("PROJECT_PATH", "")).strip() or str(
        sim_cfg.get("project_path", "")
    ).strip()
    if not project_path:
        errors.append("PROJECT_PATH not found (.env.local or simulator.project_path)")
    elif not Path(project_path).exists():
        errors.append(f"Project file path not found: {project_path}")
    else:
        passes.append("PROJECT_PATH exists")

    physics_update_hz = float(sim_cfg.get("physics_update_hz", 0.0))
    if abs(physics_update_hz - 120.0) > 1e-6:
        errors.append(
            "simulator.physics_update_hz must be 120.0 for the official spec "
            f"(got {physics_update_hz})"
        )
    else:
        passes.append("Simulator physics update rate is 120 Hz")

    vision_fps = float(vision_cfg.get("fps", 0.0))
    if abs(vision_fps - 30.0) > 1e-6:
        errors.append(f"vision.fps must be 30.0 for the official spec (got {vision_fps})")
    else:
        passes.append("Camera capture rate is 30 Hz")

    camera_resolution = _normalized_resolution(vision_cfg)
    if camera_resolution[0] <= 0 or camera_resolution[1] <= 0:
        errors.append(
            f"vision.resolution must contain positive dimensions (got {camera_resolution})"
        )
    else:
        passes.append(
            f"Camera resolution is configured: {camera_resolution[0]}x{camera_resolution[1]}"
        )

    camera_fov = float(vision_cfg.get("fov_degrees", 0.0))
    if camera_fov <= 0.0:
        errors.append(f"vision.fov_degrees must be positive (got {camera_fov})")
    else:
        passes.append(f"Camera FOV is configured: {camera_fov:.1f} degrees")

    if bool(vision_cfg.get("startup_autotune_enabled", False)):
        errors.append("vision.startup_autotune_enabled must be false for fixed 30 Hz compliance")
    else:
        passes.append("Camera startup auto-tuning is disabled for fixed timing")

    camera_pitch_up = float(camera_cfg.get("pitch_up_degrees", 0.0))
    if abs(camera_pitch_up - 20.0) > 1e-6:
        errors.append(
            f"camera.pitch_up_degrees must be 20.0 for the official spec (got {camera_pitch_up})"
        )
    else:
        passes.append("Front camera upward tilt is 20 degrees")

    pose_offset = camera_cfg.get("pose_offset", [0.35, 0.0, -0.05])
    normalized_pose_offset: list[float] | None = None
    if not isinstance(pose_offset, (list, tuple)) or len(pose_offset) != 3:
        errors.append(f"camera.pose_offset must contain exactly 3 values (got {pose_offset!r})")
    else:
        normalized_pose_offset = [float(value) for value in pose_offset]
        passes.append("Front camera pose offset is configured")

    command_rate_hz = float(control_cfg.get("command_rate_hz", 0.0))
    if not (0.0 < command_rate_hz < 100.0):
        errors.append(
            "control.command_rate_hz must be greater than 0 and less than 100 "
            f"(got {command_rate_hz})"
        )
    else:
        passes.append(f"Command rate limit is in spec (<100 Hz): {command_rate_hz:.1f} Hz")

    latency_cfg = control_cfg.get("latency_tuning", {})
    if bool(latency_cfg.get("enabled", False)):
        errors.append(
            "control.latency_tuning.enabled must be false for fixed command-rate compliance"
        )
    else:
        passes.append("Command-rate auto-tuning is disabled for fixed timing")

    spec_required = bool(sim_cfg.get("specification_required", False))
    spec_path = resolve_specification_path(config)
    if spec_path is not None and spec_path.is_file():
        try:
            spec_snapshot = json.loads(spec_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            errors.append(f"Simulator specification snapshot is invalid JSON: {spec_path} ({exc})")
        else:
            metadata = spec_snapshot.get("metadata", {})
            expected_fingerprint = conformity_fingerprint(config)
            snapshot_fingerprint = str(metadata.get("config_sha256", "")).strip()
            if snapshot_fingerprint:
                if snapshot_fingerprint == expected_fingerprint:
                    passes.append(
                        "Simulator specification snapshot matches the active conformity config"
                    )
                else:
                    errors.append(
                        "Simulator specification snapshot is stale for the current conformity "
                        "config. Run: uv run extract-simulator-specs"
                    )
            else:
                warnings.append(
                    "Simulator specification snapshot has no config fingerprint; "
                    "re-run uv run extract-simulator-specs to record freshness metadata"
                )
            drone_dims = spec_snapshot.get("drone", {}).get("dimensions_m")
            gate_dims = spec_snapshot.get("gate_reference", {}).get("dimensions_m")
            if drone_dims and gate_dims:
                passes.append(f"Simulator specification snapshot present: {spec_path}")
            else:
                errors.append(
                    "Simulator specification snapshot is missing drone/gate dimensions: "
                    f"{spec_path}"
                )
            physics_snapshot = spec_snapshot.get("physics", {})
            async_fixed_timestep_s = float(physics_snapshot.get("async_fixed_timestep_s", 0.0))
            max_substep_delta_s = float(physics_snapshot.get("max_substep_delta_time_s", 0.0))
            target_step_s = 1.0 / 120.0
            if min(
                abs(async_fixed_timestep_s - target_step_s),
                abs(max_substep_delta_s - target_step_s),
            ) <= 1e-4:
                passes.append("Simulator specification snapshot confirms 120 Hz physics timing")
            else:
                errors.append(
                    "Simulator specification snapshot does not show 120 Hz physics timing. "
                    "Run: uv run extract-simulator-specs after updating the Unreal physics settings"
                )
            runtime_camera_spec = metadata.get("runtime_camera_spec")
            if isinstance(runtime_camera_spec, dict):
                snapshot_resolution = list(runtime_camera_spec.get("resolution", []))
                if snapshot_resolution != camera_resolution:
                    errors.append(
                        "Simulator specification snapshot camera resolution does not match the "
                        "active config. Run: uv run extract-simulator-specs"
                    )
                else:
                    passes.append(
                        "Simulator specification snapshot camera resolution matches config"
                    )

                snapshot_fov = float(runtime_camera_spec.get("fov_degrees", 0.0))
                if abs(snapshot_fov - camera_fov) > 1e-6:
                    errors.append(
                        "Simulator specification snapshot camera FOV does not match the active "
                        "config. Run: uv run extract-simulator-specs"
                    )
                else:
                    passes.append("Simulator specification snapshot camera FOV matches config")

                snapshot_pose = list(runtime_camera_spec.get("pose_offset_m", []))
                if normalized_pose_offset is None:
                    errors.append(
                        "camera.pose_offset is invalid, so simulator snapshot camera offset "
                        "could not be validated"
                    )
                elif normalized_pose_offset != snapshot_pose:
                    errors.append(
                        "Simulator specification snapshot camera pose offset does not match the "
                        "active config. Run: uv run extract-simulator-specs"
                    )
                else:
                    passes.append(
                        "Simulator specification snapshot camera pose offset matches config"
                    )
            else:
                warnings.append(
                    "Simulator specification snapshot has no runtime camera metadata; "
                    "re-run uv run extract-simulator-specs to record it"
                )
    elif spec_required:
        errors.append(
            "Simulator specification snapshot is required but missing. "
            "Run: uv run extract-simulator-specs"
        )
    else:
        warnings.append("Simulator specification snapshot not found; dimension checks were skipped")

    if transport == "mavlink":
        heartbeat_ok, detail, endpoints = _mavlink_heartbeat(config)
        require_reachable = bool(
            config.get("preflight", {}).get("require_mavlink_reachable", False)
        )
        if heartbeat_ok:
            passes.append(f"MAVLink HEARTBEAT detected via {detail}")
        else:
            message = (
                "MAVLink HEARTBEAT not detected on any endpoint. "
                f"endpoints={endpoints}. last_err={detail}"
            )
            if require_reachable:
                errors.append(message)
            else:
                warnings.append(f"{message} (warning only before simulator launch)")
        imu_cfg = config.get("control", {}).get("mavlink", {}).get("highres_imu", {})
        if bool(imu_cfg.get("enabled", True)):
            imu_ok, imu_detail, imu_endpoints = _mavlink_highres_imu(config)
            require_imu = bool(imu_cfg.get("require_stream", False))
            if imu_ok:
                passes.append(f"MAVLink HIGHRES_IMU detected via {imu_detail}")
            else:
                message = (
                    "MAVLink HIGHRES_IMU not detected on any endpoint after requesting the stream. "
                    f"endpoints={imu_endpoints}. last_err={imu_detail}"
                )
                if require_imu:
                    errors.append(message)
                else:
                    warnings.append(f"{message} (warning only before simulator launch)")
    else:
        host, port = simulator_endpoint(config)
        require_reachable = bool(config.get("preflight", {}).get("require_airsim_reachable", False))
        if _airsim_reachable(host, port):
            passes.append(f"AirSim RPC reachable at {host}:{port}")
        else:
            message = f"AirSim RPC not reachable at {host}:{port}"
            if require_reachable:
                errors.append(message)
            else:
                warnings.append(f"{message} (warning only before simulator launch)")

    cfg_path = resolve_config_path()
    print(f"== Preflight (config: {cfg_path}) ==")
    for line in passes:
        print(f"[PASS] {line}")
    for line in warnings:
        print(f"[WARN] {line}")
    for line in errors:
        print(f"[FAIL] {line}")

    if errors:
        print("Preflight result: FAILED")
        return 1

    print("Preflight result: OK")
    return 0


def main() -> None:
    raise SystemExit(run_preflight())


if __name__ == "__main__":
    main()
