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
