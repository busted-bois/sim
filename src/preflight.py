"""Preflight checks for simulator/drone runtime configuration."""

from __future__ import annotations

import os
import socket
import sys
from pathlib import Path

from src.config import load_config, resolve_config_path, simulator_endpoint
from src.control.algorithms import list_algorithms
from src.sim_launch import (
    _guess_ue_416_editor,
    _resolve_configured_maze_editor,
    _resolve_maze_uproject_path,
)

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


def _has_nested_key(data: dict, key_path: str) -> bool:
    current = data
    parts = key_path.split(".")
    for part in parts[:-1]:
        if not isinstance(current, dict) or part not in current:
            return False
        current = current[part]
    return isinstance(current, dict) and parts[-1] in current


def run_preflight(*, maze_mode: bool = False) -> int:
    _load_env_local()
    config = load_config()

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
    if maze_mode:
        algo = str(config.get("maze_algorithm", algo)).strip()
    if algo:
        registered = list_algorithms()
        if algo not in registered:
            errors.append(
                f"Unknown algorithm {algo!r}; registered: {', '.join(registered) or '(none)'}"
            )
        else:
            passes.append(f"Algorithm {algo!r} is registered")

    sim_cfg = config.get("simulator", {})
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

    if maze_mode:
        maze_editor_raw = (
            os.environ.get("MAZE_COLLOSSEUM_PATH", "").strip()
            or str(sim_cfg.get("maze_colosseum_path", "")).strip()
        )
        maze_editor = _resolve_configured_maze_editor(maze_editor_raw) if maze_editor_raw else ""
        if not maze_editor:
            maze_editor = _guess_ue_416_editor()
        if not maze_editor:
            errors.append(
                "Maze editor not found (UE 4.16 UnrealEditor.exe/UE4Editor.exe). "
                "Set MAZE_COLLOSSEUM_PATH or simulator.maze_colosseum_path."
            )
        else:
            passes.append(f"Maze editor exists: {maze_editor}")

        maze_project = _resolve_maze_uproject_path(sim_cfg)
        if not maze_project:
            errors.append(
                "Maze .uproject not found. Run scripts/extract_airsim_maps.ps1 or set "
                "MAZE_PROJECT_PATH / simulator.maze_project_path."
            )
        else:
            passes.append(f"Maze .uproject exists: {maze_project}")

    host, port = simulator_endpoint(config)
    require_reachable = bool(config.get("preflight", {}).get("require_airsim_reachable", False))
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.5)
    reachable = sock.connect_ex((host, port)) == 0
    sock.close()
    if reachable:
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
    maze_mode = any(arg.strip().lower() == "maze" for arg in sys.argv[1:])
    raise SystemExit(run_preflight(maze_mode=maze_mode))


if __name__ == "__main__":
    main()
