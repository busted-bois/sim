from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Any

from src.config import load_config

ROOT = Path(__file__).resolve().parent.parent
UNREAL_DUMP_SCRIPT = ROOT / "scripts" / "unreal_dump_simulator_specs.py"


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


def resolve_specification_path(config: dict[str, Any]) -> Path | None:
    sim_cfg = config.get("simulator", {})
    raw_path = str(sim_cfg.get("specification_path", "")).strip()
    if not raw_path:
        return None
    path = Path(raw_path)
    if not path.is_absolute():
        path = ROOT / path
    return path


def _as_unreal_path(path: Path | str) -> str:
    return str(path).replace("\\", "/")


def load_specification_snapshot(config: dict[str, Any]) -> dict[str, Any] | None:
    path = resolve_specification_path(config)
    if path is None or not path.is_file():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def extract_specification_snapshot(config: dict[str, Any]) -> Path:
    _load_env_local()
    sim_cfg = config.get("simulator", {})
    output_path = resolve_specification_path(config)
    if output_path is None:
        raise FileNotFoundError("simulator.specification_path is not configured")

    project_path = os.environ.get("PROJECT_PATH", "").strip() or str(
        sim_cfg.get("project_path", "")
    ).strip()
    if not project_path:
        raise FileNotFoundError("PROJECT_PATH is not configured")

    editor_path = str(sim_cfg.get("colosseum_path", "")).strip()
    if not editor_path:
        raise FileNotFoundError("simulator.colosseum_path is not configured")
    editor_cmd = Path(editor_path).with_name("UnrealEditor-Cmd.exe")
    if not editor_cmd.is_file():
        raise FileNotFoundError(f"UnrealEditor-Cmd.exe not found next to {editor_path}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["CODEX_SIM_SPEC_OUT"] = _as_unreal_path(output_path)
    env["CODEX_SIM_SPEC_MAP"] = str(
        sim_cfg.get("map_asset", "/Game/FlyingCPP/Maps/FlyingExampleMapV2")
    )
    env["CODEX_SIM_SPEC_PAWN_ASSET"] = str(
        sim_cfg.get("pawn_asset", "/AirSim/Blueprints/BP_FlyingPawn")
    )
    env["CODEX_SIM_SPEC_GATE_TOKENS"] = str(
        sim_cfg.get("gate_search_tokens", "gate,ring,torus")
    )

    subprocess.run(
        [
            str(editor_cmd),
            project_path,
            "-run=pythonscript",
            f"-script={_as_unreal_path(UNREAL_DUMP_SCRIPT)}",
            "-NullRHI",
            "-Unattended",
            "-NoSplash",
            "-NoSound",
            "-stdout",
            "-FullStdOutLogOutput",
        ],
        check=True,
        cwd=Path(project_path).resolve().parent,
        env=env,
    )
    if not output_path.is_file():
        raise FileNotFoundError(
            f"Expected simulator specification snapshot was not produced: {output_path}"
        )
    return output_path


def main() -> None:
    config = load_config()
    output_path = extract_specification_snapshot(config)
    print(f"Simulator specification snapshot written to {output_path}")


if __name__ == "__main__":
    main()
