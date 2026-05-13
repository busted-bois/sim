from __future__ import annotations

import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
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


def _to_plain_mapping(config: dict[str, Any] | Any) -> dict[str, Any]:
    if hasattr(config, "_raw"):
        return dict(getattr(config, "_raw"))
    return dict(config)


def conformity_fingerprint_payload(config: dict[str, Any] | Any) -> dict[str, Any]:
    raw = _to_plain_mapping(config)
    sim_cfg = raw.get("simulator", {})
    vision_cfg = raw.get("vision", {})
    control_cfg = raw.get("control", {})
    latency_cfg = control_cfg.get("latency_tuning", {})
    camera_cfg = raw.get("camera", {})
    resolution = vision_cfg.get("resolution", [640, 360])
    if isinstance(resolution, (list, tuple)) and len(resolution) == 2:
        normalized_resolution = [int(resolution[0]), int(resolution[1])]
    else:
        normalized_resolution = [
            int(vision_cfg.get("width", 640)),
            int(vision_cfg.get("height", 360)),
        ]
    pose_offset = camera_cfg.get("pose_offset", [0.35, 0.0, -0.05])
    normalized_pose_offset = [float(value) for value in pose_offset[:3]]
    return {
        "simulator": {
            "map_asset": str(sim_cfg.get("map_asset", "")),
            "pawn_asset": str(sim_cfg.get("pawn_asset", "")),
            "gate_search_tokens": str(sim_cfg.get("gate_search_tokens", "")),
            "physics_update_hz": float(sim_cfg.get("physics_update_hz", 0.0)),
            "specification_required": bool(sim_cfg.get("specification_required", False)),
        },
        "vision": {
            "camera_name": str(vision_cfg.get("camera_name", "0")),
            "fps": float(vision_cfg.get("fps", 0.0)),
            "resolution": normalized_resolution,
            "fov_degrees": float(vision_cfg.get("fov_degrees", 0.0)),
            "strict_timing": bool(vision_cfg.get("strict_timing", False)),
            "startup_autotune_enabled": bool(vision_cfg.get("startup_autotune_enabled", False)),
        },
        "control": {
            "command_rate_hz": float(control_cfg.get("command_rate_hz", 0.0)),
            "latency_tuning_enabled": bool(latency_cfg.get("enabled", False)),
        },
        "camera": {
            "pose_offset_m": normalized_pose_offset,
            "pitch_up_degrees": float(camera_cfg.get("pitch_up_degrees", 0.0)),
            "roll_degrees": float(camera_cfg.get("roll_degrees", 0.0)),
            "yaw_degrees": float(camera_cfg.get("yaw_degrees", 0.0)),
        },
    }


def conformity_fingerprint(config: dict[str, Any] | Any) -> str:
    payload = conformity_fingerprint_payload(config)
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


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
    env["CODEX_SIM_SPEC_PROJECT_PATH"] = _as_unreal_path(project_path)
    env["CODEX_SIM_SPEC_EXTRACTED_AT_UTC"] = datetime.now(timezone.utc).isoformat()
    env["CODEX_SIM_SPEC_CONFIG_SHA256"] = conformity_fingerprint(config)
    fingerprint_payload = conformity_fingerprint_payload(config)
    env["CODEX_SIM_SPEC_CAMERA_RUNTIME"] = json.dumps(
        {
            "camera_name": fingerprint_payload["vision"]["camera_name"],
            "resolution": fingerprint_payload["vision"]["resolution"],
            "fov_degrees": fingerprint_payload["vision"]["fov_degrees"],
            "fps": fingerprint_payload["vision"]["fps"],
            "pose_offset_m": fingerprint_payload["camera"]["pose_offset_m"],
            "pitch_up_degrees": fingerprint_payload["camera"]["pitch_up_degrees"],
            "roll_degrees": fingerprint_payload["camera"]["roll_degrees"],
            "yaw_degrees": fingerprint_payload["camera"]["yaw_degrees"],
        },
        sort_keys=True,
    )

    completed = subprocess.run(
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
        cwd=Path(project_path).resolve().parent,
        env=env,
    )
    if not output_path.is_file():
        raise FileNotFoundError(
            f"Expected simulator specification snapshot was not produced: {output_path}"
        )
    snapshot = json.loads(output_path.read_text(encoding="utf-8"))
    if completed.returncode != 0 and (
        not snapshot.get("drone") or not snapshot.get("gate_reference")
    ):
        raise subprocess.CalledProcessError(completed.returncode, completed.args)
    return output_path


def main() -> None:
    config = load_config()
    output_path = extract_specification_snapshot(config)
    print(f"Simulator specification snapshot written to {output_path}")


if __name__ == "__main__":
    main()
