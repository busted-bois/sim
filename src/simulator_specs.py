from __future__ import annotations

import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from src.competition_specs import competition_validation_from_config
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


PHYSICS_120_HZ_STEP_S = 1.0 / 120.0
PHYSICS_120_HZ_TOLERANCE_S = 1e-4


def physics_snapshot_matches_120hz(physics: Any) -> bool:
    if not isinstance(physics, dict):
        return False
    async_fixed = float(physics.get("async_fixed_timestep_s", 0.0))
    max_substep = float(physics.get("max_substep_delta_time_s", 0.0))
    return (
        min(
            abs(async_fixed - PHYSICS_120_HZ_STEP_S),
            abs(max_substep - PHYSICS_120_HZ_STEP_S),
        )
        <= PHYSICS_120_HZ_TOLERANCE_S
    )


def vision_resolution_from_config(raw: dict[str, Any] | Any) -> list[int]:
    data = _to_plain_mapping(raw)
    vision_cfg = data.get("vision", {})
    resolution = vision_cfg.get("resolution", [640, 360])
    if isinstance(resolution, (list, tuple)) and len(resolution) == 2:
        return [int(resolution[0]), int(resolution[1])]
    return [int(vision_cfg.get("width", 640)), int(vision_cfg.get("height", 360))]


def camera_pose_offset_from_config(raw: dict[str, Any] | Any) -> list[float] | None:
    data = _to_plain_mapping(raw)
    pose_offset = data.get("camera", {}).get("pose_offset", [0.35, 0.0, -0.05])
    if not isinstance(pose_offset, (list, tuple)) or len(pose_offset) != 3:
        return None
    return [float(value) for value in pose_offset]


def specification_snapshot_validation(
    config: dict[str, Any] | Any,
    spec_snapshot: dict[str, Any],
    *,
    spec_path: Path | str,
    camera_resolution: list[int],
    camera_fov: float,
    normalized_pose_offset: list[float] | None,
) -> tuple[list[str], list[str], list[str]]:
    errors: list[str] = []
    passes: list[str] = []
    warnings: list[str] = []
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
    if physics_snapshot_matches_120hz(physics_snapshot):
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
    comp_errs, comp_passes, comp_warns = competition_validation_from_config(config, spec_snapshot)
    errors.extend(comp_errs)
    passes.extend(comp_passes)
    warnings.extend(comp_warns)
    return errors, passes, warnings


def assert_specification_snapshot_if_required(config: dict[str, Any] | Any) -> None:
    sim = _to_plain_mapping(config).get("simulator", {})
    if not bool(sim.get("specification_required", False)):
        return
    spec_path = resolve_specification_path(config)
    if spec_path is None or not spec_path.is_file():
        raise SystemExit(
            "simulator.specification_required is true but the specification snapshot is "
            "missing.\nRun: uv run extract-simulator-specs"
        )
    try:
        spec_snapshot = json.loads(spec_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Simulator specification snapshot is invalid JSON: {spec_path} ({exc})")
    cam_res = vision_resolution_from_config(config)
    cam_fov = float(_to_plain_mapping(config).get("vision", {}).get("fov_degrees", 0.0))
    pose = camera_pose_offset_from_config(config)
    errs, _, _ = specification_snapshot_validation(
        config,
        spec_snapshot,
        spec_path=spec_path,
        camera_resolution=cam_res,
        camera_fov=cam_fov,
        normalized_pose_offset=pose,
    )
    if errs:
        raise SystemExit(
            "specification snapshot check failed:\n"
            + "\n".join(f"  - {item}" for item in errs)
        )


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


def main_verify_physics_metadata() -> None:
    _load_env_local()
    cfg = load_config()
    path = resolve_specification_path(cfg)
    if path is None or not path.is_file():
        raise SystemExit("simulator.specification_path missing or file not found.")
    spec = json.loads(path.read_text(encoding="utf-8"))
    phys = spec.get("physics", {})
    if not physics_snapshot_matches_120hz(phys):
        raise SystemExit(
            f"Snapshot at {path} does not document 120 Hz physics "
            f"(expected async/substep ~{PHYSICS_120_HZ_STEP_S:.9f}s)."
        )
    print(
        f"OK: {path} documents 120 Hz physics (UE step not exposed on RPC). "
        "Run: uv run extract-simulator-specs after Unreal edits."
    )


if __name__ == "__main__":
    main()
