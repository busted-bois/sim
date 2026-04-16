"""Start UE5 Colosseum (if configured), wait, then run main.py."""

import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def _airsim_settings_path() -> Path:
    if sys.platform == "win32":
        documents = Path.home() / "Documents"
    else:
        documents = Path.home() / "Documents"
    return documents / "AirSim" / "settings.json"


def _deep_merge_dict(base: dict, update: dict) -> dict:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_dict(base[key], value)
        else:
            base[key] = value
    return base


def _normalize_view_mode(view_mode: str) -> str:
    mode = view_mode.strip().lower()
    if mode in {"3rd-person", "third-person", "flywithme"}:
        return "FlyWithMe"
    return "Fpv"


def _ensure_camera_settings(
    airsim_port: int, view_mode: str, *, corner_chase_pip: bool
) -> None:
    settings_path = _airsim_settings_path()
    settings_path.parent.mkdir(parents=True, exist_ok=True)

    settings: dict = {}
    if settings_path.is_file():
        try:
            settings = json.loads(settings_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            print(f"Warning: invalid AirSim settings JSON at {settings_path}; rewriting file.")

    # Cleanup legacy camera block from earlier launcher versions if present.
    vehicles = settings.get("Vehicles")
    if isinstance(vehicles, dict):
        drone1 = vehicles.get("Drone1")
        if isinstance(drone1, dict):
            keys = set(drone1.keys())
            cameras = drone1.get("Cameras")
            if (
                keys <= {"VehicleType", "Cameras"}
                and isinstance(cameras, dict)
                and {"0", "front_center"}.issubset(set(cameras.keys()))
            ):
                vehicles.pop("Drone1", None)
        if not vehicles:
            settings.pop("Vehicles", None)

    # Keep settings minimal so we do not accidentally override map-specific vehicle configs.
    normalized_view_mode = _normalize_view_mode(view_mode)
    required_settings = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ViewMode": normalized_view_mode,
        "ApiServerPort": int(airsim_port),
        "Recording": {
            "Cameras": [
                {"CameraName": "0", "ImageType": 0, "PixelsAsFloat": False, "Compress": False}
            ]
        },
        # Corner Chase PIP: only for `uv run sim 3rd-person` (FlyWithMe).
        # FPV / low-end leave it off.
        "SubWindows": [
            {
                "WindowID": 0,
                "ImageType": 0,
                "CameraName": "Chase",
                "External": True,
                "Visible": bool(corner_chase_pip),
            }
        ],
    }
    # FPV only: extra chase pull-back for the (hidden) external
    # camera / prior PiP tuning.
    # `uv run sim 3rd-person` (FlyWithMe) uses stock AirSim
    # CameraDirector defaults - no FollowDistance override.
    if normalized_view_mode == "Fpv":
        required_settings["CameraDirector"] = {"FollowDistance": -50.0}

    merged = _deep_merge_dict(settings, required_settings)
    if normalized_view_mode == "FlyWithMe":
        merged.pop("CameraDirector", None)

    settings_path.write_text(json.dumps(merged, indent=2), encoding="utf-8")
    print(f"Configured AirSim ViewMode={normalized_view_mode} in {settings_path}")


def _is_port_open(host: str, port: int, timeout_s: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _wait_for_airsim_rpc(host: str, port: int, timeout_s: float) -> bool:
    deadline = time.time() + max(1.0, timeout_s)
    while time.time() < deadline:
        if _is_port_open(host, port):
            return True
        time.sleep(1.0)
    return False


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


def _resolve_project_path(sim_cfg: dict) -> str:
    project = os.environ.get("PROJECT_PATH", "").strip()
    # Trust a non-empty PROJECT_PATH; UE will error clearly if it's wrong.
    if project:
        return project

    config_project = str(sim_cfg.get("project_path", "")).strip()
    if config_project:
        os.environ["PROJECT_PATH"] = config_project
        return config_project

    # Best-effort auto-fix on Windows using the existing helper script.
    fix_script = ROOT / "scripts" / "fix_project_path.ps1"
    if sys.platform == "win32" and fix_script.is_file():
        subprocess.run(
            [
                "powershell",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(fix_script),
            ],
            check=False,
        )
        _load_env_local()
        project = os.environ.get("PROJECT_PATH", "").strip()
        if project:
            return project

    return ""


def launch(
    *,
    landing_profile: str | None = None,
    low_end: bool = False,
    view_mode: str = "Fpv",
    corner_chase_pip: bool = False,
) -> None:
    _load_env_local()

    # Import after env so PROJECT_PATH is visible to any import-time reads
    from src.config import load_config

    config = load_config()
    sim_cfg = config["simulator"]

    colosseum = sim_cfg.get("colosseum_path", "")
    project = _resolve_project_path(sim_cfg)
    windowed = sim_cfg.get("windowed", True)
    res_x = sim_cfg.get("res_x", 1280)
    res_y = sim_cfg.get("res_y", 720)
    if low_end:
        low_end_cfg = config.get("low_end_profile", {})
        windowed = True
        res_x = int(low_end_cfg.get("sim_res_x", 854))
        res_y = int(low_end_cfg.get("sim_res_y", 480))
    airsim_port = sim_cfg.get("airsim_port", 41451)
    delay = sim_cfg.get("startup_delay_seconds", 30)
    _ensure_camera_settings(airsim_port, view_mode, corner_chase_pip=corner_chase_pip)

    if colosseum and Path(colosseum).exists():
        if not project:
            raise SystemExit(
                "PROJECT_PATH is not set to a valid .uproject file.\n"
                "Fix it by either:\n"
                "  1) setting PROJECT_PATH in .env.local, or\n"
                "  2) adding simulator.project_path in sim.config.json, or\n"
                "  3) running: pwsh scripts/fix_project_path.ps1\n"
            )
        if low_end:
            print(f"Launching Colosseum (low-end): {colosseum}")
        else:
            print(f"Launching Colosseum: {colosseum}")
        cmd = [colosseum]
        cmd.append(project)
        cmd.append("-game")
        cmd.append(f"-settings={_airsim_settings_path()}")
        if windowed:
            cmd.extend(["-windowed", f"-resx={res_x}", f"-resy={res_y}"])

        subprocess.Popen(
            cmd,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == "win32" else 0,
        )
        print(f"Waiting {delay}s for simulator to start...")
        time.sleep(delay)
        wait_timeout_s = max(15.0, float(sim_cfg.get("rpc_ready_timeout_seconds", 120.0)))
        if _wait_for_airsim_rpc("127.0.0.1", int(airsim_port), wait_timeout_s):
            print(f"AirSim RPC is ready on 127.0.0.1:{airsim_port}")
        else:
            raise SystemExit(
                f"AirSim RPC did not become ready on 127.0.0.1:{airsim_port} "
                f"within {wait_timeout_s:.0f}s. Ensure Unreal finished loading the map."
            )
    else:
        print(f"Colosseum not found at '{colosseum}', skipping simulator launch.")

    env = os.environ.copy()
    env["AIRSIM_PORT"] = str(airsim_port)
    if landing_profile:
        env["AIGP_LANDING_PROFILE"] = landing_profile
    if low_end:
        env["AIGP_LOW_END"] = "1"

    print("Starting main.py...")
    subprocess.run(
        [sys.executable, str(ROOT / "main.py")],
        env=env,
    )


def main() -> None:
    normalized_args = {arg.strip().lower() for arg in sys.argv[1:]}
    run_low_end = "low-end" in normalized_args
    run_third_person = "3rd-person" in normalized_args or "third-person" in normalized_args
    view_mode = "FlyWithMe" if run_third_person else "Fpv"
    launch(
        low_end=run_low_end,
        view_mode=view_mode,
        corner_chase_pip=run_third_person,
    )


def main_very_soft() -> None:
    launch(landing_profile="very_soft")


def main_low_end() -> None:
    launch(low_end=True, corner_chase_pip=False)


if __name__ == "__main__":
    launch()
