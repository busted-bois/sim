"""Start UE5 Colosseum (if configured), wait, then run main.py."""

import os
import subprocess
import sys
import time
from pathlib import Path

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


def launch(*, landing_profile: str | None = None, low_end: bool = False) -> None:
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
    airsim_port = sim_cfg.get("airsim_port", 41451)
    delay = sim_cfg.get("startup_delay_seconds", 30)

    if colosseum and Path(colosseum).exists():
        if not project:
            raise SystemExit(
                "PROJECT_PATH is not set to a valid .uproject file.\n"
                "Fix it by either:\n"
                "  1) setting PROJECT_PATH in .env.local, or\n"
                "  2) adding simulator.project_path in sim.config.json, or\n"
                "  3) running: pwsh scripts/fix_project_path.ps1\n"
            )
        print(f"Launching Colosseum: {colosseum}")
        cmd = [colosseum]
        cmd.append(project)
        cmd.append("-game")
        if windowed:
            cmd.extend(["-windowed", f"-resx={res_x}", f"-resy={res_y}"])

        subprocess.Popen(
            cmd,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == "win32" else 0,
        )
        print(f"Waiting {delay}s for simulator to start...")
        time.sleep(delay)
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
    run_low_end = any(arg.strip().lower() == "low-end" for arg in sys.argv[1:])
    launch(low_end=run_low_end)


def main_very_soft() -> None:
    launch(landing_profile="very_soft")


def main_low_end() -> None:
    launch(low_end=True)


if __name__ == "__main__":
    launch()
