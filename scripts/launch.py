"""Launch script — starts UE5 Colosseum simulator, waits, then runs main.py."""

import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from src.config import load_config  # noqa: E402


def launch() -> None:
    config = load_config()
    sim_cfg = config["simulator"]

    colosseum = sim_cfg.get("colosseum_path", "")
    project = sim_cfg.get("project_path", "")
    windowed = sim_cfg.get("windowed", True)
    res_x = sim_cfg.get("res_x", 1280)
    res_y = sim_cfg.get("res_y", 720)
    airsim_port = sim_cfg.get("airsim_port", 41451)
    delay = sim_cfg.get("startup_delay_seconds", 30)

    if colosseum and Path(colosseum).exists():
        print(f"Launching Colosseum: {colosseum}")
        cmd = [colosseum]
        if project:
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

    print("Starting main.py...")
    subprocess.run(
        [sys.executable, str(ROOT / "main.py")],
        env=env,
    )


if __name__ == "__main__":
    launch()
