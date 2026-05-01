"""Update sim.config.json maze paths (used by deploy_maze_sim.ps1)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--editor",
        default="",
        help="UE 4.16 UnrealEditor.exe path, or omit to skip",
    )
    args = parser.parse_args()
    path: Path = args.config
    data = json.loads(path.read_text(encoding="utf-8"))
    sim = data.setdefault("simulator", {})
    sim["maze_project_path"] = "opensrc/AirsimSimulation/VehilceAdvanced.uproject"
    editor = (args.editor or "").strip()
    if editor:
        sim["maze_colosseum_path"] = editor
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
