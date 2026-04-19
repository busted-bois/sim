"""Launch script — thin wrapper; logic lives in src.sim_launch."""

import importlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main() -> None:
    sim_launch = importlib.import_module("src.sim_launch")
    sim_launch.launch()


if __name__ == "__main__":
    main()
