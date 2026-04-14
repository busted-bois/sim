"""Launch script — thin wrapper; logic lives in src.sim_launch."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.sim_launch import launch  # noqa: E402

if __name__ == "__main__":
    launch()
