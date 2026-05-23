"""Offline integration smoke: unittest + exploration + attitude scripts."""

from __future__ import annotations

import subprocess
import sys


def main() -> int:
    steps: list[tuple[list[str], str]] = [
        ([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-q"], "unittest"),
        ([sys.executable, "-m", "scripts.smoke_exploration"], "exploration"),
        ([sys.executable, "scripts/smoke_attitude_integration.py"], "attitude"),
    ]
    for cmd, label in steps:
        print(f"[integration-smoke] {label}...")
        result = subprocess.run(cmd, check=False)
        if result.returncode != 0:
            print(f"[integration-smoke] FAIL: {label}", file=sys.stderr)
            return result.returncode
    print("[integration-smoke] ALL PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
