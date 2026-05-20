"""Depth calibration entry point (requires live camera feed)."""

from __future__ import annotations

import sys


def main() -> None:
    print(
        "Depth calibration requires a live camera feed; vision capture is not wired yet.",
        file=sys.stderr,
    )
    raise SystemExit(1)


if __name__ == "__main__":
    main()
