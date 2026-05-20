#!/usr/bin/env bash
# Launch PX4-SITL configured to bridge to the simulator on the Windows host.
# Requires WSL mirrored networking mode (Windows loopback shared with WSL).
# Called by scripts/dev-mavlink.ps1 — also runnable standalone from inside WSL.
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [ ! -d "$PX4_DIR" ]; then
  echo "PX4 source not found at $PX4_DIR. Set PX4_DIR or clone PX4-Autopilot." >&2
  exit 1
fi

cd "$PX4_DIR"
exec env PX4_SIM_HOST_ADDR=127.0.0.1 make px4_sitl none_iris
