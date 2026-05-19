#!/usr/bin/env bash
# Stop stale PX4-SITL processes in WSL (instance 0 / make px4_sitl).
# Used by scripts/dev-mavlink.ps1 and scripts/launch_px4_wsl.sh.
set -euo pipefail

PX4_MATCH='px4_sitl_default/bin/px4'
MAKE_MATCH='make px4_sitl'

_px4_running() {
  pgrep -f "$PX4_MATCH" >/dev/null 2>&1
}

if ! _px4_running && ! pgrep -f "$MAKE_MATCH" >/dev/null 2>&1; then
  exit 0
fi

pkill -INT -f "$PX4_MATCH" 2>/dev/null || true
pkill -INT -f "$MAKE_MATCH" 2>/dev/null || true

for _ in $(seq 1 10); do
  if ! _px4_running && ! pgrep -f "$MAKE_MATCH" >/dev/null 2>&1; then
    exit 0
  fi
  sleep 0.5
done

pkill -KILL -f "$PX4_MATCH" 2>/dev/null || true
pkill -KILL -f "$MAKE_MATCH" 2>/dev/null || true
sleep 0.5

if _px4_running || pgrep -f "$MAKE_MATCH" >/dev/null 2>&1; then
  echo "stop_px4_wsl: PX4-SITL still running after SIGKILL:" >&2
  pgrep -af "$PX4_MATCH" 2>/dev/null || true
  pgrep -af "$MAKE_MATCH" 2>/dev/null || true
  exit 1
fi

exit 0
