#!/usr/bin/env bash
set -euo
(set -o pipefail 2>/dev/null) && set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "AIGP Drone Challenge - Launch Script"
echo "====================================="
uv run python scripts/launch.py "$@"
