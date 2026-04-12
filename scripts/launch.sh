#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Source .env.local if it exists
[ -f .env.local ] && set -a && source .env.local && set +a

echo "AIGP Drone Challenge - Launch Script"
echo "====================================="
uv run scripts/launch.py "$@"
