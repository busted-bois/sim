#!/usr/bin/env bash
set -euo
(set -o pipefail 2>/dev/null) && set -o pipefail

echo "Setting up venv..."
uv sync
uv run lefthook install

echo "Setup complete."
