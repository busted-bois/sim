#!/usr/bin/env bash
set -euo

echo "Setting up venv..."
uv sync
uv run lefthook install

echo "Setup complete."
