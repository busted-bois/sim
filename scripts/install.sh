#!/usr/bin/env bash

echo "Setting up venv..."
uv sync
uv run lefthook install

echo "Setup complete."
