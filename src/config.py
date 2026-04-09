"""Configuration loader — reads sim.config.json from project root."""

import json
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).parent.parent / "sim.config.json"


def load_config(path: str | Path | None = None) -> dict:
    """Load config from JSON file. Returns plain dict."""
    config_path = Path(path) if path else DEFAULT_CONFIG_PATH
    with open(config_path) as f:
        return json.load(f)
