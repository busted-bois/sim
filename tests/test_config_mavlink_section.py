"""Validate sim.config.json MAVLink section after load."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

from src.config import load_config

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "sim.config.json"


class ConfigMavlinkSectionTests(unittest.TestCase):
    def test_loaded_config_has_mavlink_endpoint_and_bridge(self) -> None:
        config = load_config()
        mav = config.get("control", {}).get("mavlink", {})
        self.assertIsInstance(mav, dict)
        self.assertIn("endpoint", mav)
        self.assertIn("bridge_profile", mav)
        self.assertIn("highres_imu", mav)
        self.assertIn("attitude", mav)
        self.assertEqual(
            str(mav.get("endpoint", "")).strip(),
            "udpin:0.0.0.0:14550",
        )

    def test_duplicate_json_keys_rejected_at_parse(self) -> None:
        duplicates: list[str] = []

        def hook(pairs: list[tuple[str, object]]) -> dict:
            keys = [k for k, _ in pairs]
            seen: set[str] = set()
            for key in keys:
                if key in seen:
                    duplicates.append(key)
                seen.add(key)
            return dict(pairs)

        with CONFIG_PATH.open(encoding="utf-8") as handle:
            json.load(handle, object_pairs_hook=hook)
        self.assertEqual(duplicates, [])


if __name__ == "__main__":
    unittest.main()
