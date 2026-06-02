"""Tests for MAVLink prerequisite helpers."""

from __future__ import annotations

import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from src import mavlink_prereq


class TestMavlinkPrereq(unittest.TestCase):
    def test_wants_mavlink_session_from_transport(self) -> None:
        config = {"control": {"transport": "mavlink"}}
        self.assertTrue(mavlink_prereq.wants_mavlink_session(config))

    def test_wsl_mirrored_mode_enabled(self) -> None:
        with patch.object(mavlink_prereq, "wsl_config_path") as path_mock:
            cfg = MagicMock()
            cfg.is_file.return_value = True
            cfg.read_text.return_value = "[wsl2]\nnetworkingMode=mirrored\n"
            path_mock.return_value = cfg
            self.assertTrue(mavlink_prereq.wsl_mirrored_mode_enabled())

    def test_ensure_wsl_mirrored_writes_when_missing(self) -> None:
        with patch.object(mavlink_prereq, "wsl_config_path") as path_mock:
            cfg = MagicMock()
            cfg.is_file.return_value = False
            path_mock.return_value = cfg
            with patch.object(mavlink_prereq, "wsl_mirrored_mode_enabled", return_value=False):
                wrote, note = mavlink_prereq.ensure_wsl_mirrored_config()
        self.assertTrue(wrote)
        self.assertIsNotNone(note)
        cfg.write_text.assert_called_once()

    def test_ensure_wsl_mirrored_no_overwrite_existing(self) -> None:
        with patch.object(mavlink_prereq, "wsl_mirrored_mode_enabled", return_value=False):
            with patch.object(mavlink_prereq, "wsl_config_path") as path_mock:
                cfg = MagicMock()
                cfg.is_file.return_value = True
                path_mock.return_value = cfg
                wrote, note = mavlink_prereq.ensure_wsl_mirrored_config()
        self.assertFalse(wrote)
        self.assertIn("networkingMode=mirrored", note or "")

    def test_run_static_skips_wsl_on_linux(self) -> None:
        config = {"control": {"transport": "mavlink"}}
        with patch.object(mavlink_prereq.sys, "platform", "linux"):
            errors, warnings, _passes = mavlink_prereq.run_static_mavlink_checks(config)
        self.assertFalse(errors)
        self.assertTrue(any("skipped" in w.lower() for w in warnings))

    def test_run_static_px4_missing_fails(self) -> None:
        config = {"control": {"transport": "mavlink"}}
        with patch.object(mavlink_prereq.sys, "platform", "win32"):
            with patch.object(mavlink_prereq, "wsl_available", return_value=(True, "")):
                with patch.object(mavlink_prereq, "px4_wsl_binary_ready", return_value=False):
                    with patch.object(
                        mavlink_prereq, "wsl_mirrored_mode_enabled", return_value=True
                    ):
                        with patch.object(
                            mavlink_prereq,
                            "launch_px4_script_path",
                            return_value=Path("scripts/launch_px4_wsl.sh"),
                        ):
                            with patch.object(Path, "is_file", return_value=True):
                                errors, _w, _p = mavlink_prereq.run_static_mavlink_checks(
                                    config
                                )
        self.assertTrue(any("PX4 binary" in e for e in errors))

    def test_ensure_prerequisites_exits_on_errors(self) -> None:
        config = {"control": {"transport": "mavlink"}}
        with patch.object(mavlink_prereq.sys, "platform", "win32"):
            with patch.object(
                mavlink_prereq,
                "run_static_mavlink_checks",
                return_value=(["fail"], [], []),
            ):
                with self.assertRaises(SystemExit):
                    mavlink_prereq.ensure_mavlink_prerequisites(config)


if __name__ == "__main__":
    unittest.main()
