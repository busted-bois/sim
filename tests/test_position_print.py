"""Unit tests for position-print CLI."""

from __future__ import annotations

import io
import unittest
from unittest.mock import patch

from src.position_print import (
    _build_parser,
    _PrintConnection,
    _resolve_rate_hz,
    _streaming_mode,
)
from src.position_trace import PositionTraceHealth, PositionTraceSnapshot


class PositionPrintTests(unittest.TestCase):
    def test_resolve_rate_hz_from_args(self) -> None:
        args = _build_parser().parse_args(["--rate-hz", "2"])
        self.assertEqual(_resolve_rate_hz({}, args), 2.0)

    def test_resolve_rate_hz_from_hud_config(self) -> None:
        args = _build_parser().parse_args([])
        cfg = {"control": {"mavlink": {"position_hud": {"update_hz": 12}}}}
        self.assertEqual(_resolve_rate_hz(cfg, args), 12.0)

    def test_streaming_mode_defaults_off(self) -> None:
        args = _build_parser().parse_args([])
        self.assertFalse(_streaming_mode(args))

    def test_streaming_mode_duration(self) -> None:
        args = _build_parser().parse_args(["--duration", "5"])
        self.assertTrue(_streaming_mode(args))

    def test_main_prints_once_by_default(self) -> None:
        from src.position_print import main

        row = (1.0, 1000, 1.0, 2.0, -5.0, 0.0, 0.0, 0.0, 5.0)
        snap = PositionTraceSnapshot(
            health=PositionTraceHealth(1, 0, None, 0.0, 1.0, None, None),
            latest=row,
        )

        class _FakeClient:
            pass

        def provider() -> PositionTraceSnapshot:
            return snap

        with patch("src.position_print.load_config") as load_cfg, patch(
            "src.position_print.apply_low_end_overrides"
        ), patch(
            "src.position_print._resolve_connection",
            return_value=_PrintConnection(
                "airsim", _FakeClient(), provider, "127.0.0.1:41451"
            ),
        ):
            load_cfg.return_value = type("Cfg", (), {"_raw": {}})()
            buf = io.StringIO()
            with patch("sys.stdout", buf):
                rc = main([])
        self.assertEqual(rc, 0)
        self.assertIn("x=1.00", buf.getvalue())

    def test_main_fails_without_samples(self) -> None:
        from src.position_print import main

        empty = PositionTraceSnapshot(
            health=PositionTraceHealth(0, 0, None, None, None, None, None),
            latest=None,
        )

        class _FakeClient:
            pass

        with patch("src.position_print.load_config") as load_cfg, patch(
            "src.position_print.apply_low_end_overrides"
        ), patch(
            "src.position_print._resolve_connection",
            return_value=_PrintConnection(
                "airsim", _FakeClient(), lambda: empty, "127.0.0.1:41451"
            ),
        ), patch("src.position_print._wait_for_snapshot", return_value=None):
            load_cfg.return_value = type("Cfg", (), {"_raw": {}})()
            rc = main(["--wait-seconds", "0"])
        self.assertEqual(rc, 1)

    @patch("src.position_print.time.sleep")
    def test_main_stream_duration(self, _sleep) -> None:
        from src.position_print import main

        row = (1.0, 1000, 1.0, 2.0, -5.0, 0.0, 0.0, 0.0, 5.0)
        snap = PositionTraceSnapshot(
            health=PositionTraceHealth(1, 0, None, 0.0, 1.0, None, None),
            latest=row,
        )
        mono_calls = 0

        def fake_monotonic() -> float:
            nonlocal mono_calls
            mono_calls += 1
            return 0.0 if mono_calls <= 4 else 1.0

        class _FakeClient:
            pass

        with patch("src.position_print.load_config") as load_cfg, patch(
            "src.position_print.apply_low_end_overrides"
        ), patch(
            "src.position_print._resolve_connection",
            return_value=_PrintConnection(
                "airsim", _FakeClient(), lambda: snap, "127.0.0.1:41451"
            ),
        ), patch("src.position_print.time.monotonic", side_effect=fake_monotonic):
            load_cfg.return_value = type("Cfg", (), {"_raw": {}})()
            rc = main(["--duration", "0.05", "--rate-hz", "10"])
        self.assertEqual(rc, 0)

    @patch("src.position_print._airsim_snapshot_provider")
    @patch("src.position_print.first_mavlink_heartbeat_endpoint", return_value=None)
    @patch("src.position_print._airsim_rpc_ready", return_value=True)
    def test_auto_falls_back_to_airsim(
        self, _rpc_ready, _heartbeat, mock_airsim_fn
    ) -> None:
        from src.position_print import _resolve_connection

        row = (0.0, 0, 0.0, 0.0, -3.0, 0.0, 0.0, 0.0, 3.0)
        snap = PositionTraceSnapshot(
            health=PositionTraceHealth(1, 0, None, 0.0, 0.0, None, None),
            latest=row,
        )

        class _FakeClient:
            pass

        mock_airsim_fn.return_value = (_FakeClient(), lambda: snap, "127.0.0.1:41451")
        args = _build_parser().parse_args(["--transport", "auto"])
        conn = _resolve_connection({"control": {"transport": "mavlink"}}, args)
        self.assertEqual(conn.transport, "airsim")


if __name__ == "__main__":
    unittest.main()
