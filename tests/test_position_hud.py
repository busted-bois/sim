"""Unit tests for on-screen position HUD."""

from __future__ import annotations

import time
import unittest
from dataclasses import dataclass, field

from src.position_hud import (
    PositionHudConfig,
    PositionOnScreenHud,
    format_position_hud_param,
    format_tracking_hud_param,
    position_hud_config_from_dict,
    resolve_hud_data_source,
)
from src.position_trace import PositionTraceHealth, PositionTraceSnapshot
from src.tracking.snapshot import TrackingSnapshot
from src.tracking.state import TrackingHealth, TrackingState


@dataclass
class _FakeRpcClient:
    connected: bool = False
    calls: list[tuple[str, str, int]] = field(default_factory=list)

    def confirmConnection(self) -> None:
        self.connected = True

    def simPrintLogMessage(
        self,
        message: str,
        message_param: str = "",
        severity: int = 0,
    ) -> None:
        self.calls.append((message, message_param, severity))


class PositionHudTests(unittest.TestCase):
    def test_format_position_hud_param(self) -> None:
        row = (1.5, 1500, 1.0, -2.0, -5.0, 0.1, 0.2, -0.3, 5.0)
        text = format_position_hud_param(row)
        self.assertIn("t=1.50s", text)
        self.assertIn("x=1.00", text)
        self.assertIn("alt=5.00m", text)
        self.assertIn("vx=0.10", text)

    def test_rpc_tracking_hud_provider_builds_full_snapshot(self) -> None:
        from src.position_hud import RpcTrackingHudProvider

        class _Ori:
            x_val = 0.0
            y_val = 0.0
            z_val = 0.0
            w_val = 1.0

        class _Vec:
            def __init__(self, x: float, y: float, z: float) -> None:
                self.x_val = x
                self.y_val = y
                self.z_val = z

        class _Kin:
            position = _Vec(1.0, 2.0, -3.0)
            linear_velocity = _Vec(0.1, 0.2, 0.3)
            orientation = _Ori()

        class _State:
            timestamp = 99
            kinematics_estimated = _Kin()

        class _Client:
            def getMultirotorState(self):
                return _State()

            def getHighresImuHealth(self):
                return None

        snap = RpcTrackingHudProvider(_Client())()
        assert snap is not None
        text = format_tracking_hud_param(snap)
        self.assertIn("roll=", text)
        self.assertIn("pitch=", text)
        self.assertIn("yaw=", text)
        self.assertIn("vx=0.10", text)

    def test_format_tracking_hud_param(self) -> None:
        state = TrackingState(
            sim_time_ns=12_340_000_000,
            t_s=12.34,
            position_ned=(1.2, -0.5, -4.8),
            velocity_ned=(0.1, 0.0, -0.2),
            attitude_rpy=(0.0, -0.35, 0.31),
            armed=True,
            origin_set=True,
        )
        health = TrackingHealth(
            status="ok",
            reason="tracking active",
            imu_sample_count=100,
            imu_rate_hz=118.0,
            vision_correction_count=2,
            origin_set=True,
            armed=True,
        )
        snap = TrackingSnapshot(
            sim_time_ns=12_340_000_000,
            image_rgb=None,
            position_ned=state.position_ned,
            velocity_ned=state.velocity_ned,
            attitude_rpy=state.attitude_rpy,
            health=health,
            state=state,
        )
        text = format_tracking_hud_param(snap)
        self.assertIn("t=12.34s", text)
        self.assertIn("x=1.20", text)
        self.assertIn("alt=4.80m", text)
        self.assertIn("roll=0.0deg", text)
        self.assertIn("pitch=-20.1deg", text)
        self.assertIn("yaw=17.8deg", text)
        self.assertIn("imu=118Hz", text)
        self.assertIn("status=ok", text)
        self.assertIn("vis=2", text)

    def test_resolve_hud_data_source_defaults_tracking_for_mavlink(self) -> None:
        cfg = {
            "control": {
                "transport": "mavlink",
                "mavlink": {"tracking": {"enabled": True}},
            }
        }
        self.assertEqual(resolve_hud_data_source(cfg), "tracking")

    def test_resolve_hud_data_source_respects_explicit(self) -> None:
        cfg = {
            "control": {
                "transport": "mavlink",
                "mavlink": {"tracking": {"enabled": True}},
            }
        }
        self.assertEqual(resolve_hud_data_source(cfg, explicit="trace"), "trace")

    def test_resolve_hud_data_source_falls_back_on_airsim_transport(self) -> None:
        cfg = {
            "control": {
                "transport": "mavlink",
                "mavlink": {"tracking": {"enabled": True}},
            }
        }
        self.assertEqual(resolve_hud_data_source(cfg, explicit="tracking", transport="airsim"), "trace")

    def test_config_from_dict_uses_tracking_default_when_enabled(self) -> None:
        cfg = position_hud_config_from_dict(
            {"enabled": True},
            config={
                "control": {
                    "transport": "mavlink",
                    "mavlink": {"tracking": {"enabled": True}},
                }
            },
            transport="mavlink",
        )
        self.assertEqual(cfg.data_source, "tracking")

    def test_config_from_dict(self) -> None:
        cfg = position_hud_config_from_dict(
            {
                "enabled": True,
                "update_hz": 20,
                "message_label": "pos",
                "severity": 2,
                "data_source": "tracking",
            }
        )
        self.assertTrue(cfg.enabled)
        self.assertEqual(cfg.update_hz, 20.0)
        self.assertEqual(cfg.message_label, "pos: ")
        self.assertEqual(cfg.severity, 2)
        self.assertEqual(cfg.data_source, "tracking")

    def test_config_invalid_data_source_defaults_trace(self) -> None:
        cfg = position_hud_config_from_dict({"data_source": "invalid"})
        self.assertEqual(cfg.data_source, "trace")

    def test_hud_posts_to_sim_print(self) -> None:
        latest = (2.0, 2000, 0.5, 0.0, -4.0, 0.0, 0.0, 0.0, 4.0)
        health = PositionTraceHealth(
            accepted_count=1,
            rejected_count=0,
            stream_rate_hz=None,
            first_t_s=0.0,
            last_t_s=2.0,
            max_step_m=None,
            last_reject_reason=None,
        )
        snapshot = PositionTraceSnapshot(health=health, latest=latest)
        fake = _FakeRpcClient()

        def provider() -> PositionTraceSnapshot:
            return snapshot

        hud = PositionOnScreenHud(
            host="127.0.0.1",
            port=41451,
            snapshot_provider=provider,
            config=PositionHudConfig(
                enabled=True,
                update_hz=50.0,
                message_label="AIGP pos",
                severity=0,
                data_source="trace",
            ),
            client_factory=lambda _host, _port: fake,
        )
        hud.start()
        deadline = time.time() + 1.0
        while time.time() < deadline and hud.update_count < 1:
            time.sleep(0.02)
        hud.stop()
        self.assertGreaterEqual(hud.update_count, 1)
        self.assertGreaterEqual(len(fake.calls), 1)
        message, param, severity = fake.calls[0]
        self.assertEqual(message, "AIGP pos")
        self.assertIn("t=2.00s", param)
        self.assertEqual(severity, 0)

    def test_hud_posts_tracking_snapshot(self) -> None:
        state = TrackingState(
            sim_time_ns=2_000_000_000,
            t_s=2.0,
            position_ned=(0.5, 0.0, -4.0),
            velocity_ned=(0.0, 0.0, 0.0),
            attitude_rpy=(0.0, 0.0, 0.1),
            armed=True,
            origin_set=True,
        )
        health = TrackingHealth(
            status="ok",
            reason="tracking active",
            imu_sample_count=10,
            imu_rate_hz=50.0,
            vision_correction_count=0,
            origin_set=True,
            armed=True,
        )
        snap = TrackingSnapshot(
            sim_time_ns=2_000_000_000,
            image_rgb=None,
            position_ned=state.position_ned,
            velocity_ned=state.velocity_ned,
            attitude_rpy=state.attitude_rpy,
            health=health,
            state=state,
        )
        fake = _FakeRpcClient()

        def provider() -> TrackingSnapshot:
            return snap

        hud = PositionOnScreenHud(
            host="127.0.0.1",
            port=41451,
            snapshot_provider=provider,
            config=PositionHudConfig(
                enabled=True,
                update_hz=50.0,
                message_label="AIGP track",
                severity=0,
                data_source="tracking",
            ),
            client_factory=lambda _host, _port: fake,
        )
        hud.start()
        deadline = time.time() + 1.0
        while time.time() < deadline and hud.update_count < 1:
            time.sleep(0.02)
        hud.stop()
        self.assertGreaterEqual(hud.update_count, 1)
        message, param, _severity = fake.calls[0]
        self.assertEqual(message, "AIGP track")
        self.assertIn("t=2.00s", param)
        self.assertIn("imu=50Hz", param)

    def test_hud_skips_tracking_without_origin(self) -> None:
        health = TrackingHealth(
            status="waiting_origin",
            reason="awaiting arm",
            imu_sample_count=0,
            imu_rate_hz=None,
            vision_correction_count=0,
            origin_set=False,
            armed=False,
        )
        snap = TrackingSnapshot(
            sim_time_ns=0,
            image_rgb=None,
            position_ned=(0.0, 0.0, 0.0),
            velocity_ned=(0.0, 0.0, 0.0),
            attitude_rpy=(0.0, 0.0, 0.0),
            health=health,
            state=None,
        )
        fake = _FakeRpcClient()
        hud = PositionOnScreenHud(
            host="127.0.0.1",
            port=41451,
            snapshot_provider=lambda: snap,
            config=PositionHudConfig(
                enabled=True,
                update_hz=50.0,
                message_label="AIGP track",
                severity=0,
                data_source="tracking",
            ),
            client_factory=lambda _host, _port: fake,
        )
        hud.start()
        time.sleep(0.1)
        hud.stop()
        self.assertEqual(hud.update_count, 0)
        self.assertEqual(len(fake.calls), 0)

    def test_hud_skips_when_no_snapshot(self) -> None:
        fake = _FakeRpcClient()
        hud = PositionOnScreenHud(
            host="127.0.0.1",
            port=41451,
            snapshot_provider=lambda: None,
            config=PositionHudConfig(
                enabled=True,
                update_hz=50.0,
                message_label="AIGP pos",
                severity=0,
            ),
            client_factory=lambda _host, _port: fake,
        )
        hud.start()
        time.sleep(0.1)
        hud.stop()
        self.assertEqual(hud.update_count, 0)
        self.assertEqual(len(fake.calls), 0)


if __name__ == "__main__":
    unittest.main()
