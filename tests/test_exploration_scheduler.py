"""Unit tests for exploration scheduler (panorama, legs, altitude layers)."""

from __future__ import annotations

import math
import unittest

from src.control.exploration.scheduler import (
    ExplorationMode,
    ExplorationScheduler,
    ExplorationSettings,
    WanderTickInput,
    parse_exploration_settings,
    vertical_depth_bias_vz,
    vz_toward_altitude_hold,
)


def _tick(
    sched: ExplorationScheduler,
    *,
    now_s: float,
    yaw_rad: float = 0.0,
    z_ned: float = -5.0,
    fwd_speed: float = 1.5,
    yaw_rate_deg_s: float = 0.0,
    dt_s: float = 1.0 / 6.0,
    upper: float | None = None,
    lower: float | None = None,
    defer_panorama: bool = False,
):
    return sched.tick(
        WanderTickInput(
            now_s=now_s,
            dt_s=dt_s,
            yaw_rad=yaw_rad,
            z_ned=z_ned,
            fwd_speed=fwd_speed,
            yaw_rate_deg_s=yaw_rate_deg_s,
            cos_yaw=math.cos(yaw_rad),
            sin_yaw=math.sin(yaw_rad),
            base_vz=0.0,
            upper_clearance=upper,
            lower_clearance=lower,
            defer_panorama=defer_panorama,
        )
    )


class ExplorationSchedulerTests(unittest.TestCase):
    def test_vz_toward_altitude_hold(self) -> None:
        self.assertEqual(vz_toward_altitude_hold(-6.0, -5.0), 0.5)
        self.assertEqual(vz_toward_altitude_hold(-4.0, -5.0), -0.5)
        self.assertEqual(vz_toward_altitude_hold(-5.0, -5.0), 0.0)

    def test_parse_exploration_settings_defaults_layers(self) -> None:
        s = parse_exploration_settings(None, hold_altitude_m=5.0, max_altitude_m=50.0)
        self.assertEqual(s.altitude_layers_m, (5.0,))
        self.assertFalse(s.legacy_scan_enabled)

    def test_parse_legacy_scan_enabled_when_panorama_off(self) -> None:
        s = parse_exploration_settings(
            {"panorama_enabled": False},
            hold_altitude_m=5.0,
            max_altitude_m=50.0,
        )
        self.assertTrue(s.legacy_scan_enabled)

    def test_vertical_depth_bias_descends_when_lower_more_open(self) -> None:
        vz = vertical_depth_bias_vz(10.0, 20.0, gain=0.4, enabled=True)
        self.assertGreater(vz, 0.0)

    def test_vertical_depth_bias_climbs_when_upper_more_open(self) -> None:
        vz = vertical_depth_bias_vz(20.0, 10.0, gain=0.4, enabled=True)
        self.assertLess(vz, 0.0)

    def test_panorama_starts_after_interval(self) -> None:
        settings = ExplorationSettings(
            panorama_enabled=True,
            panorama_interval_s=5.0,
            leg_enabled=False,
            altitude_layers_m=(5.0,),
        )
        sched = ExplorationScheduler(
            settings, start_s=0.0, initial_yaw_rad=0.0, initial_z_ned=-5.0
        )
        out = _tick(sched, now_s=4.0, z_ned=-5.0)
        self.assertNotEqual(out.mode, ExplorationMode.PANORAMA_360)
        out = _tick(sched, now_s=5.1, z_ned=-5.0)
        self.assertEqual(out.mode, ExplorationMode.PANORAMA_360)

    def test_panorama_completes_after_measured_yaw(self) -> None:
        settings = ExplorationSettings(
            panorama_enabled=True,
            panorama_interval_s=1.0,
            leg_enabled=False,
            altitude_layers_m=(5.0,),
        )
        sched = ExplorationScheduler(
            settings, start_s=0.0, initial_yaw_rad=0.0, initial_z_ned=-5.0
        )
        _tick(sched, now_s=1.0, z_ned=-5.0, yaw_rad=0.0)
        yaw = 0.0
        out = _tick(sched, now_s=1.1, z_ned=-5.0, yaw_rad=0.0)
        self.assertEqual(out.mode, ExplorationMode.PANORAMA_360)
        for _ in range(40):
            yaw += math.radians(10.0)
            out = _tick(sched, now_s=1.2, z_ned=-5.0, yaw_rad=yaw)
            if "done" in out.label:
                break
        self.assertIn("done", out.label)
        self.assertEqual(out.mode, ExplorationMode.WANDER)

    def test_defer_panorama_blocks_start(self) -> None:
        settings = ExplorationSettings(
            panorama_enabled=True,
            panorama_interval_s=1.0,
            leg_enabled=False,
            altitude_layers_m=(5.0,),
        )
        sched = ExplorationScheduler(
            settings, start_s=0.0, initial_yaw_rad=0.0, initial_z_ned=-5.0
        )
        out = _tick(sched, now_s=2.0, z_ned=-5.0, defer_panorama=True)
        self.assertNotEqual(out.mode, ExplorationMode.PANORAMA_360)

    def test_leg_turn_advances_heading(self) -> None:
        settings = ExplorationSettings(
            panorama_enabled=False,
            leg_enabled=True,
            leg_duration_s=1.0,
            leg_turn_deg=90.0,
            altitude_layers_m=(5.0,),
        )
        sched = ExplorationScheduler(
            settings, start_s=0.0, initial_yaw_rad=0.0, initial_z_ned=-5.0
        )
        out = _tick(sched, now_s=1.5, z_ned=-5.0)
        self.assertEqual(out.mode, ExplorationMode.LEG_TURN)
        turn_target = math.pi / 2.0
        out = _tick(sched, now_s=2.0, yaw_rad=turn_target, z_ned=-5.0)
        self.assertEqual(out.mode, ExplorationMode.WANDER)

    def test_altitude_layer_cycles(self) -> None:
        settings = ExplorationSettings(
            panorama_enabled=False,
            leg_enabled=False,
            altitude_layers_m=(3.5, 5.0, 6.5),
            altitude_layer_dwell_s=10.0,
        )
        sched = ExplorationScheduler(
            settings, start_s=0.0, initial_yaw_rad=0.0, initial_z_ned=-5.0
        )
        self.assertAlmostEqual(sched.z_hold_ned, -3.5)
        _tick(sched, now_s=11.0, z_ned=-5.0)
        self.assertAlmostEqual(sched.z_hold_ned, -5.0)
        _tick(sched, now_s=22.0, z_ned=-5.0)
        self.assertAlmostEqual(sched.z_hold_ned, -6.5)

    def test_reset_panorama_timer_cancels_active_panorama(self) -> None:
        settings = ExplorationSettings(
            panorama_enabled=True,
            panorama_interval_s=1.0,
            leg_enabled=False,
            altitude_layers_m=(5.0,),
        )
        sched = ExplorationScheduler(
            settings, start_s=0.0, initial_yaw_rad=0.0, initial_z_ned=-5.0
        )
        _tick(sched, now_s=1.0, z_ned=-5.0)
        self.assertEqual(_tick(sched, now_s=1.1, z_ned=-5.0).mode, ExplorationMode.PANORAMA_360)
        sched.reset_panorama_timer(2.0)
        out = _tick(sched, now_s=2.1, z_ned=-5.0)
        self.assertEqual(out.mode, ExplorationMode.WANDER)


if __name__ == "__main__":
    unittest.main()
