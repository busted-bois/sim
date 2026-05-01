"""autonomous_explore: reactive monocular-depth exploration. No waypoints.

Each control tick the drone:
  1. reads its forward camera,
  2. runs MiDaS to get an inverse-depth map,
  3. crops a horizontal eye-level band (ignores floor/ceiling),
  4. splits the band into N vertical columns,
  5. picks the column with the most free space,
  6. yaws toward that column and modulates forward speed by how clear the
     center of the frame is,
  7. brakes (yaw-only) if the center column is significantly more blocked
     than the rest of the frame.

Inspired by:
  Michels, Saxena & Ng (2005) "High Speed Obstacle Avoidance using
  Monocular Vision and Reinforcement Learning."
  Bipin, Duggal & Madhava Krishna (2014) "Autonomous navigation of generic
  monocular quadcopter in natural environment."
  Alvarez, Alvarado, Rojas, Scaramuzza (2016) "Collision Avoidance for
  Quadrotors with a Monocular Camera."

MiDaS small outputs disparity-like inverse depth (high value = close
obstacle), so column scores are inverted before comparison. Set
`autonomous_explore.inverse_depth=false` if you swap in a true-depth model.
"""

from __future__ import annotations

import math
import time

import numpy as np

import airsim
from src.control.algorithms import Algorithm, register
from src.vision.processing import get_depth_info


def _yaw_from_orientation(orientation) -> float:
    x = float(orientation.x_val)
    y = float(orientation.y_val)
    z = float(orientation.z_val)
    w = float(orientation.w_val)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@register("autonomous_explore")
class AutonomousExplore(Algorithm):
    def run(self, client: airsim.MultirotorClient) -> None:
        cfg = self._config.get("autonomous_explore", {})
        control = self._config.get("control", {})

        cap = float(control.get("max_speed_ms", 10.0))
        max_v = _clamp(float(cfg.get("max_speed_ms", 2.5)), 0.2, cap)
        cruise_v = _clamp(float(cfg.get("cruise_speed_ms", 1.5)), 0.0, max_v)
        rate_hz = _clamp(float(cfg.get("rate_hz", 6.0)), 2.0, 20.0)
        duration_s = _clamp(float(cfg.get("duration_s", 60.0)), 5.0, 300.0)

        n_cols = int(_clamp(int(cfg.get("num_columns", 5)), 3, 11))
        eye_band_top = _clamp(float(cfg.get("eye_band_top_frac", 0.30)), 0.0, 0.9)
        eye_band_bottom = _clamp(float(cfg.get("eye_band_bottom_frac", 0.75)), 0.1, 1.0)
        if eye_band_bottom <= eye_band_top:
            eye_band_top, eye_band_bottom = 0.30, 0.75

        clearance_percentile = _clamp(float(cfg.get("clearance_percentile", 70.0)), 50.0, 95.0)
        inverse_depth = bool(cfg.get("inverse_depth", True))
        uniform_range_thresh = max(0.0, float(cfg.get("uniform_range_thresh", 5.0)))

        yaw_gain_deg_s = _clamp(float(cfg.get("yaw_gain_deg_s", 35.0)), 5.0, 90.0)
        brake_norm_thresh = _clamp(float(cfg.get("brake_norm_thresh", 0.70)), 0.3, 0.95)
        creep_norm_thresh = _clamp(float(cfg.get("creep_norm_thresh", 0.30)), 0.05, 0.7)
        if creep_norm_thresh >= brake_norm_thresh:
            creep_norm_thresh = brake_norm_thresh * 0.4

        hold_altitude_m = _clamp(float(cfg.get("hold_altitude_m", 5.0)), 1.5, 50.0)
        z_hold = -hold_altitude_m  # NED: above ground = negative z
        face_forward_on_start = bool(cfg.get("face_forward_on_start", True))

        dt = 1.0 / rate_hz
        center_idx = (n_cols - 1) / 2.0

        print(
            "[autonomous_explore] start "
            f"max_v={max_v:.2f} cruise={cruise_v:.2f} rate_hz={rate_hz:.1f} "
            f"duration_s={duration_s:.1f} n_cols={n_cols} z_hold={z_hold:.1f} "
            f"inverse_depth={inverse_depth}"
        )

        client.takeoffAsync().join()
        print("[autonomous_explore] takeoff complete")
        if face_forward_on_start:
            print("[autonomous_explore] rotating 180 degrees to face forward...")
            client.rotateByYawRateAsync(60, 3).join()

        def vz_trim() -> float:
            z = float(client.getMultirotorState().kinematics_estimated.position.z_val)
            err = z - z_hold
            if err < -0.3:
                return 0.35
            if err > 0.3:
                return -0.35
            return 0.0

        t0 = time.monotonic()
        steps = 0
        no_frame_streak = 0

        while time.monotonic() - t0 < duration_s:
            tick_start = time.monotonic()

            frame = self.latest_frame()
            depth_map: np.ndarray | None = None
            if frame is not None:
                try:
                    depth_map = get_depth_info(frame)
                except Exception as exc:
                    if steps % max(1, int(rate_hz)) == 0:
                        print(f"[autonomous_explore] depth error: {exc}")
                    depth_map = None

            yaw_rad = _yaw_from_orientation(
                client.getMultirotorState().kinematics_estimated.orientation
            )
            cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
            vz = vz_trim()

            if depth_map is None:
                no_frame_streak += 1
                client.moveByVelocityAsync(0.0, 0.0, vz, dt).join()
                if steps % max(1, int(rate_hz)) == 0:
                    print(
                        f"[autonomous_explore] no depth (streak={no_frame_streak}); hovering"
                    )
                steps += 1
                self._sleep_remaining(tick_start, dt)
                continue
            no_frame_streak = 0

            h, _w = depth_map.shape
            r0 = int(eye_band_top * h)
            r1 = int(eye_band_bottom * h)
            if r1 <= r0 + 1:
                r0, r1 = int(0.30 * h), max(int(0.30 * h) + 2, int(0.75 * h))
            band = depth_map[r0:r1, :]

            col_edges = np.linspace(0, band.shape[1], n_cols + 1, dtype=int)
            col_scores = np.empty(n_cols, dtype=np.float32)
            for i in range(n_cols):
                strip = band[:, col_edges[i]:col_edges[i + 1]]
                col_scores[i] = float(np.percentile(strip, clearance_percentile))

            obstacle_score = col_scores if inverse_depth else -col_scores
            raw_range = float(obstacle_score.max() - obstacle_score.min())

            if raw_range < uniform_range_thresh:
                chosen = round(center_idx)
                fwd_speed = cruise_v
                yaw_rate = 0.0
                center_norm = 0.0
                state_label = "uniform"
            else:
                norm = (obstacle_score - obstacle_score.min()) / max(1e-6, raw_range)
                chosen = int(np.argmin(norm))
                center_norm = float(norm[round(center_idx)])

                offset = (chosen - center_idx) / max(1.0, center_idx)
                yaw_rate = _clamp(yaw_gain_deg_s * offset, -90.0, 90.0)

                if center_norm >= brake_norm_thresh:
                    fwd_speed = 0.0
                    state_label = "brake"
                elif center_norm >= creep_norm_thresh:
                    fwd_speed = 0.35 * cruise_v
                    state_label = "creep"
                else:
                    fwd_speed = cruise_v
                    state_label = "cruise"

            vx_world = fwd_speed * cos_y
            vy_world = fwd_speed * sin_y
            client.moveByVelocityAsync(
                vx_world,
                vy_world,
                vz,
                dt,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate)),
            ).join()

            if steps % max(1, int(rate_hz)) == 0:
                rounded = np.round(col_scores, 1).tolist()
                print(
                    f"[autonomous_explore] {state_label} "
                    f"cols={rounded} chosen={chosen}/{n_cols - 1} "
                    f"fwd={fwd_speed:.2f} yaw_rate={yaw_rate:+.1f} "
                    f"center_norm={center_norm:.2f} range={raw_range:.1f}"
                )

            steps += 1
            self._sleep_remaining(tick_start, dt)

        print("[autonomous_explore] duration elapsed; hover and land")
        client.hoverAsync().join()
        time.sleep(1.0)
        client.landAsync().join()
        print("[autonomous_explore] done")

    @staticmethod
    def _sleep_remaining(tick_start_s: float, dt: float) -> None:
        remaining = dt - (time.monotonic() - tick_start_s)
        if remaining > 0:
            time.sleep(remaining)
