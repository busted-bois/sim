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
from src.control.flight_client import FlightClient
from src.control.primitives import rotate_yaw, takeoff_with_settle
from src.control.utils import _clamp, _yaw_from_orientation, make_vz_trim
from src.vision.depth_perception import DepthEstimator
from src.vision.processing import (
    blue_ring_info_normalized,
    get_depth_info,
    red_target_info_normalized,
)


@register("autonomous_explore")
class AutonomousExplore(Algorithm):
    config_section = "autonomous_explore"

    def run(self, client: FlightClient) -> None:
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

        pursue_targets = bool(cfg.get("pursue_targets", True))
        pursue_blue_rings = bool(cfg.get("pursue_blue_rings", True))
        pursue_red_targets = bool(cfg.get("pursue_red_targets", True))
        target_yaw_gain_deg_s = _clamp(float(cfg.get("target_yaw_gain_deg_s", 50.0)), 5.0, 120.0)
        target_approach_speed_ms = _clamp(
            float(cfg.get("target_approach_speed_ms", 2.0)), 0.0, max_v
        )
        target_arrival_r_frac = _clamp(float(cfg.get("target_arrival_r_frac", 0.20)), 0.05, 0.95)
        target_min_r_frac = _clamp(float(cfg.get("target_min_r_frac", 0.02)), 0.005, 0.5)

        # When a blue ring fills the frame we want to fly *through* it (it's a
        # gate), not stop at it. The drone locks its current heading and commits
        # forward at flythrough_speed for flythrough_duration_s — long enough
        # to clear the depth of the ring and exit the other side. Pursuit and
        # depth-wander are suppressed during the commit so the detector losing
        # the ring (because it fills/leaves the frame as we punch through)
        # doesn't cause veering.
        flythrough_blue_rings = bool(cfg.get("flythrough_blue_rings", True))
        flythrough_trigger_r_frac = _clamp(
            float(cfg.get("flythrough_trigger_r_frac", 0.18)), 0.05, 0.6
        )
        flythrough_duration_s = _clamp(float(cfg.get("flythrough_duration_s", 2.0)), 0.3, 8.0)
        flythrough_speed_ms = _clamp(
            float(cfg.get("flythrough_speed_ms", target_approach_speed_ms)), 0.2, max_v
        )
        # Alignment gate: even at close range, refuse to commit to fly-through
        # unless the ring center is within this normalized horizontal offset.
        # If close + off-center, drone enters a lineup phase (forward = 0, yaw
        # only) until centered, then commits. Without this the drone clips the
        # rim when the trigger fires while still partway off to one side.
        flythrough_align_max_nx = _clamp(
            float(cfg.get("flythrough_align_max_nx", 0.18)), 0.05, 0.5
        )
        # Boost yaw authority during the close-range lineup phase so the
        # drone can swing onto axis quickly without the ring drifting out of
        # frame. Multiplier on target_yaw_gain_deg_s.
        lineup_yaw_gain_mult = _clamp(float(cfg.get("lineup_yaw_gain_mult", 1.6)), 0.5, 4.0)
        # Red target should only matter once a gate has been cleared. Before
        # that, the only objective is finding and flying through the ring —
        # red detected on the side must not pull the drone off course.
        red_only_after_gate = bool(cfg.get("red_only_after_gate", True))
        # After punching through a ring, suppress blue-ring pursuit briefly so
        # the drone can scan for the next objective (typically a red target)
        # instead of immediately re-locking on whatever ring is still in view.
        post_flythrough_blue_cooldown_s = _clamp(
            float(cfg.get("post_flythrough_blue_cooldown_s", 5.0)), 0.0, 30.0
        )
        # Hold the post-flythrough heading at cruise for this long while
        # scanning for red. Without it, depth-wander picks the freest column
        # (often well off-axis) and yaws the drone away from where the red
        # target actually is. Red pursuit still preempts immediately if found.
        post_flythrough_scan_s = _clamp(float(cfg.get("post_flythrough_scan_s", 4.0)), 0.0, 20.0)
        post_flythrough_scan_speed_ms = _clamp(
            float(cfg.get("post_flythrough_scan_speed_ms", cruise_v)), 0.0, max_v
        )
        # Once a blue ring has been seen at this size or larger, lock onto it
        # and ignore red targets until either fly-through fires or the ring is
        # not seen for blue_lock_timeout_s. Without this, a red target visible
        # in the periphery yanks the drone sideways during approach and it
        # ends up jamming against the lower rim instead of going through.
        _clamp(float(cfg.get("blue_lock_r_frac", 0.08)), 0.01, 0.5)
        blue_lock_timeout_s = _clamp(float(cfg.get("blue_lock_timeout_s", 1.5)), 0.2, 5.0)

        dt = 1.0 / rate_hz
        center_idx = (n_cols - 1) / 2.0

        print(
            "[autonomous_explore] start "
            f"max_v={max_v:.2f} cruise={cruise_v:.2f} rate_hz={rate_hz:.1f} "
            f"duration_s={duration_s:.1f} n_cols={n_cols} z_hold={z_hold:.1f} "
            f"inverse_depth={inverse_depth}"
        )

        takeoff_with_settle(client, max_attempts=4, label="autonomous_explore")
        print("[autonomous_explore] takeoff complete")
        if face_forward_on_start:
            print("[autonomous_explore] rotating 180 degrees to face forward...")
            rot_cfg = self._config.get("startup_rotation", {})
            rot_rate_dps = float(rot_cfg.get("rate_dps", 60))
            rot_duration_s = float(rot_cfg.get("duration_s", 3.0))
            rotate_yaw(client, rot_rate_dps, rot_duration_s, label="autonomous_explore")
        # Diagnostic: log the heading the explore loop is about to start with
        # so you can tell at a glance whether face_forward_on_start has the
        # drone pointed the way you expect.
        spawn_yaw_deg = math.degrees(
            _yaw_from_orientation(
                client.getMultirotorState().kinematics_estimated.orientation
            )
        )
        print(f"[autonomous_explore] start heading yaw={spawn_yaw_deg:+.1f}°")

        vz_trim = make_vz_trim(client, z_hold)

        depth_cfg = self._config.get("vision", {}).get("depth", {})
        depth_enabled = depth_cfg.get("enabled", False)
        depth_estimator = DepthEstimator(depth_cfg.get("model_path")) if depth_enabled else None

        t0 = time.monotonic()
        steps = 0
        no_frame_streak = 0
        # Timers for metrics
        blue_gate_time: float | None = None
        red_target_time: float | None = None

        # Fly-through state: when a blue ring is close enough, set these so
        # subsequent ticks blindly punch forward on a locked heading until the
        # deadline. Cleared back to None once we exit the gate.
        flythrough_until_s: float | None = None
        flythrough_yaw_rad: float = 0.0
        blue_suppressed_until_s: float = 0.0
        blue_lock_until_s: float = 0.0
        red_lock_until_s: float = 0.0
        # Becomes True after the first fly-through completes. Until then,
        # red-target pursuit is suppressed (when red_only_after_gate=true)
        # so a red target visible to the side can't divert the drone before
        # it has cleared the gate.
        gate_cleared: bool = False
        scan_until_s: float = 0.0
        scan_yaw_rad: float = 0.0
        current_target_kind: str | None = None
        # Last successful blue-ring detection (nx, ny, r_frac, timestamp, yaw_rad).
        # Used to keep pursuing the ring through brief detector dropouts —
        # HoughCircles regularly misses one or two consecutive frames at
        # close range or at frame edges, and without this memory the drone
        # falls back to depth-wander on the first miss and forgets the ring.
        # The stored yaw is used to compensate the cached nx as the drone
        # rotates, so the stale value doesn't keep pinning the target at
        # the frame edge after we've already yawed toward it.
        last_blue: tuple[float, float, float, float, float] | None = None
        # Same as last_blue, but for red targets.
        last_red: tuple[float, float, float, float, float] | None = None

        # --- IMPROVED PURSUIT & SEARCH LOGIC ---
        last_target_seen_s = time.monotonic()
        search_scan_offset = 0.0
        search_direction = 1.0
        # Remembers which side the last target was on so we search in that
        # direction first if it's lost.
        last_target_nx = 0.0

        # Half of the configured camera FOV in degrees, used to map yaw delta
        # back into image-normalized horizontal offset (nx).
        cam_half_fov_deg = max(
            10.0, float(self._config.get("vision", {}).get("fov_degrees", 100.0)) / 2.0
        )

        while time.monotonic() - t0 < duration_s:
            tick_start = time.monotonic()

            # If we're committed to flying through a ring, ignore the camera
            # entirely and drive forward on the locked heading. The ring will
            # leave the frame as we punch through it, so any detector signal
            # during this window is noise.
            if flythrough_until_s is not None:
                if time.monotonic() < flythrough_until_s:
                    cos_y = math.cos(flythrough_yaw_rad)
                    sin_y = math.sin(flythrough_yaw_rad)
                    z = float(
                        client.getMultirotorState().kinematics_estimated.position.z_val
                    )
                    err = z - z_hold
                    vz_ft = 0.35 if err < -0.3 else (-0.35 if err > 0.3 else 0.0)
                    client.moveByVelocityAsync(
                        flythrough_speed_ms * cos_y,
                        flythrough_speed_ms * sin_y,
                        vz_ft,
                        dt,
                    ).join()
                    if steps % max(1, int(rate_hz)) == 0:
                        remaining = flythrough_until_s - time.monotonic()
                        print(
                            f"[autonomous_explore] flythrough committed "
                            f"({remaining:.1f}s left, fwd={flythrough_speed_ms:.2f})"
                        )
                    steps += 1
                    self._sleep_remaining(tick_start, dt)
                    continue
                flythrough_until_s = None
                blue_suppressed_until_s = (
                    time.monotonic() + post_flythrough_blue_cooldown_s
                )
                scan_until_s = time.monotonic() + post_flythrough_scan_s
                scan_yaw_rad = flythrough_yaw_rad
                gate_cleared = True
                if blue_gate_time is None:
                    blue_gate_time = time.monotonic() - t0
                print(
                    "[autonomous_explore] flythrough complete; resuming explore "
                    f"(blue-ring suppressed for {post_flythrough_blue_cooldown_s:.1f}s, "
                    f"scanning straight for {post_flythrough_scan_s:.1f}s, "
                    "red target now active)"
                )

            frame = self.latest_frame()
            depth_map: np.ndarray | None = None
            if frame is not None:
                try:
                    depth_map = get_depth_info(frame, depth_estimator) if depth_estimator else None
                except Exception as exc:
                    if steps % max(1, int(rate_hz)) == 0:
                        print(f"[autonomous_explore] depth error: {exc}")
                    depth_map = None

            yaw_rad = _yaw_from_orientation(
                client.getMultirotorState().kinematics_estimated.orientation
            )
            cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
            vz = vz_trim()

            # Target pursuit overrides depth-based wander whenever a blue ring
            # or red circle is in view. Blue takes priority — rings are gates
            # to fly through, red is a static target.
            target_info: tuple[str, float, float, float] | None = None
            if pursue_targets and frame is not None:
                now_s = time.monotonic()
                blue_active = pursue_blue_rings and now_s >= blue_suppressed_until_s
                blue = blue_ring_info_normalized(frame) if blue_active else None
                # Red is gated on `gate_cleared`: don't even look until at
                # least one ring has been flown through, so a red target
                # visible to the side can't divert the drone pre-gate.
                red_allowed = pursue_red_targets and (gate_cleared or not red_only_after_gate)
                red = red_target_info_normalized(frame) if red_allowed else None

                # Engage blue lock once we see a ring close enough to commit to,
                # and refresh the lock every frame the ring stays in view.
                if blue is not None:
                    last_target_seen_s = now_s
                    last_target_nx = blue[0]
                    last_blue = (blue[0], blue[1], blue[2], now_s, yaw_rad)
                    # CRITICAL: If we see a gate, we LOCK ON and ignore everything else
                    blue_lock_until_s = now_s + 2.5
                blue_lock_engaged = now_s < blue_lock_until_s

                if red is not None:
                    last_target_seen_s = now_s
                    last_target_nx = red[0]
                    last_red = (red[0], red[1], red[2], now_s, yaw_rad)
                    red_lock_until_s = now_s + 2.0
                red_lock_engaged = now_s < red_lock_until_s

                # Detector dropout recovery: if locked but this frame missed,
                # reuse the most recent detection — but compensate nx by the
                # yaw rotation we've performed since then. Without this the
                # cached nx=+0.99 keeps commanding a hard-right yaw forever.
                if blue is None and blue_lock_engaged and last_blue is not None:
                    age_s = now_s - last_blue[3]
                    # Extend timeout to 2.0s for better distant locking.
                    if age_s <= max(2.0, blue_lock_timeout_s):
                        delta_yaw_deg = math.degrees(yaw_rad - last_blue[4])
                        # Wrap to (-180, 180] so wraparound doesn't blow up nx.
                        delta_yaw_deg = (delta_yaw_deg + 180.0) % 360.0 - 180.0
                        nx_compensated = last_blue[0] - delta_yaw_deg / cam_half_fov_deg
                        # If we've yawed past the target (|nx_compensated| > 1.2)
                        # we'd be commanding the drone back toward where the ring
                        # was; clamp so it gently re-centers instead of overshooting.
                        nx_compensated = _clamp(nx_compensated, -1.0, 1.0)
                        blue = (nx_compensated, last_blue[1], last_blue[2])
                        # Update coasting direction for scan initialization
                        last_target_nx = nx_compensated
                        if steps % max(1, int(rate_hz)) == 0:
                            print(
                                f"[autonomous_explore] blue dropout — last seen "
                                f"{age_s:.2f}s ago, raw_nx={last_blue[0]:+.2f} "
                                f"compensated_nx={blue[0]:+.2f} "
                                f"(yawed {delta_yaw_deg:+.1f}°)"
                            )

                if red is None and red_lock_engaged and last_red is not None:
                    age_s = now_s - last_red[3]
                    if age_s <= 2.0:  # Consistent timeout for red recovery
                        delta_yaw_deg = math.degrees(yaw_rad - last_red[4])
                        delta_yaw_deg = (delta_yaw_deg + 180.0) % 360.0 - 180.0
                        nx_compensated = last_red[0] - delta_yaw_deg / cam_half_fov_deg
                        nx_compensated = _clamp(nx_compensated, -1.0, 1.0)
                        red = (nx_compensated, last_red[1], last_red[2])
                        # Update coasting direction for scan initialization
                        last_target_nx = nx_compensated
                        if steps % max(1, int(rate_hz)) == 0:
                            print(
                                f"[autonomous_explore] red dropout — last seen "
                                f"{age_s:.2f}s ago, raw_nx={last_red[0]:+.2f} "
                                f"compensated_nx={red[0]:+.2f} "
                                f"(yawed {delta_yaw_deg:+.1f}°)"
                            )

                # --- PROXIMITY-BASED TARGET ARBITRATION ---
                # Evaluate all allowed targets and pick the closest (largest r_frac).
                # Applies a 50% "stickiness" hysteresis to current_target_kind.
                candidates: list[tuple[str, float, float, float]] = []
                if blue is not None and blue[2] >= target_min_r_frac:
                    candidates.append(("blue_ring", *blue))

                # Commitment: if locked onto a blue ring and we still have a
                # blue candidate (seen or recovered), ignore red entirely to
                # avoid distraction during the approach.
                if blue_lock_engaged and blue is not None:
                    red_allowed = False
                else:
                    red_allowed = pursue_red_targets and (
                        gate_cleared or not red_only_after_gate or not blue_active
                    )

                if red is not None and red[2] >= target_min_r_frac and red_allowed:
                    candidates.append(("red_target", *red))

                if not candidates:
                    target_info = None
                    current_target_kind = None
                else:
                    # Sort candidates by size (closest first)
                    candidates.sort(key=lambda c: c[3], reverse=True)
                    best_candidate = candidates[0]

                    # Stickiness: keep the current target kind unless another
                    # is significantly (50%) closer.
                    current_cand = next(
                        (c for c in candidates if c[0] == current_target_kind), None
                    )

                    if current_cand is not None:
                        if best_candidate[3] > current_cand[3] * 1.5:
                            target_info = best_candidate
                            current_target_kind = best_candidate[0]
                        else:
                            target_info = current_cand
                    else:
                        target_info = best_candidate
                        current_target_kind = best_candidate[0]

            if target_info is not None:
                kind, nx, ny, r_frac = target_info
                # Blue ring close enough to consider committing.
                if (
                    kind == "blue_ring"
                    and flythrough_blue_rings
                    and r_frac >= flythrough_trigger_r_frac
                ):
                    if abs(nx) <= flythrough_align_max_nx:
                        # Aligned + close → commit to fly-through.
                        flythrough_until_s = time.monotonic() + flythrough_duration_s
                        flythrough_yaw_rad = yaw_rad
                        print(
                            f"[autonomous_explore] blue_ring aligned + close "
                            f"(r_frac={r_frac:.2f} nx={nx:+.2f}); "
                            f"committing to fly-through for {flythrough_duration_s:.1f}s"
                        )
                        steps += 1
                        self._sleep_remaining(tick_start, dt)
                        continue
                    # Close but off-center → lineup phase: slow down and
                    # yaw aggressively until centered. The drone maintains a
                    # small "creep" speed so it doesn't stall out.
                    lineup_v = _clamp(0.4 * cruise_v, 0.2, 1.0)
                    yaw_rate = _clamp(
                        target_yaw_gain_deg_s * lineup_yaw_gain_mult * nx, -120.0, 120.0
                    )
                    client.moveByVelocityAsync(
                        lineup_v * cos_y,
                        lineup_v * sin_y,
                        vz,
                        dt,
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate)),
                    ).join()
                    if steps % max(1, int(rate_hz // 2)) == 0:
                        print(
                            f"[autonomous_explore] lineup nx={nx:+.2f} (need |nx|<="
                            f"{flythrough_align_max_nx:.2f}) r_frac={r_frac:.2f} "
                            f"v={lineup_v:.2f} yaw_rate={yaw_rate:+.1f}"
                        )
                    steps += 1
                    self._sleep_remaining(tick_start, dt)
                    continue
                # Red target arrival (or blue with flythrough disabled) → stop.
                if r_frac >= target_arrival_r_frac:
                    arrival_time = time.monotonic() - t0
                    if kind == "red_target" and red_target_time is None:
                        red_target_time = arrival_time
                    elif kind == "blue_ring" and blue_gate_time is None:
                        blue_gate_time = arrival_time

                    print(
                        f"[autonomous_explore] {kind} arrived at {arrival_time:.2f}s "
                        f"r_frac={r_frac:.2f} >= {target_arrival_r_frac:.2f}; hover"
                    )
                    client.hoverAsync().join()
                    break
                alignment = max(0.0, 1.0 - abs(nx))
                fwd_speed = _clamp(
                    target_approach_speed_ms * (0.35 + 0.65 * alignment), 0.0, max_v
                )
                yaw_rate = _clamp(target_yaw_gain_deg_s * nx, -120.0, 120.0)
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
                    print(
                        f"[autonomous_explore] pursue {kind} nx={nx:+.2f} ny={ny:+.2f} "
                        f"r_frac={r_frac:.2f} fwd={fwd_speed:.2f} yaw_rate={yaw_rate:+.1f}"
                    )
                steps += 1
                self._sleep_remaining(tick_start, dt)
                continue

            # Post-flythrough scan window: no target this tick, but we're
            # within the post-gate window where red ought to be ahead. Hold
            # the locked heading at cruise speed instead of letting depth-
            # wander pick the freest column off-axis. Red pursuit branch
            # above runs first, so as soon as red is detected this loop
            # naturally exits scan mode and pursues.
            if time.monotonic() < scan_until_s:
                cos_s = math.cos(scan_yaw_rad)
                sin_s = math.sin(scan_yaw_rad)
                yaw_err_deg = math.degrees(
                    (scan_yaw_rad - yaw_rad + math.pi) % (2 * math.pi) - math.pi
                )
                # Gentle drift correction back to locked heading.
                yaw_rate_scan = _clamp(2.0 * yaw_err_deg, -25.0, 25.0)
                client.moveByVelocityAsync(
                    post_flythrough_scan_speed_ms * cos_s,
                    post_flythrough_scan_speed_ms * sin_s,
                    vz,
                    dt,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate_scan)),
                ).join()
                if steps % max(1, int(rate_hz)) == 0:
                    remaining = scan_until_s - time.monotonic()
                    print(
                        f"[autonomous_explore] scan-straight no_red_yet "
                        f"({remaining:.1f}s left, fwd={post_flythrough_scan_speed_ms:.2f})"
                    )
                steps += 1
                self._sleep_remaining(tick_start, dt)
                continue

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

            # --- ACTIVE SCANNING LOGIC ---
            # If we've been blind for a while, start an active search pattern.
            if target_info is None:
                time_since_target = time.monotonic() - last_target_seen_s
                if time_since_target > 2.5:  # Trigger search after 2.5s of blindness
                    # If this is the start of a search, pick direction based on
                    # where we last saw a target.
                    if search_scan_offset == 0.0 and last_target_nx != 0.0:
                        search_direction = 1.0 if last_target_nx > 0 else -1.0

                    # Perform a scan ±25 degrees to "find" the next gate.
                    search_scan_offset += search_direction * (30.0 * dt)
                    if abs(search_scan_offset) > 25.0:
                        search_direction *= -1.0

                    # Apply the scan offset to the depth-wander yaw.
                    yaw_rate += search_scan_offset
                    state_label = "SCANNING"
            else:
                # Reset scan offset if we have a target
                search_scan_offset = 0.0

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

        # Print metrics
        print("\n--- Performance Metrics ---")
        if blue_gate_time is not None:
            print(f"Time to blue gate: {blue_gate_time:.2f}s")
        else:
            print("Time to blue gate: DNF")

        if red_target_time is not None:
            print(f"Time to red target: {red_target_time:.2f}s")
        else:
            print("Time to red target: DNF")
        print("---------------------------\n")

        print("[autonomous_explore] done")

    @staticmethod
    def _sleep_remaining(tick_start_s: float, dt: float) -> None:
        remaining = dt - (time.monotonic() - tick_start_s)
        if remaining > 0:
            time.sleep(remaining)
