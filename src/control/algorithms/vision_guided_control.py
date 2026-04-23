"""Vision-guided control: search forward, align when a red target is seen, land
when we either arrive at the target (fills the frame) or lose it at close range
(we overflew / passed through it).

Uses HSV + Hough circle detection from src.vision.processing.
Set vision_guided_control.inject_fake_detection=true in sim.config.json to verify
behavior without a real target in the scene.
"""

from __future__ import annotations

import math
import time

import airsim
from src.control.algorithms import Algorithm, register
from src.vision.processing import grey_wall_info_normalized, red_target_info_normalized


def _yaw_from_orientation(orientation) -> float:
    """Extract yaw (radians, Z-axis) from an AirSim orientation quaternion."""
    x = float(orientation.x_val)
    y = float(orientation.y_val)
    z = float(orientation.z_val)
    w = float(orientation.w_val)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

_DETECTOR_DEFAULTS = {
    "red_circle": {"arrival_r_frac": 0.18, "proximity_r_frac": 0.10},
    "grey_wall": {"arrival_r_frac": 0.25, "proximity_r_frac": 0.15},
}


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@register("vision_guided_control")
class VisionGuidedControl(Algorithm):
    def run(self, client: airsim.MultirotorClient) -> None:
        cfg = self._config.get("vision_guided_control", {})
        control = self._config.get("control", {})

        cap = float(control.get("max_speed_ms", 10.0))
        max_v = _clamp(float(cfg.get("max_speed_ms", 3.0)), 0.2, cap)
        forward_v = _clamp(float(cfg.get("search_forward_speed_ms", 1.0)), 0.0, max_v)
        nudge_k = _clamp(float(cfg.get("nudge_gain", 0.85)), 0.0, 2.0)
        yaw_gain_deg_s = _clamp(float(cfg.get("yaw_gain_deg_s", 40.0)), 0.0, 120.0)
        approach_speed_ms = _clamp(float(cfg.get("approach_speed_ms", 2.2)), 0.0, max_v)
        center_deadband = _clamp(float(cfg.get("center_deadband", 0.14)), 0.02, 0.5)
        rate_hz = _clamp(float(cfg.get("rate_hz", 12.0)), 4.0, 30.0)
        duration_s = _clamp(float(cfg.get("duration_s", 45.0)), 5.0, 300.0)
        inject_fake = bool(cfg.get("inject_fake_detection", False))
        hold_when_centered = bool(cfg.get("hold_when_centered", True))

        detector_name = str(cfg.get("detector", "red_circle")).strip().lower()
        if detector_name not in _DETECTOR_DEFAULTS:
            raise ValueError(
                f"Unknown detector '{detector_name}'. "
                f"Expected one of: {', '.join(sorted(_DETECTOR_DEFAULTS.keys()))}"
            )
        detect_defaults = _DETECTOR_DEFAULTS[detector_name]
        detect_fn = {
            "red_circle": red_target_info_normalized,
            "grey_wall": grey_wall_info_normalized,
        }[detector_name]

        # Size thresholds for arrival / "we overflew it" detection. Semantics
        # differ per detector (Hough radius / image for red_circle vs mask area
        # fraction for grey_wall), so defaults shift accordingly.
        arrival_r_frac = _clamp(
            float(cfg.get("arrival_r_frac", detect_defaults["arrival_r_frac"])), 0.05, 0.95
        )
        proximity_r_frac = _clamp(
            float(cfg.get("proximity_r_frac", detect_defaults["proximity_r_frac"])), 0.02, 0.95
        )
        lost_after_close_s = _clamp(float(cfg.get("lost_after_close_s", 1.0)), 0.2, 5.0)
        # Safety cap: once we've been actively approaching (target seen) for
        # this long, land where we are even if r_frac hasn't crossed the
        # arrival threshold. Prevents indefinite chases into walls when the
        # detector can't resolve size growth at close range.
        max_approach_time_s = _clamp(float(cfg.get("max_approach_time_s", 8.0)), 1.0, 60.0)

        dt = 1.0 / rate_hz
        z_hold = float(self._config.get("waypoints", [{"z": -5.0}])[0].get("z", -5.0))

        print(
            "[vision_guided_control] start "
            f"detector={detector_name} max_v={max_v:.2f} forward={forward_v:.2f} "
            f"rate_hz={rate_hz:.1f} duration_s={duration_s:.1f} "
            f"arrival_r_frac={arrival_r_frac:.2f} proximity_r_frac={proximity_r_frac:.2f} "
            f"inject_fake={inject_fake}"
        )

        client.takeoffAsync().join()
        print("[vision_guided_control] takeoff complete")

        # Camera faces aft by default in AirSim; rotate 180° so forward cruise
        # and detection share the same heading (matches opencv_landing).
        print("[vision_guided_control] rotating 180 degrees to face forward...")
        client.rotateByYawRateAsync(60, 3).join()
        print("[vision_guided_control] rotation complete")

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
        last_close_seen_s: float | None = None  # last time we saw target with r_frac >= proximity
        # Time of first detection, used by the approach-time safety cap below.
        first_seen_s: float | None = None
        arrived_reason: str | None = None

        while time.monotonic() - t0 < duration_s:
            frame = self.latest_frame()
            if inject_fake:
                info: tuple[float, float, float] | None = (0.0, 0.0, arrival_r_frac)
            elif frame is not None:
                info = detect_fn(frame)
            else:
                info = None

            vz = vz_trim()
            now_s = time.monotonic()

            # Diagnostic: dump raw detection every ~1s so you can tell whether
            # the detector is locking onto a real object vs ambient scene noise.
            if steps % max(1, int(rate_hz)) == 0:
                if info is None:
                    print("[vision_guided_control] probe: no detection")
                else:
                    print(
                        f"[vision_guided_control] probe: nx={info[0]:.2f} "
                        f"ny={info[1]:.2f} r_frac={info[2]:.2f}"
                    )

            if info is not None:
                nx, ny, r_frac = info

                if first_seen_s is None:
                    first_seen_s = now_s

                if r_frac >= proximity_r_frac:
                    last_close_seen_s = now_s

                if r_frac >= arrival_r_frac:
                    arrived_reason = f"arrived r_frac={r_frac:.2f} >= {arrival_r_frac:.2f}"
                    break

                if (now_s - first_seen_s) >= max_approach_time_s:
                    arrived_reason = (
                        f"approach time cap ({max_approach_time_s:.1f}s elapsed since first "
                        f"detection) r_frac={r_frac:.2f}"
                    )
                    break

                # Read the drone's current yaw so we can drive velocity in its
                # body-forward direction while simultaneously yawing toward the
                # target. Strafing (pure world-frame vy) loses targets that are
                # off to one side; yawing keeps the camera pointed at them.
                state = client.getMultirotorState()
                yaw_rad = _yaw_from_orientation(state.kinematics_estimated.orientation)
                cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)

                # Only freeze in place when we're actually close to the target.
                # At long range, dropping forward speed every time |nx| dips
                # inside the deadband causes the drone to dither and never
                # close distance.
                near_target = r_frac >= proximity_r_frac
                centered = (
                    hold_when_centered
                    and near_target
                    and abs(nx) < center_deadband
                    and abs(ny) < center_deadband
                )
                if centered:
                    fwd_speed = 0.0
                    small_strafe = 0.0
                    yaw_rate = 0.0
                else:
                    # Approach speed scaled down slightly when way off-center so
                    # we yaw into alignment before committing forward.
                    alignment = max(0.0, 1.0 - abs(nx))
                    fwd_speed = _clamp(
                        approach_speed_ms * (0.35 + 0.65 * alignment), 0.0, max_v
                    )
                    # Small strafe for residual drift only — yaw does the heavy lifting.
                    small_strafe = _clamp(0.25 * nudge_k * nx * max_v, -max_v, max_v)
                    yaw_rate = _clamp(yaw_gain_deg_s * nx, -120.0, 120.0)

                vx_world = fwd_speed * cos_y - small_strafe * sin_y
                vy_world = fwd_speed * sin_y + small_strafe * cos_y
                client.moveByVelocityAsync(
                    vx_world,
                    vy_world,
                    vz,
                    dt,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate)),
                ).join()

                if steps % max(1, int(rate_hz // 2)) == 0:
                    label = "centered -> hold" if centered else "align"
                    print(
                        f"[vision_guided_control] target seen {label} nx={nx:.2f} "
                        f"ny={ny:.2f} r_frac={r_frac:.2f} fwd={fwd_speed:.2f} "
                        f"yaw_rate={yaw_rate:.1f}"
                    )
            else:
                # No detection this tick.
                if (
                    last_close_seen_s is not None
                    and (now_s - last_close_seen_s) <= lost_after_close_s
                ):
                    # We were close, now lost — we overflew / passed through it.
                    arrived_reason = (
                        "lost after close "
                        f"({now_s - last_close_seen_s:.2f}s <= {lost_after_close_s:.2f}s)"
                    )
                    break
                # Search: cruise forward in the drone's current heading so it
                # stays on its initial flight path after the 180° startup yaw.
                state = client.getMultirotorState()
                yaw_rad = _yaw_from_orientation(state.kinematics_estimated.orientation)
                cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
                client.moveByVelocityAsync(
                    forward_v * cos_y,
                    forward_v * sin_y,
                    vz,
                    dt,
                ).join()
                if steps % max(1, int(rate_hz)) == 0:
                    print("[vision_guided_control] searching (no target)")

            steps += 1

        if arrived_reason is not None:
            print(f"[vision_guided_control] arrival: {arrived_reason}")
        else:
            print("[vision_guided_control] duration elapsed without arrival")

        client.hoverAsync().join()
        print("[vision_guided_control] hover settle before land")
        time.sleep(1.0)
        client.landAsync().join()
        print("[vision_guided_control] done")
