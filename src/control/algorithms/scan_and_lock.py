"""scan_and_lock: Spin 360° scanning for a red target, lock on, approach, land.

Demo algorithm — visually shows the full perception → decision → action loop:
  1. Takeoff
  2. Rotate slowly while checking vision each tick (scan phase)
  3. On first red-target detection: freeze rotation, print TARGET LOCKED banner
  4. Approach the target using yaw + forward velocity
  5. Land when arrived (r_frac threshold) or approach time cap hit
"""

from __future__ import annotations

import math
import time

import airsim
from src.control.algorithms import Algorithm, register
from src.vision.processing import red_target_info_normalized


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _yaw_from_orientation(orientation) -> float:
    x = float(orientation.x_val)
    y = float(orientation.y_val)
    z = float(orientation.z_val)
    w = float(orientation.w_val)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@register("scan_and_lock")
class ScanAndLock(Algorithm):
    """Spin-scan for red target, lock on, approach, land."""

    def run(self, client: airsim.MultirotorClient) -> None:
        cfg = self._config.get("scan_and_lock", {})
        altitude_m       = float(cfg.get("altitude_m", 3.0))
        scan_yaw_rate    = float(cfg.get("scan_yaw_rate_deg_s", 25.0))   # spin speed during scan
        scan_timeout_s   = float(cfg.get("scan_timeout_s", 20.0))        # give up after full spin(s)
        approach_speed   = float(cfg.get("approach_speed_ms", 2.0))
        yaw_gain         = float(cfg.get("yaw_gain_deg_s", 50.0))
        arrival_r_frac   = float(cfg.get("arrival_r_frac", 0.18))
        max_approach_s   = float(cfg.get("max_approach_s", 10.0))
        rate_hz          = float(cfg.get("rate_hz", 12.0))
        z_hold           = -altitude_m
        dt               = 1.0 / rate_hz

        print("[scan_and_lock] taking off ...")
        client.takeoffAsync().join()
        client.moveToZAsync(z_hold, 2.0).join()
        print(f"[scan_and_lock] at altitude {altitude_m}m — starting 360° scan")

        # ── PHASE 1: SCAN ────────────────────────────────────────────────────
        locked_info: tuple[float, float, float] | None = None
        scan_start = time.monotonic()

        while time.monotonic() - scan_start < scan_timeout_s:
            # Rotate one tick worth at a time so we can check vision each step.
            client.rotateByYawRateAsync(scan_yaw_rate, dt).join()

            frame = self.latest_frame()
            if frame is not None:
                info = red_target_info_normalized(frame)
                if info is not None:
                    locked_info = info
                    break

            elapsed = time.monotonic() - scan_start
            if int(elapsed) % 3 == 0 and elapsed % 1.0 < dt * 2:
                print(f"[scan_and_lock] scanning ... {elapsed:.1f}s elapsed")

        if locked_info is None:
            print("[scan_and_lock] ⚠  scan complete — no target found. Landing.")
            client.landAsync().join()
            return

        nx, ny, r_frac = locked_info
        print()
        print("=" * 52)
        print("  🎯  TARGET LOCKED")
        print(f"      nx={nx:+.2f}  ny={ny:+.2f}  r_frac={r_frac:.3f}")
        print("=" * 52)
        print()

        # ── PHASE 2: APPROACH ────────────────────────────────────────────────
        approach_start = time.monotonic()
        arrived_reason: str | None = None

        while time.monotonic() - approach_start < max_approach_s:
            frame = self.latest_frame()
            info = red_target_info_normalized(frame) if frame is not None else None

            if info is None:
                # Lost sight — keep last heading, slow down
                client.moveByVelocityAsync(0.5, 0.0, 0.0, dt).join()
                continue

            nx, ny, r_frac = info

            if r_frac >= arrival_r_frac:
                arrived_reason = f"r_frac={r_frac:.2f} >= {arrival_r_frac:.2f}"
                break

            state = client.getMultirotorState()
            yaw_rad = _yaw_from_orientation(state.kinematics_estimated.orientation)
            cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)

            yaw_rate  = _clamp(yaw_gain * nx, -120.0, 120.0)
            alignment = max(0.0, 1.0 - abs(nx))
            fwd       = approach_speed * (0.4 + 0.6 * alignment)
            vx = fwd * cos_y
            vy = fwd * sin_y

            # Alt correction
            z_now = float(client.getMultirotorState().kinematics_estimated.position.z_val)
            vz = _clamp((z_now - z_hold) * 0.5, -0.4, 0.4)

            client.moveByVelocityAsync(
                vx, vy, vz, dt,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate)),
            ).join()

            print(f"[scan_and_lock] approach nx={nx:+.2f} r_frac={r_frac:.3f} fwd={fwd:.2f}")

        if arrived_reason:
            print(f"[scan_and_lock] ✅ arrived — {arrived_reason}")
        else:
            print("[scan_and_lock] approach time cap hit — landing here")

        client.hoverAsync().join()
        time.sleep(0.8)
        client.landAsync().join()
        print("[scan_and_lock] done")
