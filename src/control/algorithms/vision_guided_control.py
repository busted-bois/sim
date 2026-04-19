"""
Vision-guided control: bounded motion from camera — search forward, pause/align when a
red target is seen (HSV + Hough, see processing.find_red_circles).

Set vision_guided_control.inject_fake_detection=true to verify behavior without a real target.
"""

from __future__ import annotations

import time

import airsim
from src.control.algorithms import Algorithm, register
from src.vision.processing import red_target_offset_normalized


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@register("vision_guided_control")
class VisionGuidedControl(Algorithm):
    def run(self, client: airsim.MultirotorClient) -> None:
        cfg = self._config.get("vision_guided_control", {})
        control = self._config.get("control", {})

        cap = float(control.get("max_speed_ms", 10.0))
        max_v = _clamp(float(cfg.get("max_speed_ms", 1.2)), 0.2, cap)
        forward_v = _clamp(float(cfg.get("search_forward_speed_ms", 0.45)), 0.0, max_v)
        nudge_k = _clamp(float(cfg.get("nudge_gain", 0.85)), 0.0, 2.0)
        center_deadband = _clamp(float(cfg.get("center_deadband", 0.14)), 0.02, 0.5)
        rate_hz = _clamp(float(cfg.get("rate_hz", 12.0)), 4.0, 30.0)
        duration_s = _clamp(float(cfg.get("duration_s", 45.0)), 5.0, 300.0)
        inject_fake = bool(cfg.get("inject_fake_detection", False))
        hold_when_centered = bool(cfg.get("hold_when_centered", True))

        dt = 1.0 / rate_hz
        z_hold = float(self._config.get("waypoints", [{"z": -5.0}])[0].get("z", -5.0))

        print(
            "[vision_guided_control] start "
            f"max_v={max_v:.2f} forward={forward_v:.2f} rate_hz={rate_hz:.1f} "
            f"duration_s={duration_s:.1f} inject_fake={inject_fake}"
        )

        client.takeoffAsync().join()
        print("[vision_guided_control] takeoff complete")

        # Hold altitude loosely: small down velocity if we're above |z_hold|
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
        while time.monotonic() - t0 < duration_s:
            frame = self.latest_frame()
            if inject_fake:
                offset = (0.0, 0.0)
            elif frame is not None:
                offset = red_target_offset_normalized(frame)
            else:
                offset = None

            vz = vz_trim()

            if offset is not None:
                nx, ny = offset[0], offset[1]
                if hold_when_centered and abs(nx) < center_deadband and abs(ny) < center_deadband:
                    client.moveByVelocityAsync(0.0, 0.0, vz, dt).join()
                    if steps % max(1, int(rate_hz)) == 0:
                        print("[vision_guided_control] target centered -> hold")
                else:
                    # NED: +X forward, +Y right. Target right (+nx) -> +vy
                    vx = forward_v * 0.35
                    vy = nudge_k * nx * max_v
                    vx = _clamp(vx, 0.0, max_v)
                    vy = _clamp(vy, -max_v, max_v)
                    client.moveByVelocityAsync(vx, vy, vz, dt).join()
                    if steps % max(1, int(rate_hz // 2)) == 0:
                        print(
                            f"[vision_guided_control] target seen align nx={nx:.2f} ny={ny:.2f} "
                            f"vx={vx:.2f} vy={vy:.2f}"
                        )
            else:
                client.moveByVelocityAsync(forward_v, 0.0, vz, dt).join()
                if steps % max(1, int(rate_hz)) == 0:
                    print("[vision_guided_control] searching (no target)")

            steps += 1

        client.hoverAsync().join()
        print("[vision_guided_control] hover settle before land")
        time.sleep(1.0)
        client.landAsync().join()
        print("[vision_guided_control] done")

