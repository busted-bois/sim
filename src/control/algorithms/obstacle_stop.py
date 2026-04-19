"""obstacle_stop — fly forward and stop on depth-image obstacle detection.

Uses AirSim's DepthPlanar camera to scan the central region-of-interest every
tick. If any pixel in that region is closer than ``stop_distance_m``, the drone
stops and hovers in place.

Config block in sim.config.json (all optional):
    "obstacle_stop": {
        "cruise_speed_ms": 1.5,
        "stop_distance_m": 5.0,
        "altitude_ned_m": -5.0,
        "flight_duration_s": 20.0,
        "scan_hz": 10.0,
        "roi_fraction": 0.4,
        "camera_name": "0"
    }
"""

from __future__ import annotations

import time

import numpy as np

import airsim
from src.control.algorithms import Algorithm, register


@register("obstacle_stop")
class ObstacleStop(Algorithm):
    """Fly forward; stop and hover when an obstacle is detected via depth camera."""

    def run(self, client: airsim.MultirotorClient) -> None:
        cfg = self._config.get("obstacle_stop", {})
        cruise_speed_ms = float(cfg.get("cruise_speed_ms", 1.5))
        stop_distance_m = float(cfg.get("stop_distance_m", 5.0))
        camera_name = str(cfg.get("camera_name", "0"))
        roi_fraction = float(cfg.get("roi_fraction", 0.4))
        flight_duration_s = float(cfg.get("flight_duration_s", 20.0))
        target_z = float(cfg.get("altitude_ned_m", -5.0))
        scan_hz = min(float(cfg.get("scan_hz", 10.0)), 20.0)
        dt = 1.0 / scan_hz

        print(
            f"[obstacle_stop] init cruise={cruise_speed_ms:.2f}m/s "
            f"stop_dist={stop_distance_m:.1f}m alt={target_z:.2f} "
            f"roi={roi_fraction * 100:.0f}% scan_hz={scan_hz:.1f}"
        )

        client.takeoffAsync().join()
        client.moveToZAsync(target_z, 1.0).join()
        print("[obstacle_stop] at altitude — starting forward obstacle scan")

        obstacle_detected = False
        detection_dist_m: float | None = None
        step = 0
        deadline = time.monotonic() + flight_duration_s

        while time.monotonic() < deadline:
            t0 = time.perf_counter()
            step += 1

            min_depth = self._scan_depth(client, camera_name, roi_fraction)

            if min_depth is not None and min_depth < stop_distance_m:
                obstacle_detected = True
                detection_dist_m = min_depth
                print(
                    f"[obstacle_stop] OBSTACLE DETECTED {min_depth:.2f}m away "
                    f"(threshold={stop_distance_m:.1f}m) — stopping at step {step}"
                )
                break

            client.moveByVelocityZAsync(cruise_speed_ms, 0.0, target_z, dt).join()

            pos = client.getMultirotorState().kinematics_estimated.position
            depth_str = f"{min_depth:.2f}m" if min_depth is not None else "n/a"
            print(
                f"[obstacle_stop] step={step} "
                f"pos=({pos.x_val:.1f},{pos.y_val:.1f},{pos.z_val:.1f}) "
                f"depth_min={depth_str}"
            )

            elapsed = time.perf_counter() - t0
            sleep_s = dt - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

        if obstacle_detected:
            print(f"[obstacle_stop] hovering — obstacle was {detection_dist_m:.2f}m away")
        else:
            print("[obstacle_stop] flight complete — no obstacle detected within range")

        client.hoverAsync().join()

    def _scan_depth(
        self,
        client: airsim.MultirotorClient,
        camera_name: str,
        roi_fraction: float,
    ) -> float | None:
        """Return the minimum depth (metres) in the central ROI, or None on error."""
        try:
            responses = client.simGetImages(
                [
                    airsim.ImageRequest(
                        camera_name=camera_name,
                        image_type=airsim.ImageType.DepthPlanar,
                        pixels_as_float=True,
                        compress=False,
                    )
                ]
            )
            if not responses:
                return None
            r = responses[0]
            w, h = int(r.width), int(r.height)
            if w <= 0 or h <= 0:
                return None
            depth = np.array(r.image_data_float, dtype=np.float32).reshape(h, w)
            # Crop to the central ROI (forward-facing region of interest)
            y0 = int(h * (0.5 - roi_fraction / 2))
            y1 = int(h * (0.5 + roi_fraction / 2))
            x0 = int(w * (0.5 - roi_fraction / 2))
            x1 = int(w * (0.5 + roi_fraction / 2))
            roi = depth[y0:y1, x0:x1]
            valid = roi[(roi > 0.1) & (roi < 1000.0)]
            if valid.size == 0:
                return None
            return float(valid.min())
        except Exception as exc:  # noqa: BLE001
            print(f"[obstacle_stop] depth scan error: {exc}")
            return None
