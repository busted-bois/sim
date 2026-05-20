"""yolo_explore: YOLO-based gate-and-target pursuit without depth sensing.

Behaviour loop every tick:
  1. Grab the latest camera frame.
  2. Run YOLOv8 inference to get bounding-box detections.
  3. If a target detection is found, pursue it (yaw toward center, fly forward).
     - When close enough and aligned, commit to a fly-through (blue gate) or
       arrive (red target) exactly like autonomous_explore.
  4. If nothing is detected, cruise on the last-known heading for a short
     window, then start a yaw-scan search pattern.

No MiDaS depth is used — obstacle avoidance is not performed.
"""

from __future__ import annotations

import math
import time

import airsim
from src.control.algorithms import Algorithm, register
from src.control.flight_client import FlightClient
from src.control.primitives import rotate_yaw, takeoff_with_settle
from src.control.utils import _clamp, _yaw_from_orientation, make_vz_trim
from src.vision.processing import get_yolo_detections
from src.vision.yolo_detector import Detection


def _pick_target(
    detections: list[Detection],
    blue_classes: set[str],
    red_classes: set[str],
    blue_active: bool,
    red_active: bool,
    min_size_frac: float,
) -> tuple[str, Detection] | None:
    """Select the best detection to pursue.

    Returns ``("blue_ring", det)`` or ``("red_target", det)`` or ``None``.
    Blue gates take priority over red targets.
    """
    best_blue: Detection | None = None
    best_red: Detection | None = None

    for det in detections:
        if det.size_frac < min_size_frac:
            continue
        if blue_active and det.class_name in blue_classes and (
            best_blue is None or det.size_frac > best_blue.size_frac
        ):
            best_blue = det
        if red_active and det.class_name in red_classes and (
            best_red is None or det.size_frac > best_red.size_frac
        ):
            best_red = det

    if best_blue is not None:
        return ("blue_ring", best_blue)
    if best_red is not None:
        return ("red_target", best_red)
    return None


@register("yolo_explore")
class YoloExplore(Algorithm):
    config_section = "yolo_explore"

    def run(self, client: FlightClient) -> None:
        cfg = self._config.get("yolo_explore", {})
        control = self._config.get("control", {})

        cap = float(control.get("max_speed_ms", 10.0))
        max_v = _clamp(float(cfg.get("max_speed_ms", 2.5)), 0.2, cap)
        cruise_v = _clamp(float(cfg.get("cruise_speed_ms", 1.5)), 0.0, max_v)
        rate_hz = _clamp(float(cfg.get("rate_hz", 6.0)), 2.0, 30.0)
        duration_s = _clamp(float(cfg.get("duration_s", 60.0)), 5.0, 300.0)

        yolo_model_path = str(cfg.get("model_path", "models/yolov8n.pt"))
        yolo_confidence = _clamp(float(cfg.get("confidence", 0.5)), 0.05, 1.0)
        yolo_classes: list[str] | None = cfg.get("classes") or None
        blue_class_names: set[str] = set(cfg.get("blue_class_names", ["blue_ring"]))
        red_class_names: set[str] = set(cfg.get("red_class_names", ["red_target"]))

        target_yaw_gain_deg_s = _clamp(float(cfg.get("target_yaw_gain_deg_s", 50.0)), 5.0, 120.0)
        target_approach_speed_ms = _clamp(
            float(cfg.get("target_approach_speed_ms", 2.0)), 0.0, max_v
        )
        target_v_gain = _clamp(float(cfg.get("target_v_gain", 2.5)), 0.5, 10.0)
        target_arrival_size_frac = _clamp(
            float(cfg.get("target_arrival_size_frac", 0.15)), 0.01, 0.95
        )
        target_min_size_frac = _clamp(float(cfg.get("target_min_size_frac", 0.001)), 0.0001, 0.5)

        flythrough_trigger_size_frac = _clamp(
            float(cfg.get("flythrough_trigger_size_frac", 0.10)), 0.01, 0.6
        )
        flythrough_duration_s = _clamp(float(cfg.get("flythrough_duration_s", 2.0)), 0.3, 8.0)
        flythrough_speed_ms = _clamp(
            float(cfg.get("flythrough_speed_ms", target_approach_speed_ms)), 0.2, max_v
        )
        flythrough_align_max_nx = _clamp(float(cfg.get("flythrough_align_max_nx", 0.18)), 0.05, 0.5)
        flythrough_align_max_ny = _clamp(float(cfg.get("flythrough_align_max_ny", 0.15)), 0.05, 0.5)
        lineup_yaw_gain_mult = _clamp(float(cfg.get("lineup_yaw_gain_mult", 1.6)), 0.5, 4.0)

        red_only_after_gate = bool(cfg.get("red_only_after_gate", True))
        post_flythrough_blue_cooldown_s = _clamp(
            float(cfg.get("post_flythrough_blue_cooldown_s", 5.0)), 0.0, 30.0
        )
        post_flythrough_scan_s = _clamp(float(cfg.get("post_flythrough_scan_s", 4.0)), 0.0, 20.0)
        post_flythrough_scan_speed_ms = _clamp(
            float(cfg.get("post_flythrough_scan_speed_ms", cruise_v)), 0.0, max_v
        )
        hold_altitude_m = _clamp(float(cfg.get("hold_altitude_m", 5.0)), 1.5, 50.0)
        z_hold = -hold_altitude_m
        face_forward_on_start = bool(cfg.get("face_forward_on_start", True))

        dt = 1.0 / rate_hz

        print(
            f"[yolo_explore] start max_v={max_v:.2f} cruise={cruise_v:.2f} "
            f"rate_hz={rate_hz:.1f} duration_s={duration_s:.1f} "
            f"model={yolo_model_path} conf={yolo_confidence:.2f}"
        )

        takeoff_with_settle(client, max_attempts=4, label="yolo_explore")
        print("[yolo_explore] takeoff complete")

        if face_forward_on_start:
            print("[yolo_explore] rotating 180 degrees to face forward...")
            rot_cfg = self._config.get("startup_rotation", {})
            rot_rate_dps = float(rot_cfg.get("rate_dps", 60))
            rot_duration_s = float(rot_cfg.get("duration_s", 3.0))
            rotate_yaw(client, rot_rate_dps, rot_duration_s, label="yolo_explore")

        spawn_yaw_deg = math.degrees(
            _yaw_from_orientation(
                client.getMultirotorState().kinematics_estimated.orientation
            )
        )
        print(f"[yolo_explore] start heading yaw={spawn_yaw_deg:+.1f}deg")

        vz_trim = make_vz_trim(client, z_hold)

        t0 = time.monotonic()
        steps = 0

        blue_gate_time: float | None = None
        red_target_time: float | None = None

        flythrough_until_s: float | None = None
        flythrough_yaw_rad: float = 0.0
        flythrough_z_target: float = z_hold
        blue_suppressed_until_s: float = 0.0
        gate_cleared: bool = False
        scan_until_s: float = 0.0
        scan_yaw_rad: float = 0.0

        last_target_seen_s = time.monotonic()
        search_scan_offset = 0.0
        search_direction = 1.0
        last_target_nx = 0.0

        while time.monotonic() - t0 < duration_s:
            tick_start = time.monotonic()

            if flythrough_until_s is not None:
                if time.monotonic() < flythrough_until_s:
                    cos_y = math.cos(flythrough_yaw_rad)
                    sin_y = math.sin(flythrough_yaw_rad)
                    z = float(
                        client.getMultirotorState().kinematics_estimated.position.z_val
                    )
                    err = z - flythrough_z_target
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
                            f"[yolo_explore] flythrough committed "
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
                    "[yolo_explore] flythrough complete; resuming explore "
                    f"(blue suppressed {post_flythrough_blue_cooldown_s:.1f}s, "
                    f"scanning straight {post_flythrough_scan_s:.1f}s)"
                )

            frame = self.latest_frame()
            yaw_rad = _yaw_from_orientation(
                client.getMultirotorState().kinematics_estimated.orientation
            )
            cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
            vz = vz_trim()

            target_info: tuple[str, Detection] | None = None
            if frame is not None:
                now_s = time.monotonic()
                blue_active = now_s >= blue_suppressed_until_s
                red_active = gate_cleared or not red_only_after_gate

                detections = get_yolo_detections(
                    frame,
                    model_path=yolo_model_path,
                    confidence=yolo_confidence,
                    classes=yolo_classes,
                )

                target_info = _pick_target(
                    detections,
                    blue_class_names,
                    red_class_names,
                    blue_active,
                    red_active,
                    target_min_size_frac,
                )

                if target_info is not None:
                    last_target_seen_s = now_s
                    last_target_nx = target_info[1].nx
                    search_scan_offset = 0.0

            if target_info is not None:
                kind, det = target_info
                nx, ny, size_frac = det.nx, det.ny, det.size_frac

                # Blue gate close enough to consider fly-through
                if kind == "blue_ring" and size_frac >= flythrough_trigger_size_frac:
                    if (
                        abs(nx) <= flythrough_align_max_nx
                        and abs(ny) <= flythrough_align_max_ny
                    ):
                        flythrough_until_s = time.monotonic() + flythrough_duration_s
                        flythrough_yaw_rad = yaw_rad
                        flythrough_z_target = float(
                            client.getMultirotorState().kinematics_estimated.position.z_val
                        )
                        print(
                            f"[yolo_explore] blue_ring aligned + close "
                            f"(size_frac={size_frac:.3f} nx={nx:+.2f} ny={ny:+.2f}); "
                            f"committing fly-through for {flythrough_duration_s:.1f}s"
                        )
                        steps += 1
                        self._sleep_remaining(tick_start, dt)
                        continue
                    # Close but off-center — lineup phase
                    lineup_v = _clamp(0.4 * cruise_v, 0.2, 1.0)
                    yaw_rate = _clamp(
                        target_yaw_gain_deg_s * lineup_yaw_gain_mult * nx, -120.0, 120.0
                    )
                    vz_lineup = _clamp(target_v_gain * ny, -1.0, 1.0)
                    client.moveByVelocityAsync(
                        lineup_v * cos_y,
                        lineup_v * sin_y,
                        vz_lineup,
                        dt,
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate)),
                    ).join()
                    if steps % max(1, int(rate_hz // 2)) == 0:
                        print(
                            f"[yolo_explore] lineup nx={nx:+.2f} ny={ny:+.2f} "
                            f"size_frac={size_frac:.3f} v={lineup_v:.2f} yaw={yaw_rate:+.1f}"
                        )
                    steps += 1
                    self._sleep_remaining(tick_start, dt)
                    continue

                # Arrival check
                if size_frac >= target_arrival_size_frac:
                    arrival_time = time.monotonic() - t0
                    if kind == "red_target" and red_target_time is None:
                        red_target_time = arrival_time
                    elif kind == "blue_ring" and blue_gate_time is None:
                        blue_gate_time = arrival_time
                    print(
                        f"[yolo_explore] {kind} arrived at {arrival_time:.2f}s "
                        f"size_frac={size_frac:.3f} >= {target_arrival_size_frac:.3f}; hover"
                    )
                    client.hoverAsync().join()
                    break

                # Normal pursuit
                alignment = max(0.0, 1.0 - abs(nx))
                fwd_speed = _clamp(
                    target_approach_speed_ms * (0.35 + 0.65 * alignment), 0.0, max_v
                )
                yaw_rate = _clamp(target_yaw_gain_deg_s * nx, -120.0, 120.0)
                vz_pursue = _clamp(target_v_gain * ny, -1.2, 1.2)
                client.moveByVelocityAsync(
                    fwd_speed * cos_y,
                    fwd_speed * sin_y,
                    vz_pursue,
                    dt,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate)),
                ).join()
                if steps % max(1, int(rate_hz)) == 0:
                    print(
                        f"[yolo_explore] pursue {kind} nx={nx:+.2f} ny={ny:+.2f} "
                        f"size_frac={size_frac:.3f} fwd={fwd_speed:.2f} "
                        f"yaw_rate={yaw_rate:+.1f} vz={vz_pursue:+.1f}"
                    )
                steps += 1
                self._sleep_remaining(tick_start, dt)
                continue

            if time.monotonic() < scan_until_s:
                cos_s = math.cos(scan_yaw_rad)
                sin_s = math.sin(scan_yaw_rad)
                yaw_err_deg = math.degrees(
                    (scan_yaw_rad - yaw_rad + math.pi) % (2 * math.pi) - math.pi
                )
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
                        f"[yolo_explore] scan-straight "
                        f"({remaining:.1f}s left, fwd={post_flythrough_scan_speed_ms:.2f})"
                    )
                steps += 1
                self._sleep_remaining(tick_start, dt)
                continue

            time_since_target = time.monotonic() - last_target_seen_s
            if time_since_target > 2.0:
                if search_scan_offset == 0.0 and last_target_nx != 0.0:
                    search_direction = 1.0 if last_target_nx > 0 else -1.0
                search_scan_offset += search_direction * (35.0 * dt)
                if abs(search_scan_offset) > 30.0:
                    search_direction *= -1.0
                yaw_rate = search_scan_offset
                state_label = "SCANNING"
            elif time_since_target > 0.5:
                yaw_rate = 0.0
                state_label = "COASTING"
            else:
                yaw_rate = 0.0
                state_label = "cruise"

            client.moveByVelocityAsync(
                cruise_v * cos_y,
                cruise_v * sin_y,
                vz,
                dt,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate)),
            ).join()

            if steps % max(1, int(rate_hz)) == 0:
                print(
                    f"[yolo_explore] {state_label} fwd={cruise_v:.2f} "
                    f"yaw_rate={yaw_rate:+.1f}"
                )

            steps += 1
            self._sleep_remaining(tick_start, dt)

        print("[yolo_explore] duration elapsed; hover and land")
        client.hoverAsync().join()
        time.sleep(1.0)
        client.landAsync().join()

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
        print("[yolo_explore] done")

    @staticmethod
    def _sleep_remaining(tick_start_s: float, dt: float) -> None:
        remaining = dt - (time.monotonic() - tick_start_s)
        if remaining > 0:
            time.sleep(remaining)
