from __future__ import annotations

import json
import math
import time
from pathlib import Path

from src.control.algorithms import Algorithm, register

ROOT = Path(__file__).resolve().parents[3]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@register("attitude_four_motion")
class AttitudeFourMotion(Algorithm):
    def run(self, client):
        basic_flight_logs = bool(self._config.get("logging", {}).get("basic_flight_logs", False))
        control_cfg = self._config.get("control", {})
        vision_cfg = self._config.get("vision", {})
        rate_hz = float(control_cfg.get("command_rate_hz", 50.0))
        configured_rate_hz = rate_hz
        latency_tuning_cfg = control_cfg.get("latency_tuning", {})
        auto_tune_enabled = bool(latency_tuning_cfg.get("enabled", True))
        if auto_tune_enabled and vision_cfg.get("enabled", False):
            vision_fps = max(1.0, float(vision_cfg.get("fps", 20.0)))
            commands_per_frame = max(1.0, float(latency_tuning_cfg.get("commands_per_frame", 2.0)))
            max_command_rate_hz = float(
                latency_tuning_cfg.get("max_command_rate_hz", configured_rate_hz)
            )
            tuned_rate_hz = min(configured_rate_hz, vision_fps * commands_per_frame)
            rate_hz = min(max_command_rate_hz, tuned_rate_hz)
        rate_hz = max(5.0, rate_hz)
        dt = 1.0 / rate_hz

        max_altitude_m = float(control_cfg.get("max_altitude_m", 50.0))
        max_speed_ms = max(0.5, float(control_cfg.get("max_speed_ms", 10.0)))
        cruise_speed = min(2.5, max_speed_ms * 0.5)

        roll_max_deg = 12.0
        pitch_max_deg = 12.0
        throttle_trim = 0.63
        kp_z, ki_z, kd_z = 0.25, 0.035, 0.12
        kv_xy = 5.0

        z_integral = 0.0
        yaw_deg = 0.0
        roll_sign = 1.0
        pitch_sign = 1.0

        state = client.getMultirotorState().kinematics_estimated
        current_z = float(state.position.z_val)
        target_z = float(self._config.get("waypoints", [{"z": -5.0}])[0].get("z", -5.0))
        z_floor = -max_altitude_m
        z_ceiling = -0.5
        target_z = _clamp(target_z, z_floor, z_ceiling)
        if current_z > -1.0:
            target_z = min(target_z, -4.0)
        log_every_steps = max(1, int(rate_hz // 2))
        if basic_flight_logs:
            log_every_steps = 10_000_000

        print(
            "[attitude_four_motion] init "
            f"rate_hz={rate_hz:.1f} dt={dt:.3f} cruise_speed={cruise_speed:.2f} "
            f"target_z={target_z:.2f} z_bounds=[{z_floor:.2f},{z_ceiling:.2f}] "
            f"configured_rate_hz={configured_rate_hz:.1f}"
        )

        def apply_control(
            roll_deg: float,
            pitch_deg: float,
            yaw_cmd_deg: float,
            throttle: float,
        ) -> None:
            client.moveByRollPitchYawThrottleAsync(
                math.radians(roll_deg),
                math.radians(pitch_deg),
                math.radians(yaw_cmd_deg),
                _clamp(throttle, 0.0, 1.0),
                dt,
            ).join()

        def run_phase(
            label: str,
            vx_target: float,
            vy_target: float,
            z_ref: float,
            seconds: float,
        ) -> None:
            nonlocal z_integral
            steps = max(1, int(seconds * rate_hz))
            phase_started_s = time.perf_counter()
            loop_total_s = 0.0
            loop_max_s = 0.0
            loop_overruns = 0
            vision_start = self.vision_stats()
            print(
                f"[attitude_four_motion] phase_start name={label} duration_s={seconds:.2f} "
                f"steps={steps} targets(vx,vy,z)=({vx_target:.2f},{vy_target:.2f},{z_ref:.2f})"
            )
            for step in range(steps):
                loop_started_s = time.perf_counter()
                s = client.getMultirotorState().kinematics_estimated
                x = float(s.position.x_val)
                y = float(s.position.y_val)
                vx = float(s.linear_velocity.x_val)
                vy = float(s.linear_velocity.y_val)
                vz = float(s.linear_velocity.z_val)
                z = float(s.position.z_val)

                ex_z = z - z_ref
                ev_z = vz
                z_integral = _clamp(z_integral + ex_z * dt, -6.0, 6.0)
                throttle = throttle_trim + (kp_z * ex_z) + (ki_z * z_integral) + (kd_z * ev_z)

                pitch_cmd = kv_xy * (vx_target - vx)
                roll_cmd = kv_xy * (vy_target - vy)
                pitch_deg = _clamp(pitch_sign * pitch_cmd, -pitch_max_deg, pitch_max_deg)
                roll_deg = _clamp(roll_sign * roll_cmd, -roll_max_deg, roll_max_deg)
                apply_control(roll_deg, pitch_deg, yaw_deg, throttle)

                should_log_step = step == 0 or step == steps - 1 or step % log_every_steps == 0
                if should_log_step and not basic_flight_logs:
                    frame = self.latest_frame()
                    frame_log = ""
                    if frame is not None:
                        frame_log = (
                            f" frame(seq={frame.seq},age_ms={frame.frame_age_s * 1000.0:.1f},"
                            f"shape={frame.height}x{frame.width})"
                        )
                    loop_elapsed_ms = (time.perf_counter() - loop_started_s) * 1000.0
                    print(
                        f"[attitude_four_motion] phase={label} step={step + 1}/{steps} "
                        f"pos=({x:.2f},{y:.2f},{z:.2f}) vel=({vx:.2f},{vy:.2f},{vz:.2f}) "
                        f"cmd(r,p,y,t)=({roll_deg:.2f},{pitch_deg:.2f},{yaw_deg:.2f},"
                        f"{_clamp(throttle, 0.0, 1.0):.2f}) "
                        f"loop_ms={loop_elapsed_ms:.2f}{frame_log}"
                    )
                loop_elapsed_s = time.perf_counter() - loop_started_s
                loop_total_s += loop_elapsed_s
                loop_max_s = max(loop_max_s, loop_elapsed_s)
                if loop_elapsed_s > dt * 1.10:
                    loop_overruns += 1

            s_end = client.getMultirotorState().kinematics_estimated
            phase_elapsed_s = max(1e-6, time.perf_counter() - phase_started_s)
            avg_loop_ms = (loop_total_s / steps) * 1000.0
            achieved_hz = steps / phase_elapsed_s
            vision_end = self.vision_stats()
            vision_log = ""
            if vision_start is not None and vision_end is not None:
                produced = vision_end.capture_successes - vision_start.capture_successes
                dropped_sched = (
                    vision_end.scheduler_dropped_ticks - vision_start.scheduler_dropped_ticks
                )
                dropped_consumer = (
                    vision_end.consumer_dropped_frames - vision_start.consumer_dropped_frames
                )
                vision_log = (
                    f" vision(captured={produced},sched_drop={dropped_sched},"
                    "consumer_drop="
                    f"{dropped_consumer},"
                    f"fps(cfg={vision_end.configured_fps:.1f},active={vision_end.active_fps:.1f}),"
                    f"frame_age_ms={vision_end.latest_frame_age_s * 1000.0:.1f},"
                    f"cap_hz={vision_end.effective_capture_hz:.1f})"
                )
            print(
                f"[attitude_four_motion] phase_end name={label} "
                "pos=("
                f"{float(s_end.position.x_val):.2f},"
                f"{float(s_end.position.y_val):.2f},"
                f"{float(s_end.position.z_val):.2f}) "
                f"timing(avg_ms={avg_loop_ms:.2f},max_ms={loop_max_s * 1000.0:.2f},"
                f"overruns={loop_overruns},hz={achieved_hz:.1f}){vision_log}"
            )

        client.takeoffAsync().join()

        run_phase("stabilize", 0.0, 0.0, target_z, 2.5)
        self._run_latency_auto_tuner(
            client=client,
            current_rate_hz=rate_hz,
            vision_cfg=vision_cfg,
            latency_tuning_cfg=latency_tuning_cfg,
        )

        x0 = float(client.getMultirotorState().kinematics_estimated.position.x_val)
        run_phase("cal_pitch", 1.2, 0.0, target_z, 0.8)
        x1 = float(client.getMultirotorState().kinematics_estimated.position.x_val)
        if x1 < x0:
            pitch_sign = -1.0
        print(
            "[attitude_four_motion] calibration "
            f"pitch_sign={pitch_sign:+.0f} x0={x0:.2f} x1={x1:.2f}"
        )

        y0 = float(client.getMultirotorState().kinematics_estimated.position.y_val)
        run_phase("cal_roll", 0.0, 1.2, target_z, 0.8)
        y1 = float(client.getMultirotorState().kinematics_estimated.position.y_val)
        if y1 < y0:
            roll_sign = -1.0
        print(
            f"[attitude_four_motion] calibration roll_sign={roll_sign:+.0f} y0={y0:.2f} y1={y1:.2f}"
        )

        run_phase("+X", cruise_speed, 0.0, target_z, 3.0)
        run_phase("-X", -cruise_speed, 0.0, target_z, 3.0)
        run_phase("+Y", 0.0, cruise_speed, target_z, 3.0)
        run_phase("-Y", 0.0, -cruise_speed, target_z, 3.0)
        run_phase("+Z", 0.0, 0.0, _clamp(target_z - 1.5, z_floor, z_ceiling), 3.0)
        run_phase("-Z", 0.0, 0.0, target_z, 3.0)

        print("[attitude_four_motion] Completed 6-segment attitude routine")

    def _run_latency_auto_tuner(
        self,
        client,
        current_rate_hz: float,
        vision_cfg: dict,
        latency_tuning_cfg: dict,
    ) -> None:
        autotuner_cfg = latency_tuning_cfg.get("autotuner", {})
        if not bool(autotuner_cfg.get("enabled", True)):
            return
        duration_s = max(1.0, float(autotuner_cfg.get("duration_seconds", 8.0)))
        warmup_s = max(0.0, float(autotuner_cfg.get("warmup_seconds", 1.0)))
        commands_per_frame = max(1.0, float(latency_tuning_cfg.get("commands_per_frame", 2.0)))
        max_command_rate_hz = max(
            5.0, float(latency_tuning_cfg.get("max_command_rate_hz", current_rate_hz))
        )
        max_vision_fps = max(10.0, float(autotuner_cfg.get("max_vision_fps", 40.0)))
        vision_enabled = bool(vision_cfg.get("enabled", False))

        dt = 1.0 / max(5.0, current_rate_hz)
        print(
            "[attitude_four_motion] autotuner_start "
            f"duration_s={duration_s:.1f} warmup_s={warmup_s:.1f} "
            f"rate_hz={current_rate_hz:.1f}"
        )
        if warmup_s > 0:
            time.sleep(warmup_s)

        vision_start = self.vision_stats()
        started_s = time.perf_counter()
        next_tick = started_s
        steps = 0
        overruns = 0
        while time.perf_counter() - started_s < duration_s:
            loop_start = time.perf_counter()
            client.getMultirotorState()
            _ = self.latest_frame()
            steps += 1
            loop_elapsed_s = time.perf_counter() - loop_start
            if loop_elapsed_s > dt * 1.10:
                overruns += 1
            next_tick += dt
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

        elapsed_s = max(1e-6, time.perf_counter() - started_s)
        achieved_hz = steps / elapsed_s
        overrun_ratio = overruns / max(1, steps)

        measured_capture_hz = 0.0
        scheduler_drop_ratio = 0.0
        consumer_drop_ratio = 0.0
        if vision_enabled:
            vision_end = self.vision_stats()
            if vision_start is not None and vision_end is not None:
                captured = max(0, vision_end.capture_successes - vision_start.capture_successes)
                scheduler_drops = max(
                    0,
                    vision_end.scheduler_dropped_ticks - vision_start.scheduler_dropped_ticks,
                )
                consumer_drops = max(
                    0,
                    vision_end.consumer_dropped_frames - vision_start.consumer_dropped_frames,
                )
                measured_capture_hz = captured / elapsed_s
                scheduler_drop_ratio = scheduler_drops / max(1, captured + scheduler_drops)
                consumer_drop_ratio = consumer_drops / max(1, captured + consumer_drops)

        baseline_capture_hz = measured_capture_hz
        if baseline_capture_hz <= 0:
            baseline_capture_hz = max(1.0, float(vision_cfg.get("fps", 20.0)))

        recommended_command_hz = min(max_command_rate_hz, baseline_capture_hz * commands_per_frame)
        if overrun_ratio > 0.10:
            recommended_command_hz *= 0.85
        recommended_command_hz = max(5.0, recommended_command_hz)

        recommended_vision_fps = baseline_capture_hz
        if scheduler_drop_ratio > 0.05:
            recommended_vision_fps *= 0.90
        elif overrun_ratio < 0.02:
            recommended_vision_fps = max(
                recommended_vision_fps,
                min(max_vision_fps, recommended_command_hz / commands_per_frame),
            )
        recommended_vision_fps = _clamp(recommended_vision_fps, 10.0, max_vision_fps)

        print(
            "[attitude_four_motion] autotuner_result "
            f"measured(cmd_hz={achieved_hz:.1f},capture_hz={baseline_capture_hz:.1f},"
            f"overrun_ratio={overrun_ratio:.3f},sched_drop_ratio={scheduler_drop_ratio:.3f},"
            f"consumer_drop_ratio={consumer_drop_ratio:.3f}) "
            f"recommend(vision.fps={recommended_vision_fps:.1f},"
            f"control.command_rate_hz={recommended_command_hz:.1f})"
        )
        output_cfg = autotuner_cfg.get("output_json", {})
        if bool(output_cfg.get("enabled", False)):
            output_path = Path(
                str(
                    output_cfg.get(
                        "path",
                        "logs/latency_tuning_recommendation.json",
                    )
                )
            )
            if not output_path.is_absolute():
                output_path = ROOT / output_path
            payload = {
                "timestamp_unix_s": time.time(),
                "algorithm": "attitude_four_motion",
                "autotuner": {
                    "duration_seconds": duration_s,
                    "warmup_seconds": warmup_s,
                    "max_vision_fps": max_vision_fps,
                },
                "measured": {
                    "control_hz": achieved_hz,
                    "capture_hz": baseline_capture_hz,
                    "overrun_ratio": overrun_ratio,
                    "scheduler_drop_ratio": scheduler_drop_ratio,
                    "consumer_drop_ratio": consumer_drop_ratio,
                },
                "recommendation": {
                    "vision_fps": recommended_vision_fps,
                    "command_rate_hz": recommended_command_hz,
                },
            }
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            print(f"[attitude_four_motion] autotuner_output path={output_path}")
