from __future__ import annotations

import math

from src.control.algorithms import Algorithm, register


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@register("attitude_four_motion")
class AttitudeFourMotion(Algorithm):
    def run(self, client):
        control_cfg = self._config.get("control", {})
        rate_hz = float(control_cfg.get("command_rate_hz", 50.0))
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

        print(
            "[attitude_four_motion] init "
            f"rate_hz={rate_hz:.1f} dt={dt:.3f} cruise_speed={cruise_speed:.2f} "
            f"target_z={target_z:.2f} z_bounds=[{z_floor:.2f},{z_ceiling:.2f}]"
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
            print(
                f"[attitude_four_motion] phase_start name={label} duration_s={seconds:.2f} "
                f"steps={steps} targets(vx,vy,z)=({vx_target:.2f},{vy_target:.2f},{z_ref:.2f})"
            )
            for step in range(steps):
                s = client.getMultirotorState().kinematics_estimated
                x = float(s.position.x_val)
                y = float(s.position.y_val)
                vx = float(s.linear_velocity.x_val)
                vy = float(s.linear_velocity.y_val)
                vz = float(s.linear_velocity.z_val)
                z = float(s.position.z_val)

                # AirSim uses NED: more negative z means higher altitude.
                # Positive error should increase throttle when we need to climb.
                ex_z = z - z_ref
                ev_z = vz
                z_integral = _clamp(z_integral + ex_z * dt, -6.0, 6.0)
                throttle = throttle_trim + (kp_z * ex_z) + (ki_z * z_integral) + (kd_z * ev_z)

                pitch_cmd = kv_xy * (vx_target - vx)
                roll_cmd = kv_xy * (vy_target - vy)
                pitch_deg = _clamp(pitch_sign * pitch_cmd, -pitch_max_deg, pitch_max_deg)
                roll_deg = _clamp(roll_sign * roll_cmd, -roll_max_deg, roll_max_deg)
                apply_control(roll_deg, pitch_deg, yaw_deg, throttle)

                if step == 0 or step == steps - 1 or step % log_every_steps == 0:
                    print(
                        f"[attitude_four_motion] phase={label} step={step + 1}/{steps} "
                        f"pos=({x:.2f},{y:.2f},{z:.2f}) vel=({vx:.2f},{vy:.2f},{vz:.2f}) "
                        f"cmd(r,p,y,t)=({roll_deg:.2f},{pitch_deg:.2f},{yaw_deg:.2f},"
                        f"{_clamp(throttle, 0.0, 1.0):.2f})"
                    )

            s_end = client.getMultirotorState().kinematics_estimated
            print(
                f"[attitude_four_motion] phase_end name={label} "
                f"pos=({float(s_end.position.x_val):.2f},{float(s_end.position.y_val):.2f},{float(s_end.position.z_val):.2f})"
            )

        client.takeoffAsync().join()

        # Settle at target altitude with no lateral motion.
        run_phase("stabilize", 0.0, 0.0, target_z, 2.5)

        # Sign calibration so command polarity matches observed body response.
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
            "[attitude_four_motion] calibration "
            f"roll_sign={roll_sign:+.0f} y0={y0:.2f} y1={y1:.2f}"
        )

        # Six-direction style motion using RPYT only.
        run_phase("+X", cruise_speed, 0.0, target_z, 3.0)
        run_phase("-X", -cruise_speed, 0.0, target_z, 3.0)
        run_phase("+Y", 0.0, cruise_speed, target_z, 3.0)
        run_phase("-Y", 0.0, -cruise_speed, target_z, 3.0)
        run_phase("+Z", 0.0, 0.0, _clamp(target_z - 1.5, z_floor, z_ceiling), 3.0)
        run_phase("-Z", 0.0, 0.0, target_z, 3.0)

        print("[attitude_four_motion] Completed 6-segment attitude routine")
