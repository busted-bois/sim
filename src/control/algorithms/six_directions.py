import sys
import time

from src.control.algorithms import Algorithm, register
from src.control.flight_client import (
    SET_POSITION_FRAME_LOCAL_NED,
    FlightClient,
    SetPositionTargetLocalNedCommand,
    build_velocity_type_mask,
)
from src.control.primitives import takeoff_with_settle

DIRECTIONS = [
    ("+X", 2.0, 0.0, 0.0),
    ("-X", -2.0, 0.0, 0.0),
    ("+Y", 0.0, 2.0, 0.0),
    ("-Y", 0.0, -2.0, 0.0),
    ("+Z", 0.0, 0.0, 2.0),
    ("-Z", 0.0, 0.0, -2.0),
]
DURATION_S = 5.0
SPEED_MS = 2.0


@register("six_directions")
class SixDirections(Algorithm):
    config_section = "six_directions"

    def run(self, client: FlightClient):
        cfg = self._config.get("six_directions", {})
        control_cfg = self._config.get("control", {})
        max_speed_ms = max(0.5, float(control_cfg.get("max_speed_ms", SPEED_MS)))
        duration_s = max(0.5, float(cfg.get("duration_s", DURATION_S)))
        speed_ms = max(0.5, min(float(cfg.get("speed_ms", SPEED_MS)), max_speed_ms))
        direction_labels = cfg.get("direction_labels", [name for name, *_ in DIRECTIONS])
        selected = [d for d in DIRECTIONS if d[0] in set(direction_labels)]
        if not selected:
            selected = DIRECTIONS
        mavlink_setpoint_demo_enabled = bool(cfg.get("mavlink_setpoint_demo_enabled", False))
        stream_local_ned = getattr(client, "streamSetPositionTargetLocalNedAsync", None)
        submit_local_ned = getattr(client, "submitSetPositionTargetLocalNed", None)
        cmd_hz = max(0.5, float(control_cfg.get("command_rate_hz", 50.0)))
        submit_cap_hz: float | None = None
        raw_submit_hz = cfg.get("mavlink_submit_hz")
        if raw_submit_hz is not None and callable(submit_local_ned):
            candidate = float(raw_submit_hz)
            if candidate > 0.0:
                submit_cap_hz = candidate

        takeoff_with_settle(client, max_attempts=4, label="six_directions")

        for label, vx, vy, vz in selected:
            print(f"[six_directions] Moving {label} for {duration_s:.1f}s")
            t0 = time.perf_counter()
            vx_cmd = vx * speed_ms / SPEED_MS
            vy_cmd = vy * speed_ms / SPEED_MS
            vz_cmd = vz * speed_ms / SPEED_MS
            if mavlink_setpoint_demo_enabled:
                command = SetPositionTargetLocalNedCommand(
                    frame=SET_POSITION_FRAME_LOCAL_NED,
                    type_mask=build_velocity_type_mask(),
                    vx=vx_cmd,
                    vy=vy_cmd,
                    vz=vz_cmd,
                )
                if submit_cap_hz is not None:
                    rate_hz = max(0.5, min(cmd_hz, submit_cap_hz, 99.0))
                    period_s = 1.0 / rate_hz
                    deadline = time.perf_counter() + duration_s
                    next_tick = time.perf_counter()
                    while time.perf_counter() < deadline:
                        submit_local_ned(command)
                        next_tick += period_s
                        sleep_s = next_tick - time.perf_counter()
                        if sleep_s > 0:
                            time.sleep(sleep_s)
                        else:
                            next_tick = time.perf_counter()
                elif callable(stream_local_ned):
                    stream_local_ned(command, duration_s).join()
                else:
                    client.moveByVelocityAsync(vx_cmd, vy_cmd, vz_cmd, duration_s).join()
            else:
                client.moveByVelocityAsync(vx_cmd, vy_cmd, vz_cmd, duration_s).join()
            elapsed = time.perf_counter() - t0
            shortfall = duration_s - elapsed
            if shortfall > 1e-3:
                print(
                    f"[six_directions] segment={label} wall time {elapsed:.2f}s "
                    f"< requested {duration_s:.1f}s; padding {shortfall:.2f}s "
                    "(if unexpected, check Unreal is unpaused and simulation is real-time).",
                    file=sys.stderr,
                )
                time.sleep(shortfall)

        print("[six_directions] Complete — hovering")
        client.hoverAsync().join()
