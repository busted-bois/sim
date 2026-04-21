import sys
import time

from src.control.algorithms import Algorithm, register

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
    def run(self, client):
        cfg = self._config.get("six_directions", {})
        control_cfg = self._config.get("control", {})
        max_speed_ms = max(0.5, float(control_cfg.get("max_speed_ms", SPEED_MS)))
        duration_s = max(0.5, float(cfg.get("duration_s", DURATION_S)))
        speed_ms = max(0.5, min(float(cfg.get("speed_ms", SPEED_MS)), max_speed_ms))
        direction_labels = cfg.get("direction_labels", [name for name, *_ in DIRECTIONS])
        selected = [d for d in DIRECTIONS if d[0] in set(direction_labels)]
        if not selected:
            selected = DIRECTIONS

        takeoff_attempts = 4
        for attempt in range(1, takeoff_attempts + 1):
            try:
                client.takeoffAsync().join()
                break
            except Exception as exc:
                if attempt == takeoff_attempts:
                    raise
                print(
                    f"[six_directions] takeoff attempt {attempt}/{takeoff_attempts} "
                    f"failed ({type(exc).__name__}: {exc}); retrying...",
                    file=sys.stderr,
                )
                time.sleep(1.5 * attempt)

        for label, vx, vy, vz in selected:
            print(f"[six_directions] Moving {label} for {duration_s:.1f}s")
            t0 = time.perf_counter()
            client.moveByVelocityAsync(
                vx * speed_ms / SPEED_MS,
                vy * speed_ms / SPEED_MS,
                vz * speed_ms / SPEED_MS,
                duration_s,
            ).join()
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
