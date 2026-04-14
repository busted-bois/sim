import os
import threading
import time

import airsim
from src.config import load_config
from src.control.algorithms import get_algorithm


def _run_landing(client: airsim.MultirotorClient, config: dict) -> None:
    landing_cfg = config.get("landing", {})
    profile = os.environ.get("AIGP_LANDING_PROFILE", "").strip() or landing_cfg.get(
        "profile", "faster_soft"
    )

    min_hover_seconds = max(0.5, float(landing_cfg.get("min_hover_seconds", 1.0)))
    max_descent_speed_ms = max(0.5, float(landing_cfg.get("max_descent_speed_ms", 2.5)))

    print(f"Landing profile: {profile}")
    client.hoverAsync().join()
    if min_hover_seconds > 0:
        print(f"Hover settle: {min_hover_seconds:.2f}s")
        time.sleep(min_hover_seconds)

    if profile == "very_soft":
        client.landAsync().join()
        return

    descent_speed_ms = max(0.5, float(landing_cfg.get("descent_speed_ms", 2.0)))
    descent_speed_ms = min(descent_speed_ms, max_descent_speed_ms)
    final_land_altitude_m = max(0.3, float(landing_cfg.get("final_land_altitude_m", 1.0)))

    state = client.getMultirotorState().kinematics_estimated
    altitude_m = max(0.0, -float(state.position.z_val))
    if altitude_m > final_land_altitude_m:
        descent_distance = altitude_m - final_land_altitude_m
        descent_duration_s = descent_distance / descent_speed_ms
        print(
            f"Controlled descent: speed={descent_speed_ms:.2f} m/s "
            f"for {descent_duration_s:.2f}s before final land."
        )
        # NED frame: +vz commands downward motion.
        client.moveByVelocityAsync(0.0, 0.0, descent_speed_ms, descent_duration_s).join()
        client.hoverAsync().join()

    client.landAsync().join()


def _run_algorithm_with_timeout(algo, client, timeout_seconds: float) -> None:
    error_holder: dict[str, BaseException] = {}

    def _target() -> None:
        try:
            algo.run(client)
        except BaseException as exc:  # noqa: BLE001
            error_holder["error"] = exc

    worker = threading.Thread(target=_target, name="algorithm_runner", daemon=True)
    worker.start()
    worker.join(timeout=timeout_seconds)

    if worker.is_alive():
        raise TimeoutError(f"Algorithm timed out after {timeout_seconds:.1f}s")

    if "error" in error_holder:
        raise RuntimeError("Algorithm raised an exception") from error_holder["error"]


def main() -> None:
    config = load_config()
    sim_cfg = config["simulator"]

    client = airsim.MultirotorClient(
        ip=sim_cfg.get("host", "127.0.0.1"),
        port=sim_cfg.get("airsim_port", 41451),
    )
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    try:
        algo_name = config.get("algorithm", "six_directions")
        algo = get_algorithm(algo_name, config)
        safety_cfg = config.get("safety", {})
        algo_timeout_seconds = max(5.0, float(safety_cfg.get("algorithm_timeout_seconds", 180.0)))

        print(f"Algorithm: {algo_name}")
        _run_algorithm_with_timeout(algo, client, algo_timeout_seconds)

        print("Algorithm complete. Starting landing sequence...")
        _run_landing(client, config)
    except Exception as exc:
        print(f"Failsafe triggered: {exc}")
        print("Attempting hover and landing for safe recovery...")
        _run_landing(client, config)
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)


if __name__ == "__main__":
    main()
