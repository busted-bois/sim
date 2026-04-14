import os

import airsim
from src.config import load_config
from src.control.algorithms import get_algorithm


def _run_landing(client: airsim.MultirotorClient, config: dict) -> None:
    landing_cfg = config.get("landing", {})
    profile = os.environ.get("AIGP_LANDING_PROFILE", "").strip() or landing_cfg.get(
        "profile", "faster_soft"
    )

    print(f"Landing profile: {profile}")
    client.hoverAsync().join()

    if profile == "very_soft":
        client.landAsync().join()
        return

    descent_speed_ms = max(0.5, float(landing_cfg.get("descent_speed_ms", 2.0)))
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
        print(f"Algorithm: {algo_name}")
        algo.run(client)

        print("Algorithm complete. Starting landing sequence...")
        _run_landing(client, config)
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)


if __name__ == "__main__":
    main()
