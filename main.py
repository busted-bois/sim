import airsim
from src.config import load_config
from src.control.algorithms import get_algorithm


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

        print("Algorithm complete. Stabilizing before landing...")
        client.hoverAsync().join()
        client.landAsync().join()
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)


if __name__ == "__main__":
    main()
