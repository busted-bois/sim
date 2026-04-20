"""Launch script — thin wrapper; logic lives in src.sim_launch."""

import src.sim_launch


def main() -> None:
    src.sim_launch.launch()


if __name__ == "__main__":
    main()
