"""AIGP Drone — main entry point."""

import signal
import sys
import time

from src.comms.telemetry import TelemetryClient
from src.config import load_config


def main() -> None:
    config = load_config()
    telem_cfg = config["telemetry"]

    client = TelemetryClient(
        host=telem_cfg["host"],
        port=telem_cfg["port"],
        timeout=telem_cfg["timeout_seconds"],
    )

    def shutdown(sig, frame):
        print("\nShutting down...")
        client.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    client.start()
    print(f"Listening on {telem_cfg['host']}:{telem_cfg['port']}...")

    while True:
        if client.is_connected:
            state = client.get_state()
            stats = client.get_packet_stats()
            hb = client.get_heartbeat_stats()
            print(
                f"pos=({state.x:.1f}, {state.y:.1f}, {state.z:.1f}) "
                f"att=({state.roll:.1f}, {state.pitch:.1f}, {state.yaw:.1f}) "
                f"armed={state.armed} "
                f"pkts={stats['received']} hb_rx={hb.received}"
            )
        else:
            print("Waiting for heartbeat...")
        time.sleep(1)


if __name__ == "__main__":
    main()
