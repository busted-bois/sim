"""
Main entry point — connects to simulator and prints live telemetry.

Usage:
    # Terminal 1: start mock simulator
    python -m comms.mock_simulator

    # Terminal 2: run this
    python main.py
"""

import time

from comms.mavlink_client import TelemetryClient


def main() -> None:
    client = TelemetryClient(port=14550)
    client.start()

    print("Waiting for telemetry (start mock_simulator in another terminal)...")
    try:
        while True:
            state = client.get_state()
            if client.is_connected:
                print(
                    f"pos=({state.x:7.2f}, {state.y:7.2f}, {state.z:7.2f})  "
                    f"att=({state.roll:6.3f}, {state.pitch:6.3f}, {state.yaw:6.3f})  "
                    f"vel=({state.vx:6.2f}, {state.vy:6.2f}, {state.vz:6.2f})  "
                    f"armed={state.armed}"
                )
            else:
                print("No connection — waiting for heartbeat...")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        client.stop()


if __name__ == "__main__":
    main()

