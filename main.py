import signal
import socket
import sys
import time

from src.comms.command import arm_drone, send_velocity, set_offboard_mode
from src.comms.telemetry import TelemetryClient
from src.config import load_config
from src.control.algorithms import get_algorithm


def main() -> None:
    config = load_config()
    telem_cfg = config["telemetry"]
    cmd_cfg = config.get("command", {})
    ctrl_cfg = config.get("control", {})
    algo_name = config.get("algorithm", "six_directions")

    rate_hz = ctrl_cfg.get("command_rate_hz", 50)
    tick_s = 1.0 / rate_hz

    cmd_host = cmd_cfg.get("host", "127.0.0.1")
    cmd_port = cmd_cfg.get("port", 14540)

    client = TelemetryClient(
        host=telem_cfg["host"],
        port=telem_cfg["port"],
        timeout=telem_cfg["timeout_seconds"],
    )

    algo = get_algorithm(algo_name, config)
    print(f"Algorithm: {algo_name} @ {rate_hz}Hz -> {cmd_host}:{cmd_port}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def shutdown(sig, frame):
        print("\nShutting down...")
        send_velocity(0, 0, 0, sock=sock, target_address=cmd_host, port=cmd_port)
        client.stop()
        sock.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    client.start()
    print("Waiting for connection...")

    while not client.is_connected:
        print("Waiting for heartbeat...")
        time.sleep(0.5)

    print("Connected. Arming drone...")
    arm_drone(target_address=cmd_host, port=cmd_port, sock=sock)
    time.sleep(0.5)

    print("Setting OFFBOARD mode...")
    set_offboard_mode(target_address=cmd_host, port=cmd_port, sock=sock)
    time.sleep(0.5)

    print("Starting control loop")

    last_print = 0.0
    tick_count = 0

    while True:
        t0 = time.monotonic()
        state = client.get_state()

        cmd = algo.compute(state)
        if cmd is not None:
            send_velocity(
                cmd.vx,
                cmd.vy,
                cmd.vz,
                cmd.yaw,
                sock=sock,
                target_address=cmd_host,
                port=cmd_port,
            )

        tick_count += 1
        now = time.monotonic()
        if now - last_print >= 1.0:
            print(
                f"pos=({state.x:.1f}, {state.y:.1f}, {state.z:.1f}) "
                f"vel=({state.vx:.1f}, {state.vy:.1f}, {state.vz:.1f}) "
                f"armed={state.armed} ticks={tick_count}"
            )
            tick_count = 0
            last_print = now

        elapsed = time.monotonic() - t0
        sleep_time = tick_s - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
