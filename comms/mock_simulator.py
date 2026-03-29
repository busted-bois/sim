"""
Mock simulator — pretends to be the UE5 AIGP drone simulator.

Sends fake MAVLink telemetry over UDP so the client code can be tested
end-to-end without Unreal Engine.

Usage:
    python -m comms.mock_simulator [--host 127.0.0.1] [--port 14550] [--hz 50]

The simulator generates:
- HEARTBEAT at 1 Hz
- ATTITUDE at the requested rate (default 50 Hz)
- HIGHRES_IMU at the requested rate
- ODOMETRY at the requested rate
- TIMESYNC at 1 Hz
"""

import argparse
import math
import random
import socket
import time

from comms.mavlink_parser import (
    Attitude,
    Heartbeat,
    HighresImu,
    Odometry,
    Timesync,
    encode,
)


def _sim_time_ms(start: float) -> int:
    return int((time.monotonic() - start) * 1000)


def _sim_time_us(start: float) -> int:
    return int((time.monotonic() - start) * 1_000_000)


def run_simulator(host: str = "127.0.0.1", port: int = 14550, hz: int = 50):
    """Run the mock simulator loop, sending telemetry to the given UDP target."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (host, port)
    dt = 1.0 / hz
    start = time.monotonic()

    # Simulated drone state
    t = 0.0
    x, y, z = 0.0, 0.0, -10.0  # NED, 10m altitude
    vx, vy, vz = 0.0, 0.0, 0.0

    print(f"[mock_sim] Sending telemetry to {host}:{port} at {hz} Hz")
    print("[mock_sim] Press Ctrl+C to stop")

    heartbeat_interval = 1.0
    timesync_interval = 1.0
    last_heartbeat = 0.0
    last_timesync = 0.0
    seq = 0

    try:
        while True:
            now = time.monotonic()
            t = now - start

            # --- Heartbeat at 1 Hz ---
            if now - last_heartbeat >= heartbeat_interval:
                hb = Heartbeat(
                    custom_mode=0,
                    type=2,          # MAV_TYPE_QUADROTOR
                    autopilot=12,    # MAV_AUTOPILOT_PX4
                    base_mode=209,   # armed + custom mode
                    system_status=4, # MAV_STATE_ACTIVE
                    mavlink_version=3,
                )
                sock.sendto(encode(hb), target)
                last_heartbeat = now

            # --- Timesync at 1 Hz ---
            if now - last_timesync >= timesync_interval:
                ts = Timesync(tc1=0, ts1=int(time.time() * 1e9))
                sock.sendto(encode(ts), target)
                last_timesync = now

            # --- Simulate gentle circular flight ---
            radius = 20.0
            omega = 0.1  # rad/s
            x = radius * math.cos(omega * t)
            y = radius * math.sin(omega * t)
            z = -10.0 + 2.0 * math.sin(0.05 * t)  # gentle altitude oscillation
            vx = -radius * omega * math.sin(omega * t)
            vy = radius * omega * math.cos(omega * t)
            vz = 2.0 * 0.05 * math.cos(0.05 * t)

            roll = 0.05 * math.sin(0.3 * t)
            pitch = 0.03 * math.cos(0.2 * t)
            yaw = omega * t

            # --- Attitude ---
            att = Attitude(
                time_boot_ms=_sim_time_ms(start),
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                rollspeed=0.05 * 0.3 * math.cos(0.3 * t),
                pitchspeed=-0.03 * 0.2 * math.sin(0.2 * t),
                yawspeed=omega,
            )
            sock.sendto(encode(att), target)

            # --- HIGHRES_IMU ---
            noise = lambda: random.gauss(0, 0.01)
            imu = HighresImu(
                time_usec=_sim_time_us(start),
                xacc=-9.81 * math.sin(pitch) + noise(),
                yacc=9.81 * math.sin(roll) + noise(),
                zacc=-9.81 * math.cos(roll) * math.cos(pitch) + noise(),
                xgyro=att.rollspeed + noise(),
                ygyro=att.pitchspeed + noise(),
                zgyro=att.yawspeed + noise(),
                xmag=0.2 + noise(),
                ymag=0.0 + noise(),
                zmag=0.4 + noise(),
                abs_pressure=1013.25 + z * 0.12,
                diff_pressure=0.5 * 1.225 * (vx**2 + vy**2) * 0.01,
                pressure_alt=-z,
                temperature=25.0 + noise(),
                fields_updated=0xFFFF,
            )
            sock.sendto(encode(imu), target)

            # --- Odometry ---
            # Simple quaternion from euler (small angles)
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy

            odom = Odometry(
                time_usec=_sim_time_us(start),
                frame_id=1,        # MAV_FRAME_LOCAL_NED
                child_frame_id=1,
                x=x, y=y, z=z,
                q=(qw, qx, qy, qz),
                vx=vx, vy=vy, vz=vz,
                rollspeed=att.rollspeed,
                pitchspeed=att.pitchspeed,
                yawspeed=att.yawspeed,
            )
            sock.sendto(encode(odom), target)

            seq += 1
            if seq % (hz * 5) == 0:
                print(f"[mock_sim] t={t:.1f}s  pos=({x:.1f}, {y:.1f}, {z:.1f})  "
                      f"att=({math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, "
                      f"{math.degrees(yaw):.1f}) deg")

            # Sleep to maintain target rate
            elapsed = time.monotonic() - now
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\n[mock_sim] Stopped after {t:.1f}s, {seq} frames sent.")
    finally:
        sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AIGP Mock Drone Simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--hz", type=int, default=50)
    args = parser.parse_args()
    run_simulator(args.host, args.port, args.hz)
