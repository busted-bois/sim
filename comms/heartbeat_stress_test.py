"""
Heartbeat reliability stress test.

Runs for a configurable duration (default 8 minutes) against the mock simulator
and logs:
- Dropped heartbeats
- Timing drift
- Packet loss
- Per-message-type counts

Usage:
    # Terminal 1: start the mock simulator
    python -m comms.mock_simulator --port 14550

    # Terminal 2: run this stress test
    python -m comms.heartbeat_stress_test [--port 14550] [--duration 480]
"""

import argparse
import time

from comms.mavlink_client import TelemetryClient


def run_stress_test(port: int = 14550, duration_s: int = 480):
    """Run heartbeat reliability stress test."""
    print(f"[stress_test] Starting {duration_s}s heartbeat stress test on port {port}")
    print(f"[stress_test] Expected duration: {duration_s // 60}m {duration_s % 60}s")
    print("-" * 60)

    client = TelemetryClient(port=port)
    client.start()

    start = time.monotonic()
    report_interval = 30  # print report every 30s
    last_report = start

    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= duration_s:
                break

            # Periodic report
            if time.monotonic() - last_report >= report_interval:
                _print_report(client, elapsed)
                last_report = time.monotonic()

            time.sleep(1.0)

    except KeyboardInterrupt:
        elapsed = time.monotonic() - start
        print(f"\n[stress_test] Interrupted after {elapsed:.1f}s")

    finally:
        client.stop()

    # Final report
    elapsed = time.monotonic() - start
    print("\n" + "=" * 60)
    print("FINAL STRESS TEST REPORT")
    print("=" * 60)
    _print_report(client, elapsed, final=True)


def _print_report(client: TelemetryClient, elapsed: float, final: bool = False):
    hb = client.get_heartbeat_stats()
    pkt = client.get_packet_stats()
    state = client.get_state()

    expected_heartbeats = max(1, int(elapsed))  # 1 Hz expected
    loss_pct = ((expected_heartbeats - hb.received) / expected_heartbeats * 100
                if expected_heartbeats > 0 else 0)

    print(f"\n[t={elapsed:.0f}s] {'FINAL' if final else 'Periodic'} Report")
    print(f"  Connected: {client.is_connected}")
    print(f"  Heartbeats: {hb.received}/{expected_heartbeats} "
          f"(loss: {loss_pct:.1f}%)")
    print(f"  Missed heartbeats (gap > 1.5s): {hb.missed}")
    print(f"  Max heartbeat gap: {hb.max_gap_s:.3f}s")
    print(f"  Total packets: {pkt['received']} decoded, "
          f"{pkt['failed_decode']} failed")
    print(f"  Messages by type: {pkt['by_type']}")
    print(f"  Last position: ({state.x:.2f}, {state.y:.2f}, {state.z:.2f})")
    print(f"  Armed: {state.armed}, Status: {state.system_status}")

    if hb.gaps and final:
        print(f"\n  Heartbeat gaps (>{1.5}s):")
        for ts, gap in hb.gaps[-20:]:  # last 20 gaps
            print(f"    at t={ts:.1f}s: gap={gap:.3f}s")

    if final:
        total_expected_msgs = int(elapsed) * 50 * 3 + expected_heartbeats  # att+imu+odom at 50Hz + hb at 1Hz
        actual = pkt['received']
        overall_loss = ((total_expected_msgs - actual) / total_expected_msgs * 100
                        if total_expected_msgs > 0 else 0)
        print(f"\n  Overall packet loss estimate: {overall_loss:.1f}% "
              f"({actual}/{total_expected_msgs})")
        if hb.missed == 0 and loss_pct < 1:
            print("  RESULT: PASS - Heartbeat reliability is solid")
        else:
            print("  RESULT: ISSUES DETECTED - Review gaps above")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Heartbeat stress test")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--duration", type=int, default=480,
                        help="Test duration in seconds (default: 480 = 8 minutes)")
    args = parser.parse_args()
    run_stress_test(args.port, args.duration)
