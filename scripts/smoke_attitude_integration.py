"""Smoke: decode, bridge, UDP inject, attitude-listen CLI."""

from __future__ import annotations

import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.config import load_config  # noqa: E402
from src.mavlink.attitude import decode_attitude_payload, roll_pitch_yaw_deg  # noqa: E402
from src.mavlink.attitude_bridge import AttitudeTelemetryBridge  # noqa: E402
from src.mavlink.attitude_store import AttitudeStore  # noqa: E402
from src.mavlink.config import load_attitude_mavlink_config  # noqa: E402
from src.mavlink.frame import MAVLINK_V2_STX, frame_payload, parse_mavlink  # noqa: E402
from src.mavlink.messages import MAVLINK_MSG_ID_ATTITUDE  # noqa: E402
from tests.mavlink_fakes import FakeMav, FakeMessage  # noqa: E402

SMOKE_PORT = 14598


def _v2_attitude_frame(
    roll: float = 0.12,
    pitch: float = -0.08,
    yaw: float = 0.45,
) -> bytes:
    payload = struct.pack("<Iffffff", 9000, roll, pitch, yaw, 0.02, -0.01, 0.03)
    plen = len(payload)
    msgid_b = bytes([MAVLINK_MSG_ID_ATTITUDE & 0xFF, 0, 0])
    return bytes([MAVLINK_V2_STX, plen, 0, 0, 0, 1, 1, *msgid_b]) + payload + b"\x00\x00"


def _layer1_smoke() -> None:
    payload = struct.pack("<Iffffff", 5000, 0.05, -0.1, 1.0, 0.0, 0.0, 0.0)
    sample = decode_attitude_payload(payload)
    if sample is None:
        raise SystemExit("Layer1 FAIL: decode returned None")
    if abs(sample.roll - 0.05) > 1e-5:
        raise SystemExit(f"Layer1 FAIL: roll={sample.roll}")
    r, p, y = roll_pitch_yaw_deg(sample)
    print(f"  Layer1 decode OK: rpy_deg=({r:.2f}, {p:.2f}, {y:.2f})")


def _layer2_smoke() -> None:
    store = AttitudeStore()
    bridge = AttitudeTelemetryBridge(store, enabled=True, request_hz=50.0)
    bridge.on_message(
        FakeMessage(
            "ATTITUDE",
            time_boot_ms=5000,
            roll=0.05,
            pitch=-0.1,
            yaw=1.0,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
            source_system=1,
            source_component=1,
        )
    )
    got = store.get()
    if got is None or abs(got.roll - 0.05) > 1e-5:
        raise SystemExit("Layer2 FAIL: store missing sample")
    health = bridge.get_health()
    if health.status != "ok":
        raise SystemExit(f"Layer2 FAIL: health={health.status} {health.reason}")
    mav = FakeMav()
    bridge.request_interval(mav)
    if not mav.message_interval_calls:
        raise SystemExit("Layer2 FAIL: no message_interval_send")
    msg_id, interval_us = mav.message_interval_calls[0]
    if msg_id != MAVLINK_MSG_ID_ATTITUDE or interval_us != 20_000:
        raise SystemExit(f"Layer2 FAIL: interval call {(msg_id, interval_us)}")
    print(f"  Layer2 bridge OK: health={health.status} interval_us={interval_us}")


def _udp_injector(stop: threading.Event) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame = _v2_attitude_frame()
    target = ("127.0.0.1", SMOKE_PORT)
    while not stop.is_set():
        sock.sendto(frame, target)
        time.sleep(0.05)
    sock.close()


def _cli_smoke() -> None:
    stop = threading.Event()
    injector = threading.Thread(target=_udp_injector, args=(stop,), daemon=True)
    injector.start()
    time.sleep(0.1)

    listen_cmd = [
        sys.executable,
        "-m",
        "src.mavlink.attitude_listen",
        "--port",
        str(SMOKE_PORT),
        "--duration",
        "2",
    ]
    print(f"  Running: {' '.join(listen_cmd)}")
    listen = subprocess.run(listen_cmd, cwd=ROOT, capture_output=True, text=True)
    stop.set()
    injector.join(timeout=1.0)

    if listen.returncode != 0:
        print(listen.stdout)
        print(listen.stderr, file=sys.stderr)
        raise SystemExit(f"attitude-listen FAIL exit={listen.returncode}")
    if "PASS" not in listen.stdout:
        raise SystemExit("attitude-listen FAIL: no PASS in output")
    print("  attitude-listen OK")

    # Same frame through check-mavlink decode path (in-process)
    buf = _v2_attitude_frame()
    frame = parse_mavlink(buf)
    if frame is None or frame.msgid != MAVLINK_MSG_ID_ATTITUDE:
        raise SystemExit("check-mavlink path FAIL: parse")
    pl = frame_payload(buf, frame)
    sample = decode_attitude_payload(pl) if pl else None
    if sample is None:
        raise SystemExit("check-mavlink path FAIL: decode")
    print(
        f"  check-mavlink decode path OK: rpy_deg={roll_pitch_yaw_deg(sample)}"
    )


def main() -> int:
    probe = ROOT / "src" / "check_mavlink.py"
    if b"\x00" in probe.read_bytes():
        raise SystemExit(f"{probe} must be UTF-8 (found UTF-16 null bytes)")
    att_cfg = load_attitude_mavlink_config(load_config())
    print(f"[smoke-attitude] config control.mavlink.attitude: {att_cfg}")
    print("[smoke-attitude] Layer 1 + 2 (in-process)...")
    _layer1_smoke()
    _layer2_smoke()
    print("[smoke-attitude] CLI + UDP injection...")
    _cli_smoke()
    print("[smoke-attitude] ALL PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
