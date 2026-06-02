"""Print NED position to the terminal (one shot by default; AirSim RPC or MAVLink)."""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import airsim
from src.config import apply_low_end_overrides, load_config, simulator_endpoint
from src.control.mavlink_client import PymavlinkFlightClient
from src.mavlink_endpoints import first_mavlink_heartbeat_endpoint, resolve_control_transport
from src.position_hud import format_position_hud_param
from src.position_trace import (
    PositionTraceSnapshot,
    RpcPositionSnapshotProvider,
    position_trace_store_from_config,
)

ROOT = Path(__file__).resolve().parent.parent
_LOG = "position-print"
_DEFAULT_WAIT_S = 3.0
_DEFAULT_WAIT_STEP_S = 0.1


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Print current vehicle NED position once and exit. "
            "Start the simulator first (`uv run sim`). "
            "Use --stream or --duration for repeated output."
        ),
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Print repeatedly until Ctrl+C.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Stream for N seconds (implies repeated printing).",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=None,
        help="Print rate when streaming (default: position_hud.update_hz or 5).",
    )
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=_DEFAULT_WAIT_S,
        help=f"Max seconds to wait for first position (default {_DEFAULT_WAIT_S}).",
    )
    parser.add_argument(
        "--endpoint",
        help="MAVLink endpoint override, e.g. udpin:0.0.0.0:14550",
    )
    parser.add_argument(
        "--transport",
        choices=("airsim", "mavlink", "auto"),
        help="Control link (default: auto — probe MAVLink then AirSim RPC).",
    )
    return parser


def _resolve_rate_hz(config: dict, args: argparse.Namespace) -> float:
    if args.rate_hz is not None:
        return max(0.5, float(args.rate_hz))
    hud_hz = config.get("control", {}).get("mavlink", {}).get("position_hud", {}).get(
        "update_hz", 5.0
    )
    return max(0.5, float(hud_hz))


def _streaming_mode(args: argparse.Namespace) -> bool:
    if args.stream:
        return True
    return args.duration is not None and float(args.duration) > 0.0


def _airsim_port(config: dict) -> tuple[str, int]:
    host, port = simulator_endpoint(config)
    env_port = os.environ.get("AIRSIM_PORT", "").strip()
    if env_port:
        port = int(env_port)
    return host, port


def _airsim_rpc_ready(config: dict, *, timeout_s: float = 2.0) -> bool:
    host, port = _airsim_port(config)
    deadline = time.monotonic() + max(0.5, float(timeout_s))
    while time.monotonic() < deadline:
        try:
            client = airsim.MultirotorClient(ip=host, port=port, timeout_value=2)
            client.confirmConnection()
            return True
        except Exception:
            time.sleep(0.25)
    return False


def _mavlink_snapshot_provider(
    config: dict,
    endpoint_override: str | None,
    *,
    endpoint: str,
) -> tuple[PymavlinkFlightClient, Callable[[], PositionTraceSnapshot], str]:
    position_trace = position_trace_store_from_config(config, ROOT)
    if position_trace is None:
        raise SystemExit(
            f"[{_LOG}] MAVLink position trace is disabled. Enable "
            "control.mavlink.position_trace.enabled or position_hud.enabled."
        )
    mav_cfg = config.get("control", {}).get("mavlink", {})
    control_cfg = config.get("control", {})
    client = PymavlinkFlightClient(
        endpoint=endpoint,
        command_rate_hz=float(control_cfg.get("command_rate_hz", 50.0)),
        state_request_hz=float(mav_cfg.get("state_request_hz", 20.0)),
        guided_custom_mode=int(mav_cfg.get("guided_custom_mode", 4)),
        takeoff_altitude_m=float(mav_cfg.get("takeoff_altitude_m", 5.0)),
        land_descent_speed_ms=float(config.get("landing", {}).get("descent_speed_ms", 2.0)),
        source_system=int(mav_cfg.get("source_system", 255)),
        source_component=int(mav_cfg.get("source_component", 1)),
        sim_config=config,
        position_trace=position_trace,
        prepare_for_flight_on_connect=False,
        heartbeat_timeout_s=2.0,
    )
    client.confirmConnection()
    return client, client.getPositionTraceSnapshot, endpoint


def _airsim_snapshot_provider(
    config: dict,
) -> tuple[airsim.MultirotorClient, Callable[[], PositionTraceSnapshot], str]:
    host, port = _airsim_port(config)
    client = airsim.MultirotorClient(ip=host, port=port, timeout_value=5)
    client.confirmConnection()
    return client, RpcPositionSnapshotProvider(client), f"{host}:{port}"


@dataclass(frozen=True, slots=True)
class _PrintConnection:
    transport: str
    client: Any
    provider: Callable[[], PositionTraceSnapshot]
    source: str


def _resolve_connection(config: dict, args: argparse.Namespace) -> _PrintConnection:
    env_transport = os.environ.get("AIGP_CONTROL_TRANSPORT", "").strip().lower()
    if args.transport:
        requested = str(args.transport).strip().lower()
    elif env_transport in ("airsim", "mavlink"):
        requested = env_transport
    else:
        requested = "auto"
    if requested == "auto":
        if resolve_control_transport(config) == "airsim":
            requested = "airsim"

    host, port = _airsim_port(config)

    if requested == "airsim":
        client, provider, source = _airsim_snapshot_provider(config)
        return _PrintConnection("airsim", client, provider, source)

    if requested == "mavlink":
        endpoint = (
            (args.endpoint or "").strip()
            or os.environ.get("AIGP_MAVLINK_ENDPOINT", "").strip()
            or str(config.get("control", {}).get("mavlink", {}).get("endpoint", "")).strip()
            or "udpin:0.0.0.0:14550"
        )
        client, provider, source = _mavlink_snapshot_provider(
            config, args.endpoint, endpoint=endpoint
        )
        return _PrintConnection("mavlink", client, provider, source)

    mavlink_endpoint = (
        (args.endpoint or "").strip()
        or os.environ.get("AIGP_MAVLINK_ENDPOINT", "").strip()
        or first_mavlink_heartbeat_endpoint(config, timeout_s=2.0)
    )
    if mavlink_endpoint:
        try:
            client, provider, source = _mavlink_snapshot_provider(
                config, args.endpoint, endpoint=mavlink_endpoint
            )
            return _PrintConnection("mavlink", client, provider, source)
        except TimeoutError:
            pass

    if _airsim_rpc_ready(config, timeout_s=2.0):
        client, provider, source = _airsim_snapshot_provider(config)
        print(
            f"[{_LOG}] No MAVLink HEARTBEAT; using AirSim RPC at {source} "
            "(SimpleFlight / start PX4-SITL for MAVLink).",
            file=sys.stderr,
            flush=True,
        )
        return _PrintConnection("airsim", client, provider, source)

    raise SystemExit(
        f"[{_LOG}] No control link available. Start the simulator (`uv run sim`) or "
        f"PX4-SITL for MAVLink. Probed MAVLink and AirSim RPC on {host}:{port}."
    )


def _snapshot_from_provider(
    provider: Callable[[], PositionTraceSnapshot],
) -> PositionTraceSnapshot | None:
    snapshot = provider()
    if snapshot is None or snapshot.latest is None:
        return None
    return snapshot


def _print_position(transport: str, snapshot: PositionTraceSnapshot) -> None:
    assert snapshot.latest is not None
    print(format_position_hud_param(snapshot.latest), flush=True)


def _wait_for_snapshot(
    provider: Callable[[], PositionTraceSnapshot],
    *,
    wait_s: float,
    step_s: float,
) -> PositionTraceSnapshot | None:
    deadline = time.monotonic() + max(0.0, float(wait_s))
    while time.monotonic() < deadline:
        snapshot = _snapshot_from_provider(provider)
        if snapshot is not None:
            return snapshot
        time.sleep(max(0.01, float(step_s)))
    return _snapshot_from_provider(provider)


def _print_once(
    transport: str,
    provider: Callable[[], PositionTraceSnapshot],
    *,
    wait_s: float,
) -> bool:
    snapshot = _wait_for_snapshot(
        provider,
        wait_s=wait_s,
        step_s=_DEFAULT_WAIT_STEP_S,
    )
    if snapshot is None:
        return False
    _print_position(transport, snapshot)
    return True


def _print_stream(
    transport: str,
    provider: Callable[[], PositionTraceSnapshot],
    *,
    rate_hz: float,
    duration_s: float | None,
) -> int:
    period_s = 1.0 / rate_hz
    deadline = None if duration_s is None else time.monotonic() + duration_s
    sample_count = 0
    while deadline is None or time.monotonic() < deadline:
        loop_start = time.monotonic()
        snapshot = _snapshot_from_provider(provider)
        if snapshot is not None:
            sample_count += 1
            _print_position(transport, snapshot)
        if deadline is None:
            time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))
        else:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(period_s, remaining))
    return sample_count


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    config = load_config()
    apply_low_end_overrides(config)
    raw = getattr(config, "_raw", config)
    streaming = _streaming_mode(args)

    connection: _PrintConnection | None = None
    try:
        connection = _resolve_connection(raw, args)
        transport = connection.transport
        provider = connection.provider

        if not streaming:
            if not _print_once(transport, provider, wait_s=float(args.wait_seconds)):
                print(f"[{_LOG}] no position available.", file=sys.stderr)
                return 1
            return 0

        rate_hz = _resolve_rate_hz(raw, args)
        duration_s = None if args.stream else float(args.duration)
        sample_count = _print_stream(
            transport,
            provider,
            rate_hz=rate_hz,
            duration_s=duration_s,
        )
        if sample_count == 0:
            print(f"[{_LOG}] no position samples received.", file=sys.stderr)
            return 1
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if connection is not None and isinstance(connection.client, PymavlinkFlightClient):
            connection.client.close()


if __name__ == "__main__":
    sys.exit(main())
