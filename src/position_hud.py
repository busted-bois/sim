"""On-screen position/time overlay via AirSim simPrintLogMessage (RPC display-only)."""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal, Protocol

from src.position_trace import PositionRow, PositionTraceSnapshot
from src.tracking.snapshot import TrackingSnapshot

HudDataSource = Literal["trace", "tracking"]


class _AirSimDisplayClient(Protocol):
    def confirmConnection(self) -> None: ...

    def simPrintLogMessage(
        self,
        message: str,
        message_param: str = "",
        severity: int = 0,
    ) -> None: ...


@dataclass(frozen=True, slots=True)
class PositionHudConfig:
    enabled: bool
    update_hz: float
    message_label: str
    severity: int
    data_source: HudDataSource = "trace"


def _parse_data_source(raw: Any) -> HudDataSource:
    value = str(raw or "trace").strip().lower()
    if value in ("trace", "tracking"):
        return value  # type: ignore[return-value]
    return "trace"


def position_hud_config_from_dict(hud_cfg: dict[str, Any]) -> PositionHudConfig:
    return PositionHudConfig(
        enabled=bool(hud_cfg.get("enabled", False)),
        update_hz=max(1.0, float(hud_cfg.get("update_hz", 15.0))),
        message_label=str(hud_cfg.get("message_label", "AIGP pos")).strip() or "AIGP pos",
        severity=max(0, min(3, int(hud_cfg.get("severity", 0)))),
        data_source=_parse_data_source(hud_cfg.get("data_source")),
    )


def format_position_hud_param(row: PositionRow) -> str:
    t_s, _boot_ms, x_m, y_m, z_m, _vx, _vy, _vz, altitude_m = row
    return (
        f"t={t_s:.2f}s x={x_m:.2f} y={y_m:.2f} z={z_m:.2f} "
        f"alt={altitude_m:.2f}m"
    )


def format_tracking_hud_param(snapshot: TrackingSnapshot) -> str:
    state = snapshot.state
    if state is not None:
        t_s = state.t_s
        x, y, z = state.position_ned
        vx, vy, vz = state.velocity_ned
        _roll, _pitch, yaw = state.attitude_rpy
        alt = state.altitude_m
    else:
        t_s = snapshot.sim_time_ns / 1e9 if snapshot.sim_time_ns else 0.0
        x, y, z = snapshot.position_ned
        vx, vy, vz = snapshot.velocity_ned
        _roll, _pitch, yaw = snapshot.attitude_rpy
        alt = max(0.0, -z)
    health = snapshot.health
    imu_part = f"imu={health.imu_rate_hz:.0f}Hz" if health.imu_rate_hz is not None else "imu=--"
    return (
        f"t={t_s:.2f}s x={x:.2f} y={y:.2f} z={z:.2f} alt={alt:.2f}m "
        f"vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} yaw={yaw:.2f} {imu_part} "
        f"vis={health.vision_correction_count}"
    )


def _hud_param_from_snapshot(
    snapshot: PositionTraceSnapshot | TrackingSnapshot,
    data_source: HudDataSource,
) -> str | None:
    if data_source == "tracking":
        if not isinstance(snapshot, TrackingSnapshot):
            return None
        if not snapshot.health.origin_set:
            return None
        return format_tracking_hud_param(snapshot)
    if not isinstance(snapshot, PositionTraceSnapshot) or snapshot.latest is None:
        return None
    return format_position_hud_param(snapshot.latest)


class PositionOnScreenHud:
    """Background updater that writes the latest trace/tracking sample into the UE viewport."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        snapshot_provider: Callable[
            [], PositionTraceSnapshot | TrackingSnapshot | None
        ],
        config: PositionHudConfig,
        client_factory: Callable[[str, int], _AirSimDisplayClient] | None = None,
    ) -> None:
        self._host = host
        self._port = int(port)
        self._snapshot_provider = snapshot_provider
        self._config = config
        self._client_factory = client_factory
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._rpc: _AirSimDisplayClient | None = None
        self._update_count = 0
        self._rpc_errors = 0

    @property
    def update_count(self) -> int:
        return self._update_count

    @property
    def rpc_error_count(self) -> int:
        return self._rpc_errors

    def start(self) -> None:
        if not self._config.enabled or self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="position_hud", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._rpc = None

    def _connect_rpc(self) -> _AirSimDisplayClient | None:
        if self._rpc is not None:
            return self._rpc
        try:
            if self._client_factory is not None:
                client = self._client_factory(self._host, self._port)
            else:
                import airsim

                client = airsim.MultirotorClient(
                    ip=self._host,
                    port=self._port,
                    timeout_value=5,
                )
            client.confirmConnection()
            self._rpc = client
            return client
        except Exception:
            self._rpc_errors += 1
            self._rpc = None
            return None

    def _loop(self) -> None:
        period_s = 1.0 / self._config.update_hz
        while not self._stop.wait(period_s):
            snapshot = self._snapshot_provider()
            if snapshot is None:
                continue
            param = _hud_param_from_snapshot(snapshot, self._config.data_source)
            if param is None:
                continue
            rpc = self._connect_rpc()
            if rpc is None:
                continue
            try:
                rpc.simPrintLogMessage(
                    self._config.message_label,
                    param,
                    self._config.severity,
                )
                self._update_count += 1
            except Exception:
                self._rpc_errors += 1
                self._rpc = None
