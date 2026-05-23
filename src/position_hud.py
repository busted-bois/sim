"""On-screen position/time overlay via AirSim simPrintLogMessage (RPC display-only)."""

from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Protocol

from src.position_trace import PositionRow, PositionTraceSnapshot
from src.tracking.snapshot import TrackingSnapshot
from src.tracking.state import TrackingHealth, TrackingState

HudDataSource = Literal["trace", "tracking"]

_DEBUG_LOG_PATH = Path(__file__).resolve().parent.parent / "debug-f6d05d.log"


def _debug_log(hypothesis_id: str, location: str, message: str, data: dict[str, Any]) -> None:
    # #region agent log
    try:
        payload = {
            "sessionId": "f6d05d",
            "hypothesisId": hypothesis_id,
            "location": location,
            "message": message,
            "data": data,
            "timestamp": int(time.time() * 1000),
        }
        with _DEBUG_LOG_PATH.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, default=str) + "\n")
    except OSError:
        pass
    # #endregion


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


def resolve_hud_data_source(
    config: dict[str, Any],
    *,
    explicit: str | None = None,
    transport: str | None = None,
) -> HudDataSource:
    effective_transport = (
        transport or str(config.get("control", {}).get("transport", "airsim"))
    ).strip().lower()
    if explicit in ("trace", "tracking"):
        chosen: HudDataSource = explicit  # type: ignore[assignment]
    else:
        mav_cfg = config.get("control", {}).get("mavlink", {})
        tracking_enabled = bool(mav_cfg.get("tracking", {}).get("enabled", False))
        chosen = "tracking" if tracking_enabled and effective_transport == "mavlink" else "trace"
    if chosen == "tracking" and effective_transport != "mavlink":
        return "trace"
    return chosen


def position_hud_config_from_dict(
    hud_cfg: dict[str, Any],
    *,
    config: dict[str, Any] | None = None,
    transport: str | None = None,
) -> PositionHudConfig:
    explicit = str(hud_cfg.get("data_source", "")).strip().lower() or None
    if config is not None:
        data_source = resolve_hud_data_source(
            config, explicit=explicit, transport=transport
        )
    else:
        data_source = _parse_data_source(hud_cfg.get("data_source"))
    message_label = str(hud_cfg.get("message_label", "AIGP pos: ")).strip() or "AIGP pos: "
    if not message_label.endswith((":", " ")):
        message_label = f"{message_label}: "
    return PositionHudConfig(
        enabled=bool(hud_cfg.get("enabled", False)),
        update_hz=max(1.0, float(hud_cfg.get("update_hz", 15.0))),
        message_label=message_label,
        severity=max(0, min(3, int(hud_cfg.get("severity", 0)))),
        data_source=data_source,
    )


def format_position_hud_param(row: PositionRow) -> str:
    t_s, _boot_ms, x_m, y_m, z_m, vx, vy, vz, altitude_m = row
    return (
        f"t={t_s:.2f}s x={x_m:.2f} y={y_m:.2f} z={z_m:.2f} alt={altitude_m:.2f}m "
        f"vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}"
    )


class RpcTrackingHudProvider:
    """Build a TrackingSnapshot from AirSim RPC for the enriched on-screen HUD."""

    def __init__(self, client: Any) -> None:
        self._client = client
        self._t0: float | None = None

    def __call__(self) -> TrackingSnapshot | None:
        from src.control.utils import rpy_from_orientation

        try:
            state = self._client.getMultirotorState()
            k = state.kinematics_estimated
            p = k.position
            v = k.linear_velocity
            x_m = float(p.x_val)
            y_m = float(p.y_val)
            z_m = float(p.z_val)
            vx = float(v.x_val)
            vy = float(v.y_val)
            vz = float(v.z_val)
            roll, pitch, yaw = rpy_from_orientation(k.orientation)
            now = time.monotonic()
            if self._t0 is None:
                self._t0 = now
            t_s = now - (self._t0 or now)
            imu_health = None
            getter = getattr(self._client, "getHighresImuHealth", None)
            if callable(getter):
                imu_health = getter()
            imu_rate = None
            if imu_health is not None:
                imu_rate = imu_health.stream_rate_hz
            health = TrackingHealth(
                status=imu_health.status if imu_health is not None else "rpc",
                reason=imu_health.reason if imu_health is not None else "AirSim RPC HUD",
                imu_sample_count=imu_health.sample_count if imu_health is not None else 0,
                imu_rate_hz=imu_rate,
                vision_correction_count=0,
                origin_set=True,
                armed=True,
            )
            tracking_state = TrackingState(
                sim_time_ns=int(getattr(state, "timestamp", 0) or 0),
                t_s=t_s,
                position_ned=(x_m, y_m, z_m),
                velocity_ned=(vx, vy, vz),
                attitude_rpy=(roll, pitch, yaw),
                armed=True,
                origin_set=True,
            )
            return TrackingSnapshot(
                sim_time_ns=tracking_state.sim_time_ns,
                image_rgb=None,
                position_ned=tracking_state.position_ned,
                velocity_ned=tracking_state.velocity_ned,
                attitude_rpy=tracking_state.attitude_rpy,
                health=health,
                state=tracking_state,
            )
        except Exception as exc:
            _debug_log(
                "D",
                "position_hud.py:RpcTrackingHudProvider",
                "rpc snapshot failed",
                {"error": str(exc)},
            )
            return None


def format_tracking_hud_param(snapshot: TrackingSnapshot) -> str:
    state = snapshot.state
    if state is not None:
        t_s = state.t_s
        x, y, z = state.position_ned
        vx, vy, vz = state.velocity_ned
        roll, pitch, yaw = state.attitude_rpy
        alt = state.altitude_m
    else:
        t_s = snapshot.sim_time_ns / 1e9 if snapshot.sim_time_ns else 0.0
        x, y, z = snapshot.position_ned
        vx, vy, vz = snapshot.velocity_ned
        roll, pitch, yaw = snapshot.attitude_rpy
        alt = max(0.0, -z)
    health = snapshot.health
    imu_part = f"imu={health.imu_rate_hz:.0f}Hz" if health.imu_rate_hz is not None else "imu=--"
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    return (
        f"t={t_s:.2f}s x={x:.2f} y={y:.2f} alt={alt:.2f}m; "
        f"roll={roll_deg:.1f}deg pitch={pitch_deg:.1f}deg yaw={yaw_deg:.1f}deg; "
        f"vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}; {imu_part} "
        f"status={health.status} vis={health.vision_correction_count}"
    )


def _hud_param_from_snapshot(
    snapshot: PositionTraceSnapshot | TrackingSnapshot,
    data_source: HudDataSource,
) -> str | None:
    if isinstance(snapshot, TrackingSnapshot):
        if snapshot.health.origin_set or snapshot.health.status == "rpc":
            return format_tracking_hud_param(snapshot)
        if data_source != "tracking":
            return None
    if data_source == "tracking":
        if not isinstance(snapshot, TrackingSnapshot):
            _debug_log(
                "B",
                "position_hud.py:_hud_param_from_snapshot",
                "wrong snapshot type for tracking",
                {"type": type(snapshot).__name__},
            )
            return None
        if not snapshot.health.origin_set:
            _debug_log(
                "A",
                "position_hud.py:_hud_param_from_snapshot",
                "tracking origin not set",
                {"status": snapshot.health.status, "origin_set": snapshot.health.origin_set},
            )
            return None
        return format_tracking_hud_param(snapshot)
    if not isinstance(snapshot, PositionTraceSnapshot) or snapshot.latest is None:
        _debug_log(
            "E",
            "position_hud.py:_hud_param_from_snapshot",
            "trace snapshot missing latest",
            {
                "type": type(snapshot).__name__,
                "latest": snapshot.latest if isinstance(snapshot, PositionTraceSnapshot) else None,
            },
        )
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
        tick = 0
        while not self._stop.wait(period_s):
            tick += 1
            snapshot = self._snapshot_provider()
            if snapshot is None:
                if tick <= 3:
                    _debug_log(
                        "D",
                        "position_hud.py:_loop",
                        "snapshot provider returned None",
                        {"tick": tick, "data_source": self._config.data_source},
                    )
                continue
            param = _hud_param_from_snapshot(snapshot, self._config.data_source)
            if param is None:
                if tick <= 3:
                    _debug_log(
                        "A",
                        "position_hud.py:_loop",
                        "hud param None",
                        {
                            "tick": tick,
                            "data_source": self._config.data_source,
                            "snapshot_type": type(snapshot).__name__,
                        },
                    )
                continue
            rpc = self._connect_rpc()
            if rpc is None:
                if tick <= 3:
                    _debug_log(
                        "C",
                        "position_hud.py:_loop",
                        "rpc connect failed",
                        {"tick": tick, "rpc_errors": self._rpc_errors},
                    )
                continue
            try:
                rpc.simPrintLogMessage(
                    self._config.message_label,
                    param,
                    self._config.severity,
                )
                self._update_count += 1
                if self._update_count <= 2:
                    _debug_log(
                        "OK",
                        "position_hud.py:_loop",
                        "hud posted",
                        {
                            "update_count": self._update_count,
                            "param_len": len(param),
                            "param_prefix": param[:80],
                        },
                    )
            except Exception as exc:
                self._rpc_errors += 1
                self._rpc = None
                if self._rpc_errors <= 3:
                    _debug_log(
                        "C",
                        "position_hud.py:_loop",
                        "simPrintLogMessage failed",
                        {"error": str(exc), "rpc_errors": self._rpc_errors},
                    )
