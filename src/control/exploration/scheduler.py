"""Leg-based XY coverage, altitude layers, and periodic 360° panorama scans."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any

from src.control.utils import _clamp

_TWO_PI = 2.0 * math.pi
_HEADING_TOL_DEG = 5.0
_ALT_TOL_M = 0.35


class ExplorationMode(str, Enum):
    WANDER = "WANDER"
    LEG_TURN = "LEG_TURN"
    ALT_TRANSITION = "ALT_TRANSITION"
    PANORAMA_360 = "PANORAMA_360"


@dataclass(frozen=True)
class ExplorationSettings:
    panorama_enabled: bool = True
    panorama_interval_s: float = 10.0
    panorama_yaw_rate_dps: float = 45.0
    panorama_creep_speed_ms: float = 0.0
    leg_enabled: bool = True
    leg_duration_s: float = 10.0
    leg_turn_deg: float = 90.0
    leg_turn_rate_dps: float = 35.0
    altitude_layers_m: tuple[float, ...] = (3.5, 5.0, 6.5)
    altitude_layer_dwell_s: float = 18.0
    vertical_depth_bias_gain: float = 0.4
    vertical_depth_bias_enabled: bool = True
    legacy_scan_enabled: bool = False


@dataclass(frozen=True)
class WanderTickInput:
    now_s: float
    dt_s: float
    yaw_rad: float
    z_ned: float
    fwd_speed: float
    yaw_rate_deg_s: float
    cos_yaw: float
    sin_yaw: float
    base_vz: float
    upper_clearance: float | None = None
    lower_clearance: float | None = None
    defer_panorama: bool = False
    yaw_rate_bias_deg_s: float = 0.0
    request_loop_closure_panorama: bool = False


@dataclass(frozen=True)
class WanderTickOutput:
    fwd_speed: float
    yaw_rate_deg_s: float
    vz: float
    mode: ExplorationMode
    label: str
    z_hold_ned: float


def yaw_delta_rad(prev_rad: float, curr_rad: float) -> float:
    return abs((curr_rad - prev_rad + math.pi) % (2 * math.pi) - math.pi)


def parse_exploration_settings(
    raw: dict[str, Any] | None,
    *,
    hold_altitude_m: float,
    max_altitude_m: float,
) -> ExplorationSettings:
    expl = raw or {}
    layers_raw = expl.get("altitude_layers_m")
    if isinstance(layers_raw, list) and layers_raw:
        layers = tuple(_clamp(float(a), 1.5, max_altitude_m) for a in layers_raw)
    else:
        layers = (_clamp(hold_altitude_m, 1.5, max_altitude_m),)

    panorama_enabled = bool(expl.get("panorama_enabled", True))
    legacy_raw = expl.get("legacy_scan_enabled")
    if legacy_raw is None:
        legacy_scan_enabled = not panorama_enabled
    else:
        legacy_scan_enabled = bool(legacy_raw)

    return ExplorationSettings(
        panorama_enabled=panorama_enabled,
        panorama_interval_s=_clamp(float(expl.get("panorama_interval_s", 10.0)), 3.0, 120.0),
        panorama_yaw_rate_dps=_clamp(float(expl.get("panorama_yaw_rate_dps", 45.0)), 10.0, 90.0),
        panorama_creep_speed_ms=_clamp(float(expl.get("panorama_creep_speed_ms", 0.0)), 0.0, 2.0),
        leg_enabled=bool(expl.get("leg_enabled", True)),
        leg_duration_s=_clamp(float(expl.get("leg_duration_s", 10.0)), 2.0, 60.0),
        leg_turn_deg=_clamp(float(expl.get("leg_turn_deg", 90.0)), 45.0, 180.0),
        leg_turn_rate_dps=_clamp(float(expl.get("leg_turn_rate_dps", 35.0)), 10.0, 90.0),
        altitude_layers_m=layers,
        altitude_layer_dwell_s=_clamp(float(expl.get("altitude_layer_dwell_s", 18.0)), 5.0, 120.0),
        vertical_depth_bias_gain=_clamp(float(expl.get("vertical_depth_bias_gain", 0.4)), 0.0, 1.5),
        vertical_depth_bias_enabled=bool(expl.get("vertical_depth_bias_enabled", True)),
        legacy_scan_enabled=legacy_scan_enabled,
    )


def build_wander_tick_input(
    *,
    now_s: float,
    dt_s: float,
    yaw_rad: float,
    z_ned: float,
    cos_yaw: float,
    sin_yaw: float,
    base_vz: float,
    fwd_speed: float = 0.0,
    yaw_rate_deg_s: float = 0.0,
    upper_clearance: float | None = None,
    lower_clearance: float | None = None,
    defer_panorama: bool = False,
    yaw_rate_bias_deg_s: float = 0.0,
    request_loop_closure_panorama: bool = False,
) -> WanderTickInput:
    return WanderTickInput(
        now_s=now_s,
        dt_s=dt_s,
        yaw_rad=yaw_rad,
        z_ned=z_ned,
        fwd_speed=fwd_speed,
        yaw_rate_deg_s=yaw_rate_deg_s,
        cos_yaw=cos_yaw,
        sin_yaw=sin_yaw,
        base_vz=base_vz,
        upper_clearance=upper_clearance,
        lower_clearance=lower_clearance,
        defer_panorama=defer_panorama,
        yaw_rate_bias_deg_s=yaw_rate_bias_deg_s,
        request_loop_closure_panorama=request_loop_closure_panorama,
    )


def yaw_error_deg(target_rad: float, current_rad: float) -> float:
    err = math.degrees(target_rad - current_rad)
    return (err + 180.0) % 360.0 - 180.0


def vertical_depth_bias_vz(
    upper_clearance: float | None,
    lower_clearance: float | None,
    *,
    gain: float,
    enabled: bool,
) -> float:
    if not enabled or upper_clearance is None or lower_clearance is None:
        return 0.0
    delta = lower_clearance - upper_clearance
    if abs(delta) < 1e-3:
        return 0.0
    return _clamp(gain * math.copysign(1.0, delta), -0.8, 0.8)


def _altitude_z_hold_ned(altitude_m: float) -> float:
    return -altitude_m


def vz_toward_altitude_hold(
    z_ned: float,
    z_hold_ned: float,
    *,
    tol_m: float = _ALT_TOL_M,
    rate_ms: float = 0.5,
) -> float:
    err = z_ned - z_hold_ned
    if err < -tol_m:
        return rate_ms
    if err > tol_m:
        return -rate_ms
    return 0.0


class ExplorationScheduler:
    def __init__(
        self,
        settings: ExplorationSettings,
        *,
        start_s: float,
        initial_yaw_rad: float,
        initial_z_ned: float,
    ) -> None:
        self._s = settings
        self._mode = ExplorationMode.WANDER
        self._last_panorama_s = start_s
        self._panorama_yaw_accum_rad = 0.0
        self._panorama_prev_yaw_rad: float | None = None
        self._leg_heading_rad = initial_yaw_rad
        self._leg_until_s = start_s + settings.leg_duration_s
        self._leg_turn_target_rad = initial_yaw_rad
        self._altitude_index = 0
        self._alt_layer_until_s = start_s + settings.altitude_layer_dwell_s
        layers = settings.altitude_layers_m
        self._z_hold_ned = _altitude_z_hold_ned(layers[0])
        if abs(initial_z_ned - self._z_hold_ned) > _ALT_TOL_M:
            self._mode = ExplorationMode.ALT_TRANSITION

    @property
    def z_hold_ned(self) -> float:
        return self._z_hold_ned

    def reset_panorama_timer(self, now_s: float) -> None:
        self._last_panorama_s = now_s
        self._panorama_yaw_accum_rad = 0.0
        self._panorama_prev_yaw_rad = None
        if self._mode == ExplorationMode.PANORAMA_360:
            self._mode = ExplorationMode.WANDER

    def tick(self, inp: WanderTickInput) -> WanderTickOutput:
        self._advance_altitude_layer(inp.now_s)

        if self._mode == ExplorationMode.PANORAMA_360:
            if inp.defer_panorama:
                self._abort_panorama()
                self._mode = ExplorationMode.WANDER
            else:
                return self._tick_panorama(inp)

        if (
            self._s.panorama_enabled
            and not inp.defer_panorama
            and self._mode == ExplorationMode.WANDER
            and inp.request_loop_closure_panorama
        ):
            self._mode = ExplorationMode.PANORAMA_360
            self._panorama_yaw_accum_rad = 0.0
            self._panorama_prev_yaw_rad = None
            return self._tick_panorama(inp)

        alt_err = abs(inp.z_ned - self._z_hold_ned)
        if alt_err > _ALT_TOL_M:
            self._mode = ExplorationMode.ALT_TRANSITION
            return self._tick_altitude(inp)

        if (
            self._s.panorama_enabled
            and not inp.defer_panorama
            and self._mode == ExplorationMode.WANDER
        ):
            if inp.now_s - self._last_panorama_s >= self._s.panorama_interval_s:
                self._mode = ExplorationMode.PANORAMA_360
                self._panorama_yaw_accum_rad = 0.0
                self._panorama_prev_yaw_rad = None
                return self._tick_panorama(inp)

        if self._s.leg_enabled and inp.now_s >= self._leg_until_s:
            self._mode = ExplorationMode.LEG_TURN
            turn_rad = math.radians(self._s.leg_turn_deg)
            self._leg_turn_target_rad = self._leg_heading_rad + turn_rad
            return self._tick_leg_turn(inp)

        self._mode = ExplorationMode.WANDER
        return self._tick_wander(inp)

    def _abort_panorama(self) -> None:
        self._panorama_yaw_accum_rad = 0.0
        self._panorama_prev_yaw_rad = None

    def _advance_altitude_layer(self, now_s: float) -> None:
        if now_s < self._alt_layer_until_s:
            return
        layers = self._s.altitude_layers_m
        if len(layers) <= 1:
            self._alt_layer_until_s = now_s + self._s.altitude_layer_dwell_s
            return
        self._altitude_index = (self._altitude_index + 1) % len(layers)
        self._z_hold_ned = _altitude_z_hold_ned(layers[self._altitude_index])
        self._alt_layer_until_s = now_s + self._s.altitude_layer_dwell_s
        self._mode = ExplorationMode.ALT_TRANSITION

    def _vz_toward_hold(self, z_ned: float) -> float:
        return vz_toward_altitude_hold(z_ned, self._z_hold_ned)

    def _tick_altitude(self, inp: WanderTickInput) -> WanderTickOutput:
        vz = self._vz_toward_hold(inp.z_ned)
        label = f"ALT_TRANSITION z_hold={-self._z_hold_ned:.1f}"
        if abs(inp.z_ned - self._z_hold_ned) <= _ALT_TOL_M:
            self._mode = ExplorationMode.WANDER
        return WanderTickOutput(
            fwd_speed=0.0,
            yaw_rate_deg_s=0.0,
            vz=vz,
            mode=ExplorationMode.ALT_TRANSITION,
            label=label,
            z_hold_ned=self._z_hold_ned,
        )

    def _tick_panorama(self, inp: WanderTickInput) -> WanderTickOutput:
        if self._panorama_prev_yaw_rad is None:
            self._panorama_prev_yaw_rad = inp.yaw_rad
        else:
            self._panorama_yaw_accum_rad += yaw_delta_rad(
                self._panorama_prev_yaw_rad, inp.yaw_rad
            )
        self._panorama_prev_yaw_rad = inp.yaw_rad

        mode = ExplorationMode.PANORAMA_360
        if self._panorama_yaw_accum_rad >= _TWO_PI:
            self._mode = ExplorationMode.WANDER
            self._last_panorama_s = inp.now_s
            self._abort_panorama()
            mode = ExplorationMode.WANDER
            label = "PANORAMA_360 done"
        else:
            remaining_deg = math.degrees(_TWO_PI - self._panorama_yaw_accum_rad)
            label = f"PANORAMA_360 ({remaining_deg:.0f} deg left)"
        return WanderTickOutput(
            fwd_speed=self._s.panorama_creep_speed_ms,
            yaw_rate_deg_s=self._s.panorama_yaw_rate_dps,
            vz=self._vz_toward_hold(inp.z_ned),
            mode=mode,
            label=label,
            z_hold_ned=self._z_hold_ned,
        )

    def _tick_leg_turn(self, inp: WanderTickInput) -> WanderTickOutput:
        err = yaw_error_deg(self._leg_turn_target_rad, inp.yaw_rad)
        if abs(err) <= _HEADING_TOL_DEG:
            self._leg_heading_rad = self._leg_turn_target_rad
            self._leg_until_s = inp.now_s + self._s.leg_duration_s
            self._mode = ExplorationMode.WANDER
            return self._tick_wander(inp)
        yaw_rate = _clamp(2.0 * err, -self._s.leg_turn_rate_dps, self._s.leg_turn_rate_dps)
        return WanderTickOutput(
            fwd_speed=0.0,
            yaw_rate_deg_s=yaw_rate,
            vz=self._vz_toward_hold(inp.z_ned),
            mode=ExplorationMode.LEG_TURN,
            label=f"LEG_TURN err={err:+.1f} deg",
            z_hold_ned=self._z_hold_ned,
        )

    def _tick_wander(self, inp: WanderTickInput) -> WanderTickOutput:
        fwd = inp.fwd_speed
        yaw_rate = inp.yaw_rate_deg_s
        if self._s.leg_enabled:
            blend = 0.35
            leg_cos = math.cos(self._leg_heading_rad)
            leg_sin = math.sin(self._leg_heading_rad)
            combined_cos = (1.0 - blend) * leg_cos + blend * inp.cos_yaw
            combined_sin = (1.0 - blend) * leg_sin + blend * inp.sin_yaw
            norm = math.hypot(combined_cos, combined_sin)
            if norm > 1e-6:
                combined_cos /= norm
                combined_sin /= norm
            err = yaw_error_deg(math.atan2(combined_sin, combined_cos), inp.yaw_rad)
            yaw_rate = _clamp(
                0.6 * inp.yaw_rate_deg_s + 0.4 * err + inp.yaw_rate_bias_deg_s,
                -90.0,
                90.0,
            )
        elif inp.yaw_rate_bias_deg_s != 0.0:
            yaw_rate = _clamp(inp.yaw_rate_deg_s + inp.yaw_rate_bias_deg_s, -90.0, 90.0)
        vz = inp.base_vz + vertical_depth_bias_vz(
            inp.upper_clearance,
            inp.lower_clearance,
            gain=self._s.vertical_depth_bias_gain,
            enabled=self._s.vertical_depth_bias_enabled,
        )
        if abs(inp.z_ned - self._z_hold_ned) > _ALT_TOL_M:
            vz = self._vz_toward_hold(inp.z_ned)
        label = "LEG_CRUISE" if self._s.leg_enabled else "WANDER"
        return WanderTickOutput(
            fwd_speed=fwd,
            yaw_rate_deg_s=yaw_rate,
            vz=_clamp(vz, -1.2, 1.2),
            mode=ExplorationMode.WANDER,
            label=label,
            z_hold_ned=self._z_hold_ned,
        )
