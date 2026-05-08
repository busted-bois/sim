"""Configuration loader for JSON config files.

Resolves the config path from an explicit argument, ``AIGP_CONFIG``, or the
default ``sim.config.json`` in the project root, and can apply a profile
overlay via ``AIGP_PROFILE``.
"""
from __future__ import annotations

import json
import os
from collections.abc import MutableMapping
from pathlib import Path
from typing import Any

DEFAULT_CONFIG_PATH = Path(__file__).parent.parent / "sim.config.json"
CONFIG_PATH_ENV = "AIGP_CONFIG"
PROFILE_ENV = "AIGP_PROFILE"


def resolve_config_path(explicit: str | Path | None = None) -> Path:
    """Resolve which JSON file to load (explicit arg > AIGP_CONFIG > default)."""
    if explicit:
        return Path(explicit)
    env_path = os.environ.get(CONFIG_PATH_ENV, "").strip()
    if env_path:
        return Path(env_path)
    return DEFAULT_CONFIG_PATH


def _deep_merge_into(base: dict[str, Any], update: dict[str, Any]) -> dict[str, Any]:
    """Recursively merge update into base (mutates base). Skips registry-only keys."""
    for key, value in update.items():
        if key in {"profiles"}:
            continue
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_into(base[key], value)
        else:
            base[key] = value
    return base


def _normalize_test_profile_overlay(tp: dict[str, Any]) -> dict[str, Any]:
    """Lift flat test_profile fields into simulator / attitude_four_motion sections."""
    patch = {k: v for k, v in tp.items() if k != "name"}
    sim_keys = {
        "map_name",
        "startup_delay_seconds",
        "airsim_port",
        "host",
        "project_path",
        "colosseum_path",
        "windowed",
        "res_x",
        "res_y",
        "rpc_ready_timeout_seconds",
    }
    sim_patch: dict[str, Any] = {}
    for key in list(patch.keys()):
        if key in sim_keys:
            sim_patch[key] = patch.pop(key)
    if sim_patch:
        existing = patch.get("simulator")
        merged_sim = dict(existing) if isinstance(existing, dict) else {}
        merged_sim.update(sim_patch)
        patch["simulator"] = merged_sim

    afm_patch: dict[str, Any] = {}
    for key in ("cruise_speed_ms", "segment_duration_s", "calibration_move_s", "stabilize_s"):
        if key in patch:
            afm_patch[key] = patch.pop(key)
    if afm_patch:
        existing = patch.get("attitude_four_motion")
        merged_afm = dict(existing) if isinstance(existing, dict) else {}
        merged_afm.update(afm_patch)
        patch["attitude_four_motion"] = merged_afm

    return patch


def _apply_profile_overlay(data: dict[str, Any]) -> None:
    """If AIGP_PROFILE is set, deep-merge matching profile or legacy test_profile."""
    name = os.environ.get(PROFILE_ENV, "").strip()
    if not name:
        return

    profiles = data.get("profiles")
    overlay: dict[str, Any] | None = None
    if isinstance(profiles, dict):
        raw = profiles.get(name)
        if isinstance(raw, dict):
            overlay = dict(raw)

    if overlay is None:
        tp = data.get("test_profile")
        if isinstance(tp, dict) and name == str(tp.get("name", "")).strip():
            overlay = _normalize_test_profile_overlay(tp)

    if not overlay:
        print(
            f"Warning: {PROFILE_ENV}={name!r} did not match any entry in "
            "'profiles' or 'test_profile.name'; config unchanged."
        )
        return

    _deep_merge_into(data, overlay)


class Config(MutableMapping):
    """Typed wrapper around config dict with full MutableMapping support."""

    def __init__(self, raw: dict[str, Any]) -> None:
        self._raw = raw

    def __getitem__(self, key: str) -> Any:
        return self._raw[key]

    def __setitem__(self, key: str, value: Any) -> None:
        self._raw[key] = value

    def __delitem__(self, key: str) -> None:
        del self._raw[key]

    def __iter__(self):
        return iter(self._raw)

    def __len__(self) -> int:
        return len(self._raw)

    def __contains__(self, key: object) -> bool:
        return key in self._raw

    @property
    def simulator(self) -> dict[str, Any]:
        return self._raw.get("simulator", {})

    @property
    def vision(self) -> dict[str, Any]:
        return self._raw.get("vision", {})

    @property
    def control(self) -> dict[str, Any]:
        return self._raw.get("control", {})

    @property
    def landing(self) -> dict[str, Any]:
        return self._raw.get("landing", {})

    @property
    def safety(self) -> dict[str, Any]:
        return self._raw.get("safety", {})

    @property
    def host(self) -> str:
        sim = self._raw.get("simulator", {})
        return str(sim.get("host", "127.0.0.1")).strip() or "127.0.0.1"

    @property
    def port(self) -> int:
        sim = self._raw.get("simulator", {})
        return int(sim.get("airsim_port", 41451))

    @property
    def algorithm_name(self) -> str:
        return str(self._raw.get("algorithm", "autonomous_explore"))


def load_config(path: str | Path | None = None) -> Config:
    """Load config from JSON, apply optional profile overlay (AIGP_PROFILE)."""
    config_path = resolve_config_path(path)
    if not config_path.is_file():
        msg = f"Config file not found: {config_path}"
        if config_path != DEFAULT_CONFIG_PATH:
            raise FileNotFoundError(msg)
        raise FileNotFoundError(f"{msg} (set {CONFIG_PATH_ENV} or restore sim.config.json)")

    with open(config_path, encoding="utf-8") as f:
        data: dict[str, Any] = json.load(f)

    _apply_profile_overlay(data)
    return Config(data)


def simulator_endpoint(config: Config | dict[str, Any]) -> tuple[str, int]:
    """AirSim RPC host and port from merged config."""
    sim = config.get("simulator", {})
    host = str(sim.get("host", "127.0.0.1")).strip() or "127.0.0.1"
    port = int(sim.get("airsim_port", 41451))
    return host, port


def apply_low_end_overrides(config: Config | dict[str, Any]) -> None:
    raw = config._raw if isinstance(config, Config) else config
    if os.environ.get("AIGP_LOW_END", "").strip() != "1":
        return
    print("Low-end mode enabled: prioritizing smooth flight over detailed logging.")
    low_end_cfg = raw.setdefault("low_end_profile", {})

    vision_cfg = raw.setdefault("vision", {})
    vision_cfg["enabled"] = bool(low_end_cfg.get("vision_enabled", False))
    if vision_cfg["enabled"]:
        vision_cfg["fps"] = float(low_end_cfg.get("vision_fps", 8.0))

    control_cfg = raw.setdefault("control", {})
    command_rate_hz = float(low_end_cfg.get("command_rate_hz", 25.0))
    control_cfg["command_rate_hz"] = max(10.0, min(35.0, command_rate_hz))
    latency_cfg = control_cfg.setdefault("latency_tuning", {})
    latency_cfg["enabled"] = False
    latency_cfg.setdefault("autotuner", {})["enabled"] = False

    landing_cfg = raw.setdefault("landing", {})
    landing_cfg.setdefault("telemetry_log", {})["enabled"] = False
    landing_cfg["min_hover_seconds"] = min(0.6, float(landing_cfg.get("min_hover_seconds", 1.0)))

    log_cfg = raw.setdefault("logging", {})
    log_cfg["basic_flight_logs"] = True
    raw["algorithm"] = str(low_end_cfg.get("algorithm", "attitude_four_motion"))
    raw.setdefault("safety", {})["algorithm_timeout_seconds"] = float(
        low_end_cfg.get("algorithm_timeout_seconds", 90.0)
    )

    six_cfg = raw.setdefault("six_directions", {})
    six_cfg["duration_s"] = float(low_end_cfg.get("segment_duration_s", 1.2))
    six_cfg["speed_ms"] = float(low_end_cfg.get("speed_ms", 1.6))
    six_cfg["direction_labels"] = list(
        low_end_cfg.get("direction_labels", ["+X", "-X", "+Y", "-Y"])
    )
