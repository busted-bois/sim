"""Configuration loader for JSON config files.

Resolves the config path from an explicit argument, ``AIGP_CONFIG``, or the
default ``sim.config.json`` in the project root, and can apply a profile
overlay via ``AIGP_PROFILE``.
"""
from __future__ import annotations

import json
import os
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
        "maze_colosseum_path",
        "maze_project_path",
        "windowed",
        "res_x",
        "res_y",
        "rpc_ready_timeout_seconds",
        "maze_rpc_ready_timeout_seconds",
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


def load_config(path: str | Path | None = None) -> dict:
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
    return data


def simulator_endpoint(config: dict[str, Any]) -> tuple[str, int]:
    """AirSim RPC host and port from merged config."""
    sim = config.get("simulator", {})
    host = str(sim.get("host", "127.0.0.1")).strip() or "127.0.0.1"
    port = int(sim.get("airsim_port", 41451))
    return host, port
