"""AirSim settings.json writers for launcher (SimpleFlight RPC vs PX4 TCP HIL).

PX4 SITL must use TCP HIL on :4560 (AirSim PX4Multirotor). Call ``ensure_launch_settings``
from ``sim_launch`` so mavlink vs airsim paths cannot drift.
"""

from __future__ import annotations

import copy
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# AirSim PX4 HIL listener (PX4-SITL connects here). Do not change without updating docs/tests.
PX4_HIL_TCP_PORT = 4560
PX4_VEHICLE_NAME = "PX4"
SIMPLEFLIGHT_VEHICLE_NAME = "Drone1"
SETTINGS_VERSION = 1.2
SIMPLEFLIGHT_BACKUP_NAME = "settings.simpleflight.bak.json"

PX4_VEHICLE_TEMPLATE: dict[str, Any] = {
    "VehicleType": "PX4Multirotor",
    "UseSerial": False,
    "UseTcp": True,
    "TcpPort": PX4_HIL_TCP_PORT,
    "LockStep": True,
    "ControlIp": "remote",
    "ControlPortLocal": 14540,
    "ControlPortRemote": 14580,
    "LocalHostIp": "0.0.0.0",
    "QgcHostIp": "127.0.0.1",
    "QgcPort": 14550,
    "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0,
    },
}


@dataclass(frozen=True)
class ViewportOptions:
    """View / trace options applied to both SimpleFlight and PX4 settings files."""

    view_mode: str = "Fpv"
    corner_chase_pip: bool = False
    enable_trace: bool = False


def settings_path() -> Path:
    return Path.home() / "Documents" / "AirSim" / "settings.json"


def normalize_view_mode(view_mode: str) -> str:
    mode = view_mode.strip().lower()
    if mode in {"3rd-person", "third-person", "flywithme"}:
        return "FlyWithMe"
    return "Fpv"


def airsim_mavlink_profile(config: dict[str, Any]) -> dict[str, Any]:
    raw = config.get("control", {}).get("mavlink", {}).get("airsim_profile", {})
    return dict(raw) if isinstance(raw, dict) else {}


def camera_block_from_config(config: dict[str, Any]) -> dict[str, Any]:
    """Front-camera block (conformant resolution / FOV from sim.config.json)."""
    from src.vision.intrinsics import horizontal_fov_degrees

    vision_cfg = config.get("vision", {})
    camera_cfg = config.get("camera", {})
    pose_offset = camera_cfg.get("pose_offset", [0.35, 0.0, -0.05])
    camera_pitch = -float(camera_cfg.get("pitch_up_degrees", 20.0))
    camera_roll = float(camera_cfg.get("roll_degrees", 0.0))
    camera_yaw = float(camera_cfg.get("yaw_degrees", 0.0))
    camera_name = str(vision_cfg.get("camera_name", "0"))
    resolution = vision_cfg.get("resolution", [640, 360])
    if isinstance(resolution, (list, tuple)) and len(resolution) == 2:
        capture_width = int(resolution[0])
        capture_height = int(resolution[1])
    else:
        capture_width = int(vision_cfg.get("width", 640))
        capture_height = int(vision_cfg.get("height", 360))
    front_camera_settings = {
        "X": float(pose_offset[0]),
        "Y": float(pose_offset[1]),
        "Z": float(pose_offset[2]),
        "Pitch": camera_pitch,
        "Roll": camera_roll,
        "Yaw": camera_yaw,
        "CaptureSettings": [
            {
                "ImageType": 0,
                "Width": capture_width,
                "Height": capture_height,
                "FOV_Degrees": float(vision_cfg.get("fov_degrees", horizontal_fov_degrees())),
            }
        ],
    }
    return {
        camera_name: front_camera_settings,
        "front_center": dict(front_camera_settings),
    }


def assert_px4_hil_vehicle(vehicle: dict[str, Any]) -> None:
    """Guard against regressions that break PX4-SITL (UDP-only / wrong TCP port)."""
    if vehicle.get("VehicleType") != "PX4Multirotor":
        raise ValueError(
            f"expected VehicleType PX4Multirotor, got {vehicle.get('VehicleType')!r}"
        )
    if not vehicle.get("UseTcp"):
        raise ValueError("PX4 HIL requires UseTcp=true in AirSim settings")
    if int(vehicle.get("TcpPort", 0)) != PX4_HIL_TCP_PORT:
        raise ValueError(
            f"PX4 HIL TcpPort must be {PX4_HIL_TCP_PORT}, got {vehicle.get('TcpPort')!r}"
        )


def px4_hil_vehicle_from_config(config: dict[str, Any], *, enable_trace: bool) -> dict[str, Any]:
    """PX4 vehicle dict: TCP HIL + MAVLink bridge ports from config."""
    vehicle = copy.deepcopy(PX4_VEHICLE_TEMPLATE)
    profile = airsim_mavlink_profile(config)
    vehicle["LockStep"] = bool(profile.get("lock_step", True))
    vehicle["ControlPortLocal"] = int(profile.get("control_port_local", 14540))
    vehicle["ControlPortRemote"] = int(profile.get("control_port_remote", 14580))
    qgc_host_ip = str(profile.get("qgc_host_ip", "127.0.0.1")).strip() or "127.0.0.1"
    vehicle["QgcHostIp"] = qgc_host_ip
    vehicle["QgcPort"] = int(profile.get("qgc_port", 14550))
    vehicle["AllowAPIAlways"] = True
    vehicle["UseTcp"] = True
    vehicle["TcpPort"] = PX4_HIL_TCP_PORT
    vehicle["Cameras"] = camera_block_from_config(config)
    if enable_trace:
        vehicle["EnableTrace"] = True
    assert_px4_hil_vehicle(vehicle)
    return vehicle


def recording_block() -> dict[str, Any]:
    return {
        "Cameras": [
            {"CameraName": "0", "ImageType": 0, "PixelsAsFloat": False, "Compress": False}
        ]
    }


def apply_viewport_to_settings(settings: dict[str, Any], viewport: ViewportOptions) -> None:
    normalized = normalize_view_mode(viewport.view_mode)
    settings["ViewMode"] = normalized
    settings.pop("SubWindows", None)
    settings.pop("CameraDirector", None)
    if viewport.corner_chase_pip:
        settings["SubWindows"] = [
            {
                "WindowID": 0,
                "ImageType": 0,
                "CameraName": "0",
                "External": False,
                "Visible": True,
            }
        ]
    if normalized == "Fpv":
        settings["CameraDirector"] = {"FollowDistance": -50.0}


def load_settings_file(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        print(f"Warning: invalid AirSim settings JSON at {path}; rewriting file.")
        return {}
    return loaded if isinstance(loaded, dict) else {}


def write_settings_file(path: Path, settings: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(settings, indent=2), encoding="utf-8")


def backup_simpleflight_if_needed(path: Path, settings: dict[str, Any]) -> None:
    backup_path = path.with_name(SIMPLEFLIGHT_BACKUP_NAME)
    vehicles = settings.get("Vehicles")
    is_simple = isinstance(vehicles, dict) and any(
        isinstance(v, dict) and v.get("VehicleType") == "SimpleFlight"
        for v in vehicles.values()
    )
    if is_simple and not backup_path.is_file():
        backup_path.write_text(json.dumps(settings, indent=2), encoding="utf-8")
        print(f"[launcher] backed up SimpleFlight settings to {backup_path}")


def base_multirotor_shell(airsim_port: int, viewport: ViewportOptions) -> dict[str, Any]:
    return {
        "SettingsVersion": SETTINGS_VERSION,
        "SimMode": "Multirotor",
        "LocalHostIp": "127.0.0.1",
        "ApiServerPort": int(airsim_port),
        "Recording": recording_block(),
    }


def ensure_px4_hil_settings(
    airsim_port: int,
    config: dict[str, Any],
    *,
    viewport: ViewportOptions | None = None,
) -> Path:
    """Write PX4Multirotor + TCP HIL settings (used by ``uv run sim`` in mavlink mode)."""
    vp = viewport or ViewportOptions()
    path = settings_path()
    existing = load_settings_file(path)
    backup_simpleflight_if_needed(path, existing)

    px4_vehicle = px4_hil_vehicle_from_config(config, enable_trace=vp.enable_trace)
    settings = base_multirotor_shell(airsim_port, vp)
    settings["ClockType"] = "SteppableClock"
    settings["Vehicles"] = {PX4_VEHICLE_NAME: px4_vehicle}
    apply_viewport_to_settings(settings, vp)

    write_settings_file(path, settings)
    normalized = normalize_view_mode(vp.view_mode)
    print(
        f"Configured AirSim PX4 HIL (TCP :{PX4_HIL_TCP_PORT}), "
        f"ViewMode={normalized} in {path}"
    )
    return path


def ensure_launch_settings(
    transport: str,
    airsim_port: int,
    config: dict[str, Any],
    *,
    view_mode: str = "Fpv",
    corner_chase_pip: bool = False,
    enable_trace: bool = False,
) -> Path | None:
    """Single entry for launcher: mavlink → PX4 HIL; airsim → caller handles SimpleFlight."""
    if str(transport).strip().lower() == "mavlink":
        return ensure_px4_hil_settings(
            airsim_port,
            config,
            viewport=ViewportOptions(
                view_mode=view_mode,
                corner_chase_pip=corner_chase_pip,
                enable_trace=enable_trace,
            ),
        )
    return None
