"""Start UE5 Colosseum (if configured), wait, then run main.py."""

import json
import os
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


@dataclass
class _LaunchHandles:
    """Processes started by this launcher (for Ctrl+C / SIGINT cleanup)."""

    ue: subprocess.Popen | None = None
    main: subprocess.Popen | None = None
    cleanup_done: bool = False


_handles = _LaunchHandles()
_signals_registered: bool = False


def _cleanup_on_interrupt() -> None:
    """Terminate drone client, then Unreal if we launched it."""
    if _handles.cleanup_done:
        return
    _handles.cleanup_done = True
    print("\nInterrupt received — stopping drone client and simulator...", file=sys.stderr)
    if _handles.main is not None and _handles.main.poll() is None:
        _handles.main.terminate()
        try:
            _handles.main.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            _handles.main.kill()
            try:
                _handles.main.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                pass
    if _handles.ue is not None and _handles.ue.poll() is None:
        _handles.ue.terminate()
        try:
            _handles.ue.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            _handles.ue.kill()
            try:
                _handles.ue.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                pass


def _sigint_handler(_signum: int, _frame) -> None:
    """SIGINT (Ctrl+C) runs full teardown; SIGTERM is not handled here."""
    _cleanup_on_interrupt()
    raise SystemExit(130)


def _register_signal_handlers_once() -> None:
    global _signals_registered
    if _signals_registered:
        return
    signal.signal(signal.SIGINT, _sigint_handler)
    _signals_registered = True


def _airsim_settings_path() -> Path:
    return Path.home() / "Documents" / "AirSim" / "settings.json"


def _deep_merge_dict(base: dict, update: dict) -> dict:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_dict(base[key], value)
        else:
            base[key] = value
    return base


def _normalize_view_mode(view_mode: str) -> str:
    mode = view_mode.strip().lower()
    if mode in {"3rd-person", "third-person", "flywithme"}:
        return "FlyWithMe"
    return "Fpv"


def _ensure_camera_settings(
    airsim_port: int,
    view_mode: str,
    *,
    corner_chase_pip: bool,
    enable_trace: bool,
    config: dict,
    transport: str,
    use_vjoy: bool = False,
) -> None:
    settings_path = _airsim_settings_path()
    settings_path.parent.mkdir(parents=True, exist_ok=True)

    settings: dict = {}
    if settings_path.is_file():
        try:
            settings = json.loads(settings_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            print(f"Warning: invalid AirSim settings JSON at {settings_path}; rewriting file.")

    vehicles = settings.get("Vehicles")
    if isinstance(vehicles, dict):
        # Drop any vehicle that isn't "Drone1" — AirSim spawns one drone per
        # entry, so a stray "SimpleFlight" / "Drone2" / etc. causes duplicate
        # drones to appear stacked at spawn. Drone1 is the only name the rest
        # of this codebase uses.
        for stray in [name for name in vehicles.keys() if name != "Drone1"]:
            vehicles.pop(stray, None)
            print(
                f"[launcher] Removed stray vehicle '{stray}' from settings "
                "to prevent duplicate spawn."
            )

        drone1 = vehicles.get("Drone1")
        if isinstance(drone1, dict):
            keys = set(drone1.keys())
            cameras = drone1.get("Cameras")
            if keys == {"EnableTrace"}:
                vehicles.pop("Drone1", None)
            if (
                keys <= {"VehicleType", "Cameras"}
                and isinstance(cameras, dict)
                and {"0", "front_center"}.issubset(set(cameras.keys()))
            ):
                vehicles.pop("Drone1", None)
        if not vehicles:
            settings.pop("Vehicles", None)

    normalized_view_mode = _normalize_view_mode(view_mode)
    transport_l = str(transport).strip().lower()
    vehicle_name = "Drone1"
    vision_cfg = config.get("vision", {})
    camera_cfg = config.get("camera", {})
    pose_offset = camera_cfg.get("pose_offset", [0.35, 0.0, -0.05])
    camera_pitch = float(camera_cfg.get("pitch_up_degrees", 20.0))
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
                "FOV_Degrees": float(vision_cfg.get("fov_degrees", 100.0)),
            }
        ],
    }
    cameras_settings = {
        camera_name: front_camera_settings,
        "front_center": dict(front_camera_settings),
    }
    if transport_l == "mavlink":
        mav_cfg = config.get("control", {}).get("mavlink", {})
        airsim_mav_cfg = mav_cfg.get("airsim_profile", {})
        udp_ip = str(airsim_mav_cfg.get("udp_ip", "127.0.0.1")).strip() or "127.0.0.1"
        udp_port = int(airsim_mav_cfg.get("udp_port", 14560))
        control_port_local = int(airsim_mav_cfg.get("control_port_local", 14540))
        control_port_remote = int(airsim_mav_cfg.get("control_port_remote", 14580))
        qgc_host_ip = str(airsim_mav_cfg.get("qgc_host_ip", "127.0.0.1")).strip() or "127.0.0.1"
        qgc_port = int(airsim_mav_cfg.get("qgc_port", 14550))
        vehicle_type = str(airsim_mav_cfg.get("vehicle_type", "PX4Multirotor")).strip()
        allowed_mav = {"px4multirotor", "arducopter", "ardurover", "arducoptersolo"}
        if vehicle_type.lower() not in allowed_mav:
            print(
                "Warning: control.mavlink.airsim_profile.vehicle_type="
                f"{vehicle_type!r} is not a supported AirSim MAVLink backend; "
                "forcing PX4Multirotor."
            )
            vehicle_type = "PX4Multirotor"
        vehicle_settings = {
            "VehicleType": vehicle_type,
            "UseSerial": False,
            "UseTcp": False,
            "LockStep": bool(airsim_mav_cfg.get("lock_step", False)),
            "UdpIp": udp_ip,
            "UdpPort": udp_port,
            "ControlIp": "127.0.0.1",
            "ControlPortLocal": control_port_local,
            "ControlPortRemote": control_port_remote,
            "QgcHostIp": qgc_host_ip,
            "QgcPort": qgc_port,
            "AllowAPIAlways": True,
            "EnableTrace": bool(enable_trace),
            "Cameras": cameras_settings,
        }
    else:
        vehicle_settings = {
            "VehicleType": "SimpleFlight",
            "AllowAPIAlways": True,
            "EnableTrace": bool(enable_trace),
            "Cameras": cameras_settings,
        }

    required_settings = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ViewMode": normalized_view_mode,
        "LocalHostIp": "127.0.0.1",
        "ApiServerPort": int(airsim_port),
        "Vehicles": {vehicle_name: vehicle_settings},
        "Recording": {
            "Cameras": [
                {"CameraName": "0", "ImageType": 0, "PixelsAsFloat": False, "Compress": False}
            ]
        },
    }
    if use_vjoy:
        required_settings["Usage"] = "vJoy"
        print("[launcher] AirSim configured for vJoy manual control.")

    if corner_chase_pip:
        # Use forward camera for PiP; "Chase" crashes on some builds.
        required_settings["SubWindows"] = [
            {
                "WindowID": 0,
                "ImageType": 0,
                "CameraName": "0",
                "External": False,
                "Visible": True,
            }
        ]
    # Avoid SubWindows — camera name lookup crashes on some builds.
    # FPV: pull back external camera. FlyWithMe uses stock defaults.
    if normalized_view_mode == "Fpv":
        required_settings["CameraDirector"] = {"FollowDistance": -50.0}

    merged = _deep_merge_dict(settings, required_settings)
    merged["Vehicles"] = dict(required_settings["Vehicles"])
    merged.pop("SimpleFlight", None)
    # Drop stale SubWindows, re-add only the safe PiP above.
    merged.pop("SubWindows", None)
    if "SubWindows" in required_settings:
        merged["SubWindows"] = required_settings["SubWindows"]

    if use_vjoy:
        # Clear specific vehicles to prevent AirSim from spawning a second drone
        # when 'Usage: vJoy' is also set at the top level.
        merged.pop("Vehicles", None)
        print("[launcher] Cleared 'Vehicles' from settings for vJoy mode.")

    if enable_trace and not use_vjoy:
        # Enable AirSim trace for third-person mode (only if not in vJoy mode
        # to keep settings clean and avoid double-spawn).
        merged_vehicles = merged.get("Vehicles")
        if not isinstance(merged_vehicles, dict):
            merged_vehicles = {}
            merged["Vehicles"] = merged_vehicles
        merged_drone1 = merged_vehicles.get("Drone1")
        if not isinstance(merged_drone1, dict):
            merged_drone1 = {"VehicleType": "SimpleFlight"}
            merged_vehicles["Drone1"] = merged_drone1
        merged_drone1["EnableTrace"] = True
    if normalized_view_mode == "FlyWithMe":
        merged.pop("CameraDirector", None)

    settings_path.write_text(json.dumps(merged, indent=2), encoding="utf-8")
    print(f"Configured AirSim ViewMode={normalized_view_mode} in {settings_path}")


def _is_port_open(host: str, port: int, timeout_s: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def _wait_for_airsim_rpc(host: str, port: int, timeout_s: float) -> bool:
    """Wait until AirSim is actually answering RPCs with a spawned multirotor.

    The TCP port can open seconds before the vehicle is spawned in the Unreal
    scene, so issuing real commands during that window produces flaky startup
    errors. Probe both ping() and getMultirotorState() to cover the full ready
    path. Use a throwaway client per attempt so a failed RPC can't leave dirty
    state behind.
    """
    import airsim as _airsim  # local import: keep module load cheap

    deadline = time.time() + max(1.0, timeout_s)
    port_seen_open = False
    while time.time() < deadline:
        if not port_seen_open:
            if not _is_port_open(host, port):
                time.sleep(1.0)
                continue
            port_seen_open = True
        probe_client = None
        try:
            probe_client = _airsim.MultirotorClient(ip=host, port=port, timeout_value=3)
            if probe_client.ping() is True:
                probe_client.getMultirotorState()
                return True
        except Exception:
            pass
        finally:
            if probe_client is not None:
                try:
                    probe_client.client.close()
                except Exception:
                    pass
        time.sleep(1.0)
    return False


def _wait_for_control_link(
    host: str,
    airsim_port: int,
    config: dict,
    wait_timeout_s: float,
    transport: str,
    *,
    require_requested_transport: bool = False,
) -> tuple[str, str | None]:
    from src.mavlink_endpoints import (
        describe_mavlink_heartbeat_failure,
        probe_mavlink_heartbeat,
    )

    requested = str(transport).strip().lower()
    timeout_s = max(15.0, float(wait_timeout_s))
    if requested != "mavlink":
        if _wait_for_airsim_rpc(host, airsim_port, timeout_s):
            return "airsim", None
        raise SystemExit(
            f"AirSim RPC did not become ready on {host}:{airsim_port} "
            f"within {timeout_s:.0f}s. Ensure Unreal finished loading the map."
        )

    strict = require_requested_transport or os.environ.get("AIGP_MAVLINK_STRICT", "").strip() == "1"
    mav_phase_s = min(30.0, max(8.0, timeout_s * 0.25))
    probe = probe_mavlink_heartbeat(config, timeout_s=mav_phase_s)
    if probe.endpoint is not None:
        return "mavlink", probe.endpoint

    failure_detail = describe_mavlink_heartbeat_failure(config, probe)

    if strict:
        raise SystemExit(failure_detail)

    rest_s = max(10.0, timeout_s - mav_phase_s)
    print(
        f"{failure_detail} "
        f"Trying AirSim RPC for up to {rest_s:.0f}s. "
        'Set AIGP_MAVLINK_STRICT=1 to require MAVLink.'
    )
    if _wait_for_airsim_rpc(host, airsim_port, rest_s):
        print("AirSim RPC is ready; using AirSim transport for this session.")
        return "airsim", None

    raise SystemExit(
        f"{failure_detail} AirSim RPC was also not ready on {host}:{airsim_port} "
        f"within {timeout_s:.0f}s."
    )


def _load_env_local() -> None:
    path = ROOT / ".env.local"
    if not path.is_file():
        return
    text = path.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        line = line.strip().lstrip("\ufeff")
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :].strip()
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key:
            os.environ[key] = value


def _resolve_project_path(sim_cfg: dict) -> str:
    project = os.environ.get("PROJECT_PATH", "").strip()
    if project:
        return project

    config_project = str(sim_cfg.get("project_path", "")).strip()
    if config_project:
        os.environ["PROJECT_PATH"] = config_project
        return config_project

    fix_script = ROOT / "scripts" / "fix_project_path.ps1"
    if sys.platform == "win32" and fix_script.is_file():
        subprocess.run(
            [
                "powershell",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(fix_script),
            ],
            check=False,
        )
        _load_env_local()
        project = os.environ.get("PROJECT_PATH", "").strip()
        if project:
            return project

    return ""


PX4_VEHICLE_TEMPLATE: dict = {
    "VehicleType": "PX4Multirotor",
    "UseSerial": False,
    "UseTcp": True,
    "TcpPort": 4560,
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
PX4_HIL_TCP_PORT = 4560


def _ensure_px4_mavlink_settings(airsim_port: int) -> Path:
    settings_path = _airsim_settings_path()
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    settings: dict = {}
    if settings_path.is_file():
        try:
            settings = json.loads(settings_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            print(f"[mavlink] Warning: invalid AirSim settings at {settings_path}; rewriting.")

    backup_path = settings_path.with_name("settings.simpleflight.bak.json")
    is_simple = (
        isinstance(settings.get("Vehicles"), dict)
        and any(
            isinstance(v, dict) and v.get("VehicleType") == "SimpleFlight"
            for v in settings["Vehicles"].values()
        )
    )
    if is_simple and not backup_path.is_file():
        backup_path.write_text(json.dumps(settings, indent=2), encoding="utf-8")
        print(f"[mavlink] backed up SimpleFlight settings to {backup_path}")

    settings["SettingsVersion"] = 1.2
    settings["SimMode"] = "Multirotor"
    settings["ApiServerPort"] = int(airsim_port)
    settings["ClockType"] = "SteppableClock"
    settings["Vehicles"] = {"PX4": json.loads(json.dumps(PX4_VEHICLE_TEMPLATE))}
    settings.pop("SubWindows", None)
    settings.pop("CameraDirector", None)

    settings_path.write_text(json.dumps(settings, indent=2), encoding="utf-8")
    print(f"[mavlink] wrote PX4Multirotor settings to {settings_path}")
    return settings_path


def _is_tcp_listening_passive(port: int) -> bool:
    """Check if any process is LISTENING on a TCP port WITHOUT opening a connection.

    AirSim's PX4Multirotor HIL accepts exactly one TCP connection then stops listening
    (single-shot acceptTcp at MavLinkMultirotorApi.hpp:1317). An active probe like
    socket.create_connection() would consume the slot, leaving the real PX4 unable to
    connect. Use kernel state (netstat) instead.
    """
    try:
        if sys.platform == "win32":
            out = subprocess.run(
                ["netstat", "-ano", "-p", "TCP"],
                capture_output=True, text=True, timeout=5,
            )
            needle = f":{port} "
            for line in out.stdout.splitlines():
                if needle in line and "LISTENING" in line:
                    return True
            return False
        out = subprocess.run(
            ["ss", "-tln"], capture_output=True, text=True, timeout=5,
        )
        needle = f":{port} "
        return any(needle in line for line in out.stdout.splitlines())
    except (OSError, subprocess.SubprocessError):
        return False


def _windows_host_ip_for_wsl() -> str:
    try:
        out = subprocess.run(
            ["wsl", "-e", "bash", "-c", "ip route show default | awk '{print $3}'"],
            capture_output=True, text=True, timeout=5,
        )
        ip = out.stdout.strip()
        if ip and ip.count(".") == 3:
            return ip
    except (OSError, subprocess.SubprocessError):
        pass
    return "127.0.0.1"


def _print_px4_bringup_instructions(wsl_host_ip: str) -> None:
    make_cmd = (
        f"cd ~/PX4-Autopilot && PX4_SIM_HOST_ADDR={wsl_host_ip} make px4_sitl none_iris"
    )
    print("\n[mavlink] === Start PX4-SITL in a SECOND terminal ===")
    print("  From Windows PowerShell:")
    print(f"    wsl -d Ubuntu -e bash -c '{make_cmd}'")
    print("  From inside WSL Ubuntu:")
    print(f"    {make_cmd}")
    print(
        "\nLook for 'INFO  [simulator_mavlink] Simulator connected on TCP port 4560.' "
        "in the PX4 log."
    )
    print(
        "If you see 'INFO  [init] SIH simulator' instead, PX4 is using its built-in "
        "simulator and will never connect to AirSim -- rebuild with `make px4_sitl none_iris`."
    )
    print(
        "Once connected, AirSim will push PX4's MAVLink to 127.0.0.1:14550 (the probe). "
        "Frame rate ~2 Hz heartbeat is enough to PASS.\n"
    )


def _build_mavlink_launch_plan() -> dict:
    """Resolve UE launch invocation for MAVLink mode without spawning anything.

    Used by the PowerShell orchestrator (scripts/dev-mavlink.ps1) which owns
    process lifecycle. Writes PX4Multirotor settings.json as a side effect.
    """
    _load_env_local()
    from src.config import load_config

    config = load_config()
    sim_cfg = config["simulator"]
    airsim_port = int(sim_cfg.get("airsim_port", 41451))
    settings_path = _ensure_px4_mavlink_settings(airsim_port)

    colosseum = sim_cfg.get("colosseum_path", "")
    project = _resolve_project_path(sim_cfg)
    if not colosseum or not Path(colosseum).exists():
        return {"error": f"colosseum_path not found: {colosseum!r}"}
    if not project:
        return {"error": "PROJECT_PATH not set or .uproject missing"}

    args = [project, "-game", f"-settings={settings_path}"]
    windowed = sim_cfg.get("windowed", True)
    res_x = sim_cfg.get("res_x", 1280)
    res_y = sim_cfg.get("res_y", 720)
    if windowed:
        args.extend(["-windowed", f"-resx={res_x}", f"-resy={res_y}"])
    extra = sim_cfg.get("extra_ue_args") or []
    if isinstance(extra, list):
        args.extend(str(a) for a in extra if str(a).strip())

    return {
        "colosseum": str(colosseum),
        "args": args,
        "settings_path": str(settings_path),
        "airsim_port": airsim_port,
        "hil_tcp_port": PX4_HIL_TCP_PORT,
    }


def print_mavlink_launch_plan() -> None:
    """CLI: emit JSON launch plan for the orchestrator. Exit 1 on error."""
    plan = _build_mavlink_launch_plan()
    print(json.dumps(plan))
    if "error" in plan:
        raise SystemExit(1)


def restore_simpleflight_settings() -> bool:
    """Restore SimpleFlight settings.json from backup if present. Returns True if restored."""
    settings_path = _airsim_settings_path()
    backup_path = settings_path.with_name("settings.simpleflight.bak.json")
    if backup_path.is_file():
        settings_path.write_text(backup_path.read_text(encoding="utf-8"), encoding="utf-8")
        return True
    return False


def _maybe_restore_simpleflight_from_backup() -> bool:
    # Self-heal for users who ran sim-mavlink/mavlink-all and skipped Ctrl+C
    # cleanup (Task Manager kill, BSOD). Only restores when the current file
    # still looks like PX4Multirotor, so a freshly-edited SimpleFlight config
    # is never clobbered.
    settings_path = _airsim_settings_path()
    backup_path = settings_path.with_name("settings.simpleflight.bak.json")
    if not (settings_path.is_file() and backup_path.is_file()):
        return False
    try:
        current = json.loads(settings_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    vehicles = current.get("Vehicles")
    is_px4 = isinstance(vehicles, dict) and any(
        isinstance(v, dict) and v.get("VehicleType") == "PX4Multirotor"
        for v in vehicles.values()
    )
    if not is_px4:
        return False
    settings_path.write_text(backup_path.read_text(encoding="utf-8"), encoding="utf-8")
    print(
        f"[launcher] Restored SimpleFlight from {backup_path.name} "
        "(prior MAVLink session left PX4Multirotor settings)."
    )
    return True


def main_restore_simpleflight() -> None:
    print("restored" if restore_simpleflight_settings() else "no-backup")


def main_mavlink_all() -> None:
    """CLI: invoke the PowerShell orchestrator (dev-mavlink.ps1)."""
    if sys.platform != "win32":
        raise SystemExit("mavlink-all is Windows-only (uses PowerShell + WSL).")
    script = ROOT / "scripts" / "dev-mavlink.ps1"
    if not script.is_file():
        raise SystemExit(f"orchestrator script not found: {script}")
    cmd = [
        "powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", str(script), *sys.argv[1:],
    ]
    raise SystemExit(subprocess.call(cmd))


def launch_mavlink(*, run_probe: bool = False, probe_seconds: float = 60.0) -> None:
    _register_signal_handlers_once()
    _handles.ue = None
    _handles.main = None
    _handles.cleanup_done = False

    _load_env_local()

    from src.config import load_config

    config = load_config()
    sim_cfg = config["simulator"]
    colosseum = sim_cfg.get("colosseum_path", "")
    project = _resolve_project_path(sim_cfg)
    windowed = sim_cfg.get("windowed", True)
    res_x = sim_cfg.get("res_x", 1280)
    res_y = sim_cfg.get("res_y", 720)
    airsim_port = int(sim_cfg.get("airsim_port", 41451))

    _ensure_px4_mavlink_settings(airsim_port)

    if colosseum and Path(colosseum).exists():
        if not project:
            raise SystemExit(
                "PROJECT_PATH not set to a valid .uproject. "
                "See README for setup."
            )
        print(f"[mavlink] launching UE with PX4Multirotor settings: {colosseum}")
        cmd = [colosseum, project, "-game", f"-settings={_airsim_settings_path()}"]
        if windowed:
            cmd.extend(["-windowed", f"-resx={res_x}", f"-resy={res_y}"])
        extra = sim_cfg.get("extra_ue_args") or []
        if isinstance(extra, list):
            cmd.extend(str(a) for a in extra if str(a).strip())
        _handles.ue = subprocess.Popen(
            cmd,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == "win32" else 0,
        )
        print(
            f"[mavlink] waiting (passively) for AirSim HIL TCP listener on "
            f":{PX4_HIL_TCP_PORT}..."
        )
        deadline = time.time() + 120.0
        ready = False
        while time.time() < deadline:
            if _is_tcp_listening_passive(PX4_HIL_TCP_PORT):
                ready = True
                break
            time.sleep(1.0)
        if not ready:
            raise SystemExit(
                f"[mavlink] AirSim HIL TCP listener never came up on :{PX4_HIL_TCP_PORT}. "
                "Check UE logs and that the AirSim plugin is loaded."
            )
        print(f"[mavlink] AirSim HIL listener is ready on :{PX4_HIL_TCP_PORT}")
    else:
        print(
            f"[mavlink] colosseum not found at '{colosseum}'; "
            "start UE manually with the new settings."
        )

    _print_px4_bringup_instructions(_windows_host_ip_for_wsl())

    if run_probe:
        print(
            f"[mavlink] running probe for {probe_seconds:.0f}s -- start PX4-SITL now.\n"
        )
        try:
            rc = subprocess.call(
                [sys.executable, "-m", "src.check_mavlink",
                 "--duration", str(probe_seconds)],
            )
        except KeyboardInterrupt:
            _cleanup_on_interrupt()
            raise SystemExit(130) from None

        print()
        if rc == 0:
            print("[mavlink] SUCCESS: Colosseum + PX4-SITL emitted MAVLink.")
        else:
            print(
                "[mavlink] FAIL: probe saw no MAVLink. Verify (1) PX4 actually started, "
                "(2) PX4 logged 'Simulator connected on TCP port 4560', (3) Windows "
                "Defender Firewall isn't blocking inbound UDP from WSL."
            )
        raise SystemExit(rc)

    print(
        "[mavlink] UE is running. Run `uv run check-mavlink` in another terminal "
        "to verify MAVLink output."
    )
    print("[mavlink] Press Ctrl+C here to stop UE and exit.\n")
    try:
        if _handles.ue is not None:
            _handles.ue.wait()
    except KeyboardInterrupt:
        _cleanup_on_interrupt()
        raise SystemExit(130) from None


def main_mavlink() -> None:
    args = {a.strip().lower() for a in sys.argv[1:]}
    run_probe = "probe" in args or "--probe" in args
    launch_mavlink(run_probe=run_probe)


def launch(
    *,
    landing_profile: str | None = None,
    low_end: bool = False,
    view_mode: str = "Fpv",
    corner_chase_pip: bool = False,
    enable_trace: bool = False,
    manual_gui: bool = False,
    manual_debug: bool = False,
    use_vjoy: bool = False,
    script_path: str = "main.py",
    require_requested_transport: bool = False,
) -> None:
    _register_signal_handlers_once()
    _handles.ue = None
    _handles.main = None
    _handles.cleanup_done = False

    _load_env_local()

    from src.config import load_config
    from src.mavlink_endpoints import first_mavlink_heartbeat_endpoint, resolve_control_transport

    config = load_config()
    sim_cfg = config["simulator"]
    transport = resolve_control_transport(config)
    resolved_transport = transport
    resolved_mavlink_endpoint: str | None = None
    if transport == "mavlink":
        resolved_mavlink_endpoint = first_mavlink_heartbeat_endpoint(config, timeout_s=2.0)

    colosseum = sim_cfg.get("colosseum_path", "")
    project = _resolve_project_path(sim_cfg)
    windowed = sim_cfg.get("windowed", True)
    res_x = sim_cfg.get("res_x", 1280)
    res_y = sim_cfg.get("res_y", 720)
    if low_end:
        low_end_cfg = config.get("low_end_profile", {})
        windowed = True
        res_x = int(low_end_cfg.get("sim_res_x", 854))
        res_y = int(low_end_cfg.get("sim_res_y", 480))
    host = str(sim_cfg.get("host", "127.0.0.1")).strip() or "127.0.0.1"
    airsim_port = int(sim_cfg.get("airsim_port", 41451))
    rpc_ready_timeout_s = max(15.0, float(sim_cfg.get("rpc_ready_timeout_seconds", 120.0)))
    rpc_tout_label = f"{rpc_ready_timeout_s:.0f}"
    _maybe_restore_simpleflight_from_backup()
    _ensure_camera_settings(
        airsim_port,
        view_mode,
        corner_chase_pip=corner_chase_pip,
        enable_trace=enable_trace,
        config=config,
        transport=transport,
        use_vjoy=use_vjoy,
    )

    if resolved_mavlink_endpoint is not None:
        print(
            "MAVLink heartbeat already available on "
            f"{resolved_mavlink_endpoint}; skipping simulator launch."
        )
    elif colosseum and Path(colosseum).exists():
        if not project:
            raise SystemExit(
                "PROJECT_PATH is not set to a valid .uproject file.\n"
                "Fix it by either:\n"
                "  1) setting PROJECT_PATH in .env.local, or\n"
                "  2) adding simulator.project_path in sim.config.json, or\n"
                "  3) running: pwsh scripts/fix_project_path.ps1\n"
            )
        if low_end:
            print(f"Launching Colosseum (low-end): {colosseum}")
        else:
            print(f"Launching Colosseum: {colosseum}")
        cmd = [colosseum]
        cmd.append(project)
        cmd.append("-game")
        cmd.append(f"-settings={_airsim_settings_path()}")
        if windowed:
            cmd.extend(["-windowed", f"-resx={res_x}", f"-resy={res_y}"])
        extra = sim_cfg.get("extra_ue_args")
        extra_list = extra if isinstance(extra, list) else []
        extra_str = extra.strip() if isinstance(extra, str) else ""
        if extra_list:
            cmd.extend(str(arg) for arg in extra_list if str(arg).strip())
        elif extra_str:
            cmd.append(extra_str)
        map_hint = str(sim_cfg.get("map_name", "")).strip()
        has_map_launch_args = any(str(a).strip() for a in extra_list) or bool(extra_str)
        if map_hint and not has_map_launch_args:
            print(
                f"Note: simulator.map_name={map_hint!r} is informational only; "
                "add simulator.extra_ue_args (e.g. -map=/Game/...) to force a map at launch."
            )

        _handles.ue = subprocess.Popen(
            cmd,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == "win32" else 0,
        )
        wait_label = "MAVLink/AirSim control link" if transport == "mavlink" else "AirSim RPC"
        print(f"Waiting for {wait_label} on {host}:{airsim_port} (timeout {rpc_tout_label}s)...")
        try:
            resolved_transport, resolved_mavlink_endpoint = _wait_for_control_link(
                host,
                airsim_port,
                config,
                rpc_ready_timeout_s,
                transport,
                require_requested_transport=require_requested_transport,
            )
        except KeyboardInterrupt:
            _cleanup_on_interrupt()
            raise SystemExit(130) from None
        if resolved_transport == "mavlink" and resolved_mavlink_endpoint is not None:
            print(f"MAVLink heartbeat detected on {resolved_mavlink_endpoint}")
        else:
            print(f"AirSim RPC is ready on {host}:{airsim_port}")
    else:
        print(f"Colosseum not found at '{colosseum}', skipping simulator launch.")
        wait_label = "MAVLink/AirSim control link" if transport == "mavlink" else "AirSim RPC"
        print(f"Waiting for {wait_label} on {host}:{airsim_port} (timeout {rpc_tout_label}s)...")
        try:
            resolved_transport, resolved_mavlink_endpoint = _wait_for_control_link(
                host,
                airsim_port,
                config,
                rpc_ready_timeout_s,
                transport,
                require_requested_transport=require_requested_transport,
            )
        except KeyboardInterrupt:
            _cleanup_on_interrupt()
            raise SystemExit(130) from None
        if resolved_transport == "mavlink" and resolved_mavlink_endpoint is not None:
            print(f"MAVLink heartbeat detected on {resolved_mavlink_endpoint}")
        else:
            print(f"AirSim RPC is ready on {host}:{airsim_port}")

    env = os.environ.copy()
    env["AIRSIM_PORT"] = str(airsim_port)
    env["AIGP_CONTROL_TRANSPORT"] = resolved_transport
    if resolved_mavlink_endpoint is not None:
        env["AIGP_MAVLINK_ENDPOINT"] = resolved_mavlink_endpoint
    else:
        env.pop("AIGP_MAVLINK_ENDPOINT", None)
    if landing_profile:
        env["AIGP_LANDING_PROFILE"] = landing_profile
    if low_end:
        env["AIGP_LOW_END"] = "1"
    if enable_trace:
        env["AIGP_ENABLE_TRACE"] = "1"

    # Start the secondary process (GUI) if requested
    gui_proc = None
    if manual_gui:
        gui_path = ROOT / "manual_flight_gui.py"
        if not gui_path.is_file():
            print(f"Warning: Manual flight GUI not found at {gui_path}. Skipping.")
        else:
            probe = subprocess.run(
                [sys.executable, "-c", "import pyvjoy, pynput, PySimpleGUI"],
                capture_output=True,
            )
            if probe.returncode != 0:
                print(
                    "Warning: manual flight extras not installed in this venv "
                    "(pyvjoy / pynput / PySimpleGUI). Skipping GUI. "
                    "Run: uv sync --extra manual"
                )
            else:
                gui_cmd = [sys.executable, str(gui_path), "--vjoy"]
                if manual_debug:
                    gui_cmd.append("--debug")
                print(f"Starting manual_flight_gui.py ({' '.join(gui_cmd[2:])})...")
                gui_proc = subprocess.Popen(gui_cmd, env=env)

    # Start the primary script
    print(f"Starting {script_path}...")
    _handles.main = subprocess.Popen(
        [sys.executable, str(ROOT / script_path)],
        env=env,
    )

    try:
        rc = _handles.main.wait()
    except KeyboardInterrupt:
        _cleanup_on_interrupt()
        raise SystemExit(130) from None
    finally:
        if gui_proc and gui_proc.poll() is None:
            gui_proc.terminate()
        _handles.main = None

    if rc == 0:
        print(
            f"{script_path} exited successfully. If this launcher started Unreal/Colosseum, "
            "that process may still be running — close it from the editor or Task Manager "
            "if needed.",
            file=sys.stderr,
        )
    else:
        print(f"{script_path} exited with code {rc}. See logs above.", file=sys.stderr)
    raise SystemExit(rc)


def _parse_manual_flags(args: set[str]) -> tuple[bool, bool]:
    manual = "vjoy" in args or "manual" in args
    debug = "debug" in args
    return manual, debug


def main() -> None:
    normalized_args = {arg.strip().lower() for arg in sys.argv[1:]}
    run_low_end = "low-end" in normalized_args
    run_third_person = "3rd-person" in normalized_args or "third-person" in normalized_args
    manual_gui, manual_debug = _parse_manual_flags(normalized_args)
    view_mode = "FlyWithMe" if run_third_person else "Fpv"
    launch(
        low_end=run_low_end,
        view_mode=view_mode,
        corner_chase_pip=run_third_person,
        enable_trace=run_third_person,
        manual_gui=manual_gui,
        manual_debug=manual_debug,
    )


def main_calibrate() -> None:
    # Calibration command always includes manual control GUI for ease of use
    launch(
        view_mode="FlyWithMe",
        use_vjoy=True,
        manual_gui=True,
        script_path="scripts/calibrate_depth.py"
    )


def main_very_soft() -> None:
    normalized_args = {arg.strip().lower() for arg in sys.argv[1:]}
    manual_gui, manual_debug = _parse_manual_flags(normalized_args)
    launch(landing_profile="very_soft", manual_gui=manual_gui, manual_debug=manual_debug)


def main_low_end() -> None:
    normalized_args = {arg.strip().lower() for arg in sys.argv[1:]}
    manual_gui, manual_debug = _parse_manual_flags(normalized_args)
    launch(
        low_end=True,
        corner_chase_pip=False,
        manual_gui=manual_gui,
        manual_debug=manual_debug,
    )


def main_timesync_smoke() -> None:
    launch(script_path="src/timesync_smoke.py")


def main_highres_imu_smoke() -> None:
    os.environ["AIGP_CONTROL_TRANSPORT"] = "mavlink"
    os.environ.setdefault("AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT", "1")
    launch(
        script_path="src/highres_imu_smoke.py",
        require_requested_transport=True,
    )


if __name__ == "__main__":
    launch()
