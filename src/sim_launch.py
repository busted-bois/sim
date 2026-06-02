"""Start UE5 Colosseum (if configured), wait, then run main.py."""

import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from src.airsim_settings import (
    PX4_HIL_TCP_PORT,
    ViewportOptions,
    apply_viewport_to_settings,
    base_multirotor_shell,
    camera_block_from_config,
    ensure_launch_settings,
    ensure_px4_hil_settings,
    load_settings_file,
    normalize_view_mode,
)
from src.airsim_settings import (
    settings_path as airsim_settings_path,
)

ROOT = Path(__file__).resolve().parent.parent


@dataclass
class _LaunchHandles:
    """Processes started by this launcher (for Ctrl+C / SIGINT cleanup)."""

    ue: subprocess.Popen | None = None
    main: subprocess.Popen | None = None
    px4: subprocess.Popen | None = None
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
    _stop_px4_wsl_process()


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
    return airsim_settings_path()


def _deep_merge_dict(base: dict, update: dict) -> dict:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_dict(base[key], value)
        else:
            base[key] = value
    return base


def _normalize_view_mode(view_mode: str) -> str:
    return normalize_view_mode(view_mode)


def _px4_hil_vehicle_settings(config: dict, *, enable_trace: bool) -> dict:
    from src.airsim_settings import px4_hil_vehicle_from_config

    return px4_hil_vehicle_from_config(config, enable_trace=enable_trace)


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
    settings = load_settings_file(settings_path)

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
    if transport_l == "mavlink":
        raise ValueError(
            "MAVLink sessions must use _ensure_px4_mavlink_settings (TCP HIL :4560), "
            "not _ensure_camera_settings."
        )
    from src.airsim_settings import SIMPLEFLIGHT_VEHICLE_NAME

    vehicle_name = SIMPLEFLIGHT_VEHICLE_NAME
    cameras_settings = camera_block_from_config(config)
    vehicle_settings = {
        "VehicleType": "SimpleFlight",
        "AllowAPIAlways": True,
        "EnableTrace": bool(enable_trace),
        "Cameras": cameras_settings,
    }

    viewport = ViewportOptions(
        view_mode=view_mode,
        corner_chase_pip=corner_chase_pip,
        enable_trace=enable_trace,
    )
    required_settings = {
        **base_multirotor_shell(airsim_port, viewport),
        "Vehicles": {vehicle_name: vehicle_settings},
    }
    apply_viewport_to_settings(required_settings, viewport)
    if use_vjoy:
        required_settings["Usage"] = "vJoy"
        print("[launcher] AirSim configured for vJoy manual control.")

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
    import airsim

    deadline = time.monotonic() + max(1.0, float(timeout_s))
    poll_s = 0.5
    while time.monotonic() < deadline:
        if _is_port_open(host, port, timeout_s=0.5):
            try:
                client = airsim.MultirotorClient(ip=host, port=port, timeout_value=2)
                client.confirmConnection()
                return True
            except Exception:
                pass
        time.sleep(poll_s)
    return False


def _settings_use_simpleflight() -> bool:
    settings_path = _airsim_settings_path()
    if not settings_path.is_file():
        return False
    try:
        settings = json.loads(settings_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    vehicles = settings.get("Vehicles")
    if not isinstance(vehicles, dict):
        return False
    return any(
        isinstance(v, dict) and v.get("VehicleType") == "SimpleFlight"
        for v in vehicles.values()
    )


def _launch_transport(config: dict, *, restored_simpleflight: bool) -> str:
    from src.mavlink_endpoints import resolve_control_transport

    transport = resolve_control_transport(config)
    if restored_simpleflight or _settings_use_simpleflight():
        if transport != "airsim":
            print("[launcher] SimpleFlight settings - using AirSim RPC transport.")
        return "airsim"
    return transport


def _wait_for_control_link(
    host: str,
    airsim_port: int,
    config: dict,
    wait_timeout_s: float,
    transport: str,
) -> tuple[str, str | None]:
    from src.mavlink_endpoints import first_mavlink_heartbeat_endpoint

    requested = str(transport).strip().lower()
    timeout_s = max(15.0, float(wait_timeout_s))
    if requested != "mavlink":
        if _wait_for_airsim_rpc(host, airsim_port, timeout_s):
            return "airsim", None
        raise SystemExit(
            f"AirSim RPC did not become ready on {host}:{airsim_port} "
            f"within {timeout_s:.0f}s. Ensure Unreal finished loading the map."
        )

    strict = os.environ.get("AIGP_MAVLINK_STRICT", "").strip() == "1"
    deadline = time.time() + timeout_s
    cycle = 0
    printed_mav_fallback = False
    rpc_ready = False
    while time.time() < deadline:
        cycle += 1
        remaining = max(0.1, deadline - time.time())
        resolved = first_mavlink_heartbeat_endpoint(
            config,
            timeout_s=min(2.0, remaining),
        )
        if resolved is not None:
            return "mavlink", resolved

        remaining = max(0.1, deadline - time.time())
        if not rpc_ready:
            rpc_ready = _wait_for_airsim_rpc(host, airsim_port, min(2.0, remaining))
        if rpc_ready:
            if strict:
                raise SystemExit(
                    "AIGP_MAVLINK_STRICT=1: AirSim RPC is ready but "
                    "MAVLink HEARTBEAT was not detected."
                )
            if not printed_mav_fallback:
                print(
                    "No MAVLink HEARTBEAT detected yet. "
                    f"AirSim RPC on {host}:{airsim_port} is up; continuing to probe MAVLink "
                    f"for up to {timeout_s:.0f}s total. "
                    "For PX4Multirotor, start PX4-SITL (uv run sim-mavlink) or set "
                    "control.transport to 'auto' or 'airsim' for SimpleFlight."
                )
                printed_mav_fallback = True

        time.sleep(0.25)

    if strict:
        raise SystemExit(
            f"AIGP_MAVLINK_STRICT=1: no MAVLink HEARTBEAT within {timeout_s:.0f}s. "
            "Check MAVLink wiring in the simulator."
        )
    raise SystemExit(
        f"Neither MAVLink HEARTBEAT nor AirSim RPC became ready within {timeout_s:.0f}s. "
        "If using PX4Multirotor, start PX4-SITL (see: uv run sim-mavlink). "
        "For SimpleFlight without PX4, set control.transport to 'airsim' in sim.config.json."
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
    def _valid(path: str) -> bool:
        return bool(path) and Path(path).is_file()

    project = os.environ.get("PROJECT_PATH", "").strip()
    if project and not _valid(project):
        print(
            f"Warning: PROJECT_PATH points to missing file: {project!r}. "
            "Attempting auto-repair...",
            file=sys.stderr,
        )
        project = ""

    if _valid(project):
        return project

    config_project = str(sim_cfg.get("project_path", "")).strip()
    if _valid(config_project):
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
        if _valid(project):
            return project

    return ""


def _ensure_px4_mavlink_settings(
    airsim_port: int,
    *,
    config: dict | None = None,
    view_mode: str = "Fpv",
    enable_trace: bool = False,
    corner_chase_pip: bool = False,
) -> Path:
    """Backward-compatible wrapper around ``ensure_px4_hil_settings``."""
    return ensure_px4_hil_settings(
        airsim_port,
        config if config is not None else {},
        viewport=ViewportOptions(
            view_mode=view_mode,
            corner_chase_pip=corner_chase_pip,
            enable_trace=enable_trace,
        ),
    )


def _wait_for_hil_tcp_listener(timeout_s: float) -> bool:
    deadline = time.time() + max(5.0, float(timeout_s))
    next_status_s = time.time() + 15.0
    while time.time() < deadline:
        if _is_tcp_listening_passive(PX4_HIL_TCP_PORT):
            print(f"[launcher] AirSim HIL TCP :{PX4_HIL_TCP_PORT} is listening.")
            return True
        if time.time() >= next_status_s:
            remaining = max(0.0, deadline - time.time())
            print(
                f"[launcher] Waiting for AirSim HIL TCP :{PX4_HIL_TCP_PORT} "
                f"({remaining:.0f}s left)..."
            )
            next_status_s = time.time() + 15.0
        time.sleep(1.0)
    return False


def _stop_px4_wsl_process() -> None:
    if _handles.px4 is not None and _handles.px4.poll() is None:
        try:
            _handles.px4.terminate()
            _handles.px4.wait(timeout=5.0)
        except (OSError, subprocess.TimeoutExpired):
            try:
                _handles.px4.kill()
            except OSError:
                pass
        _handles.px4 = None
    from src.mavlink_prereq import stop_stale_px4_wsl

    stop_stale_px4_wsl({})


def _start_px4_wsl(config: dict) -> subprocess.Popen[str] | None:
    from src.mavlink_prereq import launch_px4_script_path, wsl_cmd_prefix, wsl_run_checked

    script = launch_px4_script_path()
    if not script.is_file():
        print(f"[launcher] PX4 launch script missing: {script}", file=sys.stderr)
        return None
    try:
        wslpath = wsl_run_checked(
            [*wsl_cmd_prefix(config), "-e", "wslpath", "-u", str(script)],
            timeout_s=15.0,
        )
        script_wsl = wslpath.stdout.strip()
        if wslpath.returncode != 0 or not script_wsl:
            print(
                "[launcher] Could not resolve WSL path for launch_px4_wsl.sh. "
                "Is WSL installed?",
                file=sys.stderr,
            )
            return None
        cmd = [*wsl_cmd_prefix(config), "-e", "bash", script_wsl]
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        _handles.px4 = proc
        return proc
    except (OSError, subprocess.SubprocessError) as exc:
        print(f"[launcher] Failed to start PX4 in WSL: {exc}", file=sys.stderr)
        return None


def _wait_for_px4_airsim_connected(proc: subprocess.Popen[str], timeout_s: float) -> bool:
    connected = threading.Event()
    conn_line: list[str] = []

    def _reader() -> None:
        if proc.stdout is None:
            return
        try:
            for raw in proc.stdout:
                line = raw.rstrip()
                if line:
                    print(f"[launcher] [px4] {line}")
                if "Simulator connected on TCP port" in raw:
                    conn_line.append(line)
                    connected.set()
                    return
        except OSError:
            pass

    thread = threading.Thread(target=_reader, name="px4_wsl_stdout", daemon=True)
    thread.start()
    if connected.wait(timeout=max(5.0, float(timeout_s))):
        return True
    if proc.poll() is not None:
        print(
            f"[launcher] PX4-SITL exited early (code {proc.returncode}).",
            file=sys.stderr,
        )
    else:
        print(
            "[launcher] Timed out waiting for PX4 to connect to AirSim on TCP :4560.",
            file=sys.stderr,
        )
    return False


def _bootstrap_px4_wsl_for_mavlink(config: dict) -> None:
    """Start PX4-SITL in WSL after UE HIL listener is up (Windows one-command flow)."""
    from src.mavlink_prereq import PX4_BUILD_HINT, wsl_available

    if os.environ.get("AIGP_SKIP_PX4_AUTOSTART", "").strip() == "1":
        print("[launcher] AIGP_SKIP_PX4_AUTOSTART=1 — start PX4-SITL manually in WSL.")
        return
    mav_cfg = config.get("control", {}).get("mavlink", {})
    px4_cfg = mav_cfg.get("px4_wsl", {})
    if not bool(px4_cfg.get("auto_start", True)):
        return
    if sys.platform != "win32":
        print(
            "[launcher] PX4 WSL autostart is Windows-only; start PX4-SITL manually, then re-run.",
            file=sys.stderr,
        )
        return
    from src.mavlink_endpoints import first_mavlink_heartbeat_endpoint

    if first_mavlink_heartbeat_endpoint(config, timeout_s=1.0) is not None:
        print("[launcher] MAVLink heartbeat already present — skipping PX4 autostart.")
        return
    ok, detail = wsl_available(config)
    if not ok:
        raise SystemExit(
            f"[launcher] WSL is required for PX4 autostart but is unavailable: {detail}"
        )
    from src.mavlink_prereq import px4_wsl_binary_ready

    if not px4_wsl_binary_ready(config):
        raise SystemExit(
            "[launcher] PX4 binary not found in WSL at "
            "~/PX4-Autopilot/build/px4_sitl_default/bin/px4.\n"
            f"Build once: {PX4_BUILD_HINT}"
        )
    connect_timeout_s = max(30.0, float(px4_cfg.get("connect_timeout_seconds", 90.0)))
    print("[launcher] Starting PX4-SITL in WSL (PX4_SIM_HOST_ADDR=127.0.0.1)...")
    proc = _start_px4_wsl(config)
    if proc is None:
        raise SystemExit("[launcher] Failed to spawn PX4-SITL in WSL.")
    if not _wait_for_px4_airsim_connected(proc, connect_timeout_s):
        raise SystemExit(
            "[launcher] PX4 did not connect to AirSim. Ensure WSL mirrored networking "
            "(see scripts/dev-mavlink.ps1 -EnableMirrored) and that UE HIL :4560 is up."
        )
    print("[launcher] PX4 connected to AirSim; waiting for MAVLink HEARTBEAT on UDP 14550...")


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


def _maybe_restore_simpleflight_from_backup(config: dict | None = None) -> bool:
    # Self-heal for users who ran sim-mavlink/mavlink-all and skipped Ctrl+C
    # cleanup (Task Manager kill, BSOD). Only restores when the current file
    # still looks like PX4Multirotor, so a freshly-edited SimpleFlight config
    # is never clobbered. Skip when MAVLink / position-trace HUD is requested.
    from src.mavlink_prereq import wants_mavlink_session

    if config is not None and wants_mavlink_session(config):
        return False
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
) -> None:
    _register_signal_handlers_once()
    _handles.ue = None
    _handles.main = None
    _handles.cleanup_done = False

    _load_env_local()

    from src.config import load_config
    from src.mavlink_endpoints import first_mavlink_heartbeat_endpoint
    from src.simulator_specs import assert_specification_snapshot_if_required

    config = load_config()
    assert_specification_snapshot_if_required(config)
    sim_cfg = config["simulator"]
    restored_simpleflight = _maybe_restore_simpleflight_from_backup(config)
    transport = _launch_transport(config, restored_simpleflight=restored_simpleflight)
    resolved_transport = transport
    resolved_mavlink_endpoint: str | None = None
    if transport == "mavlink":
        from src.mavlink_prereq import ensure_mavlink_prerequisites

        ensure_mavlink_prerequisites(config)
        resolved_mavlink_endpoint = first_mavlink_heartbeat_endpoint(config, timeout_s=2.0)
    hil_already_up = transport == "mavlink" and _is_tcp_listening_passive(PX4_HIL_TCP_PORT)

    colosseum = sim_cfg.get("colosseum_path", "")
    project = _resolve_project_path(sim_cfg)
    if project:
        print(f"Using PROJECT_PATH={project}")
    if transport == "mavlink":
        print(
            "[launcher] MAVLink one-command flow: UE + auto PX4-SITL (WSL) + main.py. "
            "Position trace/HUD when enabled. Do not run 'uv run sim-mavlink' in parallel."
        )
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
    ensure_launch_settings(
        transport,
        airsim_port,
        config,
        view_mode=view_mode,
        corner_chase_pip=corner_chase_pip,
        enable_trace=enable_trace,
    )
    if transport != "mavlink":
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
    elif hil_already_up:
        print(
            "[launcher] AirSim HIL TCP :4560 already listening — skipping UE launch to avoid "
            "a second Colosseum instance. Stop 'uv run sim-mavlink' if it is still running, "
            "then use this 'uv run sim' session only."
        )
        if transport == "mavlink" and resolved_mavlink_endpoint is None:
            _bootstrap_px4_wsl_for_mavlink(config)
        wait_label = "MAVLink/AirSim control link"
        print(f"Waiting for {wait_label} on {host}:{airsim_port} (timeout {rpc_tout_label}s)...")
        try:
            resolved_transport, resolved_mavlink_endpoint = _wait_for_control_link(
                host,
                airsim_port,
                config,
                rpc_ready_timeout_s,
                transport,
            )
        except KeyboardInterrupt:
            _cleanup_on_interrupt()
            raise SystemExit(130) from None
        if resolved_transport == "mavlink" and resolved_mavlink_endpoint is not None:
            print(f"MAVLink heartbeat detected on {resolved_mavlink_endpoint}")
        else:
            print(f"AirSim RPC is ready on {host}:{airsim_port}")
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
        physics_hz = float(sim_cfg.get("physics_update_hz", 0.0) or 0.0)
        print(
            f"[launcher] UE physics timestep is project-defined ({physics_hz:.0f} Hz in config); "
            "not set via AirSim settings.json. Run: uv run preflight"
        )
        if transport == "mavlink":
            hil_timeout_s = max(
                30.0,
                float(
                    config.get("control", {})
                    .get("mavlink", {})
                    .get("px4_wsl", {})
                    .get("hil_ready_timeout_seconds", 120.0)
                ),
            )
            if not _wait_for_hil_tcp_listener(hil_timeout_s):
                raise SystemExit(
                    f"[launcher] AirSim HIL TCP :{PX4_HIL_TCP_PORT} did not start within "
                    f"{hil_timeout_s:.0f}s. Check UE/AirSim logs."
                )
            if resolved_mavlink_endpoint is None:
                _bootstrap_px4_wsl_for_mavlink(config)
        wait_label = "MAVLink/AirSim control link" if transport == "mavlink" else "AirSim RPC"
        print(f"Waiting for {wait_label} on {host}:{airsim_port} (timeout {rpc_tout_label}s)...")
        try:
            resolved_transport, resolved_mavlink_endpoint = _wait_for_control_link(
                host,
                airsim_port,
                config,
                rpc_ready_timeout_s,
                transport,
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
        if transport == "mavlink" and resolved_mavlink_endpoint is None:
            _bootstrap_px4_wsl_for_mavlink(config)
        wait_label = "MAVLink/AirSim control link" if transport == "mavlink" else "AirSim RPC"
        print(f"Waiting for {wait_label} on {host}:{airsim_port} (timeout {rpc_tout_label}s)...")
        try:
            resolved_transport, resolved_mavlink_endpoint = _wait_for_control_link(
                host,
                airsim_port,
                config,
                rpc_ready_timeout_s,
                transport,
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


def main_attitude_smoke() -> None:
    launch(script_path="src/attitude_smoke.py")


def main_highres_imu_smoke() -> None:
    os.environ["AIGP_CONTROL_TRANSPORT"] = "mavlink"
    os.environ.setdefault("AIGP_ALLOW_MAVLINK_SIMPLEFLIGHT", "1")
    launch(
        script_path="src/highres_imu_smoke.py",
        require_requested_transport=True,
    )


if __name__ == "__main__":
    launch()
