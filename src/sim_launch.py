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
    required_settings = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ViewMode": normalized_view_mode,
        "ApiServerPort": int(airsim_port),
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

    config = load_config()
    sim_cfg = config["simulator"]

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
    _ensure_camera_settings(
        airsim_port,
        view_mode,
        corner_chase_pip=corner_chase_pip,
        enable_trace=enable_trace,
        use_vjoy=use_vjoy,
    )

    if colosseum and Path(colosseum).exists():
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
        print(f"Waiting for AirSim RPC on {host}:{airsim_port} (timeout {rpc_tout_label}s)...")
        try:
            if not _wait_for_airsim_rpc(host, airsim_port, rpc_ready_timeout_s):
                raise SystemExit(
                    f"AirSim RPC did not become ready on {host}:{airsim_port} "
                    f"within {rpc_ready_timeout_s:.0f}s. Ensure Unreal finished loading the map."
                )
        except KeyboardInterrupt:
            _cleanup_on_interrupt()
            raise SystemExit(130) from None
        print(f"AirSim RPC is ready on {host}:{airsim_port}")
    else:
        print(f"Colosseum not found at '{colosseum}', skipping simulator launch.")
        print(f"Waiting for AirSim RPC on {host}:{airsim_port} (timeout {rpc_tout_label}s)...")
        try:
            if not _wait_for_airsim_rpc(host, airsim_port, rpc_ready_timeout_s):
                raise SystemExit(
                    f"AirSim RPC did not become ready on {host}:{airsim_port} "
                    f"within {rpc_ready_timeout_s:.0f}s. Start the simulator, then try again."
                )
        except KeyboardInterrupt:
            _cleanup_on_interrupt()
            raise SystemExit(130) from None
        print(f"AirSim RPC is ready on {host}:{airsim_port}")

    env = os.environ.copy()
    env["AIRSIM_PORT"] = str(airsim_port)
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


if __name__ == "__main__":
    launch()
