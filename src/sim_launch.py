"""Start Unreal/AirSim (Colosseum or maze UE 4.16 project), wait, then run main.py."""

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
MAZE_MAP_ARG = "-map=/Game/MazeCreator/Maps/MapMaze"
MAZE_MAP_FALLBACK_ARG = "-map=/Game/MazeCreator/Maps/Map"

# UE4.x Epic builds often ship UE4Editor.exe; newer layouts use UnrealEditor.exe.
_MAZE_WIN64_EDITOR_NAMES = ("UnrealEditor.exe", "UE4Editor.exe")


def _resolve_win64_editor_exe(install_root: Path) -> str:
    """Return path to editor exe under ``install_root`` (folder that contains ``Engine/``)."""
    win64 = install_root / "Engine" / "Binaries" / "Win64"
    for name in _MAZE_WIN64_EDITOR_NAMES:
        candidate = win64 / name
        if candidate.is_file():
            return str(candidate.resolve())
    return ""


def _resolve_configured_maze_editor(raw: str) -> str:
    """Resolve env/config path to an editor executable.

    Accepts:
    - Full path to UnrealEditor.exe / UE4Editor.exe
    - Win64 folder path
    - Engine root path (folder containing Engine/)
    """
    text = os.path.expandvars(raw.strip())
    if not text:
        return ""
    p = Path(text).expanduser()
    if p.is_file():
        return str(p.resolve())
    if p.is_dir():
        # Common user input: engine root folder (e.g. C:\Program Files\Epic Games\UE_4.16).
        found = _resolve_win64_editor_exe(p)
        if found:
            return found
        # Alternate input: direct Win64 folder.
        for name in _MAZE_WIN64_EDITOR_NAMES:
            candidate = p / name
            if candidate.is_file():
                return str(candidate.resolve())
    if p.suffix.lower() == ".exe":
        parent = p.parent
        for name in _MAZE_WIN64_EDITOR_NAMES:
            alt = parent / name
            if alt.is_file():
                return str(alt.resolve())
    return ""


def _resolve_maze_map_launch_arg(project_path: str) -> str | None:
    """Pick the best available maze map argument for the active Unreal project.

    Returns a valid -map launch arg when a known maze map exists, otherwise None.
    """
    project_file = Path(project_path)
    project_root = project_file.parent
    map_maze_path = project_root / "Content" / "MazeCreator" / "Maps" / "MapMaze.umap"
    map_fallback_path = project_root / "Content" / "MazeCreator" / "Maps" / "Map.umap"
    print(f"Maze mode: inspecting Unreal project folder: {project_root}")
    print(f"Maze mode: expecting map file: {map_maze_path}")
    print(f"Maze mode: fallback map file: {map_fallback_path}")
    candidates: list[tuple[Path, str]] = [
        (map_maze_path, MAZE_MAP_ARG),
        (map_fallback_path, MAZE_MAP_FALLBACK_ARG),
    ]
    for umap_path, map_arg in candidates:
        if umap_path.is_file():
            return map_arg
    return None


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


def _write_legacy_airsim_settings(
    settings_path: Path, airsim_port: int, view_mode: str
) -> None:
    """Minimal settings.json for AirSim 1.1.x (UE 4.16 maze). Avoids 1.2-only keys that can
    prevent the RPC server from starting when merged into older plugins.
    """
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    normalized = _normalize_view_mode(view_mode)
    minimal: dict = {
        "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
        "SettingsVersion": 1.1,
        "SimMode": "Multirotor",
        "ViewMode": normalized,
        "ClockSpeed": 1.0,
        "ApiServerPort": int(airsim_port),
        # Gives getSettingsString a Vehicles{} block when listVehicles is missing (many UE4 forks).
        "Vehicles": {
            "SimpleFlight": {
                "VehicleType": "SimpleFlight",
            }
        },
    }
    settings_path.write_text(json.dumps(minimal, indent=2), encoding="utf-8")
    print(
        f"Configured legacy AirSim settings (SettingsVersion=1.1, ApiServerPort={airsim_port}) "
        f"in {settings_path}"
    )


def _ensure_camera_settings(
    airsim_port: int,
    view_mode: str,
    *,
    corner_chase_pip: bool,
    enable_trace: bool,
    legacy_airsim: bool = False,
) -> None:
    settings_path = _airsim_settings_path()
    if legacy_airsim:
        _write_legacy_airsim_settings(settings_path, airsim_port, view_mode)
        return

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
    if enable_trace:
        # Enable AirSim trace for third-person mode.
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


def _wait_for_airsim_rpc(
    host: str,
    port: int,
    timeout_s: float,
    *,
    relaxed_vehicle_ready: bool = False,
) -> bool:
    """Wait until AirSim is answering RPCs.

    Prefer ``ping()`` plus ``getMultirotorState()`` so the multirotor exists. On older
    AirSim (e.g. 1.1.x with UE 4.16), ``getMultirotorState`` may fail briefly even when the
    API is up; ``relaxed_vehicle_ready`` treats a successful ``ping`` as ready.
    """
    import airsim as _airsim  # local import: keep module load cheap

    _maze_probe_veh = (
        os.environ.get("AIGP_AIRSIM_VEHICLE_NAME", "").strip() or "SimpleFlight"
    )

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
            if not probe_client.ping():
                continue
            try:
                if os.environ.get("AIGP_MAZE", "").strip() == "1" or os.environ.get(
                    "AIGP_AIRSIM_LEGACY_RPC", ""
                ).strip() == "1":
                    raw = probe_client.client
                    while hasattr(raw, "_inner"):
                        raw = raw._inner
                    raw.call("getMultirotorState", _maze_probe_veh)
                else:
                    probe_client.getMultirotorState()
            except Exception as exc:
                if relaxed_vehicle_ready:
                    print(
                        "AirSim RPC: ping OK; getMultirotorState failed (treating as ready for "
                        f"maze/legacy sim): {type(exc).__name__}: {exc}"
                    )
                    return True
                continue
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


def _default_maze_uproject() -> Path:
    return ROOT / "opensrc" / "AirsimSimulation" / "VehilceAdvanced.uproject"


def _try_maze_uproject(raw: str) -> Path | None:
    """Resolve a maze .uproject path: absolute file, or path relative to repo ROOT."""
    text = os.path.expandvars(raw.strip())
    if not text:
        return None
    p = Path(text).expanduser()
    if p.is_file():
        return p.resolve()
    if not p.is_absolute():
        cand = (ROOT / text).resolve()
        if cand.is_file():
            return cand
    return None


def _resolve_maze_uproject_path(sim_cfg: dict) -> str:
    """Resolve maze .uproject with env/config/default fallbacks.

    Priority:
    1) MAZE_PROJECT_PATH
    2) simulator.maze_project_path
    3) PROJECT_PATH (for teams reusing .env.local across modes)
    4) opensrc/AirsimSimulation/VehilceAdvanced.uproject
    """
    for raw in (
        os.environ.get("MAZE_PROJECT_PATH", "").strip(),
        str(sim_cfg.get("maze_project_path", "")).strip(),
        os.environ.get("PROJECT_PATH", "").strip(),
    ):
        got = _try_maze_uproject(raw) if raw else None
        if got is not None:
            return str(got)
    default_maze = _default_maze_uproject()
    return str(default_maze.resolve()) if default_maze.is_file() else ""


def _guess_ue_416_from_launcher_dat() -> str:
    """Parse Epic LauncherInstalled.dat (Windows) for a UE 4.16 engine install."""
    program_data = os.environ.get("PROGRAMDATA", "").strip()
    if not program_data:
        return ""
    dat = Path(program_data) / "Epic" / "UnrealEngineLauncher" / "LauncherInstalled.dat"
    if not dat.is_file():
        return ""
    try:
        payload = json.loads(dat.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return ""
    items = payload.get("InstallationList")
    if not isinstance(items, list):
        return ""
    for item in items:
        if not isinstance(item, dict):
            continue
        loc = str(item.get("InstallLocation", "")).strip().rstrip("/\\")
        if not loc:
            continue
        hay = f"{loc.upper()} {str(item.get('AppName', '')).upper()}"
        if "4.16" not in hay and "UE_4.16" not in hay:
            continue
        found = _resolve_win64_editor_exe(Path(loc))
        if found:
            return found
    return ""


def _guess_ue_416_editor() -> str:
    """Best-effort path to UE 4.16 UnrealEditor.exe (Epic Launcher + common folders)."""
    found = _guess_ue_416_from_launcher_dat()
    if found:
        return found

    roots: list[Path] = []
    epic = os.environ.get("EPIC_GAMES_PATH", "").strip()
    if epic:
        roots.append(Path(epic))
    roots.extend(
        [
            Path(r"C:\Program Files\Epic Games"),
            Path(r"C:\Program Files (x86)\Epic Games"),
        ]
    )
    for letter in "DEFGHIJKLMNOPQRSTUVWXYZ":
        extra = Path(f"{letter}:\\Program Files\\Epic Games")
        if extra.is_dir():
            roots.append(extra)
    seen: set[Path] = set()
    for root in roots:
        root = root.resolve() if root.exists() else root
        if root in seen or not root.is_dir():
            continue
        seen.add(root)
        exact_root = root / "UE_4.16"
        found = _resolve_win64_editor_exe(exact_root)
        if found:
            return found
        try:
            for child in sorted(root.iterdir(), key=lambda p: p.name.lower()):
                if not child.is_dir():
                    continue
                name_u = child.name.upper()
                if "4.16" not in name_u and name_u != "UE_4.16":
                    continue
                found = _resolve_win64_editor_exe(child)
                if found:
                    return found
        except OSError:
            continue
    return ""


def _resolve_ue_and_project(sim_cfg: dict, *, maze_mode: bool) -> tuple[str, str]:
    """Return (unreal_editor_exe, uproject_path) for this run.

    Normal mode: ``colosseum_path`` + ``PROJECT_PATH`` / ``project_path``.

    Maze mode (``uv run sim maze``): always UE **4.16** editor + AirsimSimulation
    ``VehilceAdvanced.uproject`` only — no fallback to UE 5.4 or BlocksV2.
    """
    colosseum = str(sim_cfg.get("colosseum_path", "")).strip()
    default_project = _resolve_project_path(sim_cfg)

    if not maze_mode:
        return colosseum, default_project

    maze_editor = (
        os.environ.get("MAZE_COLLOSSEUM_PATH", "").strip()
        or str(sim_cfg.get("maze_colosseum_path", "")).strip()
    )
    ue_editor = _resolve_configured_maze_editor(maze_editor)
    if not ue_editor:
        guessed = _guess_ue_416_editor()
        if guessed:
            print(f"Maze mode: auto-detected UE 4.16 editor: {guessed}")
            ue_editor = guessed
    if not ue_editor:
        raise SystemExit(
            "uv run sim maze could not find a UE 4.16 editor executable "
            "(UnrealEditor.exe or UE4Editor.exe under Engine\\Binaries\\Win64).\n"
            "If Epic Launcher lists UE 4.16 but this fails, the engine may be incomplete: "
            "Library -> UE 4.16 -> ... -> Verify or Resume installation.\n"
            "Or set simulator.maze_colosseum_path or MAZE_COLLOSSEUM_PATH to the full path "
            "to UnrealEditor.exe or UE4Editor.exe.\n"
            "Searched: Epic LauncherInstalled.dat "
            "(%ProgramData%\\Epic\\UnrealEngineLauncher\\), EPIC_GAMES_PATH, "
            "Program Files\\Epic Games (extra drives), UE_4.16 / folder names containing 4.16.\n"
            "This command does not fall back to simulator.colosseum_path (UE 5.x)."
        )

    uproject = _resolve_maze_uproject_path(sim_cfg)
    if not uproject:
        raise SystemExit(
            "uv run sim maze could not find VehilceAdvanced.uproject.\n"
            "Run:\n"
            '  powershell -ExecutionPolicy Bypass -File ".\\scripts\\extract_airsim_maps.ps1"\n'
            "Or set MAZE_PROJECT_PATH / simulator.maze_project_path / PROJECT_PATH "
            "(e.g. in .env.local), as an absolute path or a path relative to this repo, "
            "such as opensrc/AirsimSimulation/VehilceAdvanced.uproject.\n"
            f"Default location checked: {_default_maze_uproject()}"
        )

    return ue_editor, uproject


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
    ue_args_override: list[str] | None = None,
    maze_mode: bool = False,
) -> None:
    if maze_mode:
        # Same process runs the AirSim RPC probe; shim in airsim/client.py keys off this env.
        os.environ["AIGP_MAZE"] = "1"
        os.environ["AIGP_AIRSIM_LEGACY_RPC"] = "1"
    _register_signal_handlers_once()
    _handles.ue = None
    _handles.main = None
    _handles.cleanup_done = False

    _load_env_local()
    if maze_mode:
        # .env.local can override env vars — force maze/legacy flags again for this process.
        os.environ["AIGP_MAZE"] = "1"
        os.environ["AIGP_AIRSIM_LEGACY_RPC"] = "1"

    from src.config import load_config

    config = load_config()
    sim_cfg = config["simulator"]

    ue_editor, project = _resolve_ue_and_project(sim_cfg, maze_mode=maze_mode)
    if maze_mode:
        print(f"Maze mode: UnrealEditor={ue_editor or '(none)'}")
        print(f"Maze mode: .uproject={project or '(none)'}")
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
    if maze_mode:
        maze_rpc_t = float(sim_cfg.get("maze_rpc_ready_timeout_seconds", 300.0))
        rpc_ready_timeout_s = max(rpc_ready_timeout_s, maze_rpc_t)
    rpc_tout_label = f"{rpc_ready_timeout_s:.0f}"
    _ensure_camera_settings(
        airsim_port,
        view_mode,
        corner_chase_pip=corner_chase_pip,
        enable_trace=enable_trace,
        legacy_airsim=maze_mode,
    )

    if ue_editor and Path(ue_editor).exists():
        if not project:
            if maze_mode:
                raise SystemExit(
                    "Maze mode: internal error — maze .uproject path was empty after resolution."
                )
            raise SystemExit(
                "PROJECT_PATH is not set to a valid .uproject file.\n"
                "Fix it by either:\n"
                "  1) setting PROJECT_PATH in .env.local, or\n"
                "  2) adding simulator.project_path in sim.config.json, or\n"
                "  3) running: pwsh scripts/fix_project_path.ps1\n"
            )
        if low_end:
            print(f"Launching Unreal (low-end): {ue_editor}")
        else:
            print(f"Launching Unreal: {ue_editor}")
        cmd = [ue_editor]
        cmd.append(project)
        cmd.append("-game")
        cmd.append(f"-settings={_airsim_settings_path()}")
        if windowed:
            cmd.extend(["-windowed", f"-resx={res_x}", f"-resy={res_y}"])
        if ue_args_override is not None:
            extra_list = [str(arg).strip() for arg in ue_args_override if str(arg).strip()]
            if MAZE_MAP_ARG in extra_list:
                resolved_maze_arg = _resolve_maze_map_launch_arg(project)
                if resolved_maze_arg is None:
                    print(
                        "Maze mode requested, but no maze map was found in the active Unreal "
                        "project (expected Content/MazeCreator/Maps/MapMaze.umap or Map.umap). "
                        "Launching without a forced maze map."
                    )
                    extra_list = [arg for arg in extra_list if arg != MAZE_MAP_ARG]
                elif resolved_maze_arg != MAZE_MAP_ARG:
                    print(
                        "Maze mode: MapMaze.umap not found; falling back to "
                        "/Game/MazeCreator/Maps/Map."
                    )
                    extra_list = [
                        resolved_maze_arg if arg == MAZE_MAP_ARG else arg for arg in extra_list
                    ]
            extra_str = ""
        else:
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
            if not _wait_for_airsim_rpc(
                host,
                airsim_port,
                rpc_ready_timeout_s,
                relaxed_vehicle_ready=maze_mode,
            ):
                raise SystemExit(
                    f"AirSim RPC did not become ready on {host}:{airsim_port} "
                    f"within {rpc_ready_timeout_s:.0f}s. Ensure Unreal finished loading the map."
                )
        except KeyboardInterrupt:
            _cleanup_on_interrupt()
            raise SystemExit(130) from None
        print(f"AirSim RPC is ready on {host}:{airsim_port}")
    else:
        print(f"Unreal editor not found at '{ue_editor}', skipping simulator launch.")
        print(f"Waiting for AirSim RPC on {host}:{airsim_port} (timeout {rpc_tout_label}s)...")
        try:
            if not _wait_for_airsim_rpc(
                host,
                airsim_port,
                rpc_ready_timeout_s,
                relaxed_vehicle_ready=maze_mode,
            ):
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
    if maze_mode:
        env["AIGP_MAZE"] = "1"
        env["AIGP_AIRSIM_LEGACY_RPC"] = "1"
    if landing_profile:
        env["AIGP_LANDING_PROFILE"] = landing_profile
    if low_end:
        env["AIGP_LOW_END"] = "1"
    if enable_trace:
        env["AIGP_ENABLE_TRACE"] = "1"

    print("Starting main.py...")
    # Do not use CREATE_NEW_PROCESS_GROUP for main.py on Windows: it correlated with the
    # child exiting almost immediately while the launcher kept running; Ctrl+C still goes
    # to the launcher first in typical Cursor/terminal setups.
    _handles.main = subprocess.Popen(
        [sys.executable, str(ROOT / "main.py")],
        env=env,
    )
    try:
        rc = _handles.main.wait()
    except KeyboardInterrupt:
        _cleanup_on_interrupt()
        raise SystemExit(130) from None
    finally:
        _handles.main = None
    if rc == 0:
        print(
            "main.py exited successfully. If this launcher started Unreal/Colosseum, "
            "that process may still be running — close it from the editor or Task Manager "
            "if needed.",
            file=sys.stderr,
        )
    else:
        print(f"main.py exited with code {rc}. See logs above.", file=sys.stderr)
    raise SystemExit(rc)


def main() -> None:
    normalized_args = {arg.strip().lower() for arg in sys.argv[1:]}
    run_maze = "maze" in normalized_args
    run_low_end = "low-end" in normalized_args
    run_third_person = "3rd-person" in normalized_args or "third-person" in normalized_args
    view_mode = "FlyWithMe" if run_third_person else "Fpv"
    ue_args_override = [MAZE_MAP_ARG] if run_maze else None
    if run_maze:
        print(f"Maze mode enabled: forcing Unreal map launch arg {MAZE_MAP_ARG}")
    launch(
        low_end=run_low_end,
        view_mode=view_mode,
        corner_chase_pip=run_third_person,
        enable_trace=run_third_person,
        ue_args_override=ue_args_override,
        maze_mode=run_maze,
    )


def main_very_soft() -> None:
    launch(landing_profile="very_soft")


def main_low_end() -> None:
    launch(low_end=True, corner_chase_pip=False)


def main_maze() -> None:
    launch(maze_mode=True, ue_args_override=[MAZE_MAP_ARG])


if __name__ == "__main__":
    launch()
