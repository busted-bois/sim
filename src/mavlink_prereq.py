"""MAVLink / PX4 / WSL prerequisite checks and safe auto-setup for uv run sim."""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
WSLCONFIG_TEMPLATE = "[wsl2]\nnetworkingMode=mirrored\n"
PX4_BUILD_HINT = (
    "wsl -e bash -c 'cd ~/PX4-Autopilot && make px4_sitl none_iris'"
)


def wants_mavlink_session(config: dict[str, Any]) -> bool:
    """True when config/env requests MAVLink or position-trace HUD features."""
    control = config.get("control", {})
    env_transport = os.environ.get("AIGP_CONTROL_TRANSPORT", "").strip().lower()
    raw = env_transport or str(control.get("transport", "airsim")).strip().lower()
    if raw in {"mavlink", "auto"}:
        return True
    mav = control.get("mavlink", {})
    trace = mav.get("position_trace", {})
    hud = mav.get("position_hud", {})
    return bool(trace.get("enabled")) or bool(hud.get("enabled"))


def _preflight_mavlink_flags(config: dict[str, Any]) -> dict[str, bool]:
    pre = config.get("preflight", {})
    mav = pre.get("mavlink", {})
    return {
        "run_before_sim": bool(pre.get("run_before_sim", True)),
        "ensure_wsl": bool(mav.get("ensure_wsl", True)),
        "ensure_px4_binary": bool(mav.get("ensure_px4_binary", True)),
        "ensure_wsl_mirrored": bool(mav.get("ensure_wsl_mirrored", True)),
        "stop_stale_px4": bool(mav.get("stop_stale_px4", True)),
    }


def wsl_config_path() -> Path:
    return Path.home() / ".wslconfig"


def wsl_mirrored_mode_enabled() -> bool:
    cfg = wsl_config_path()
    if not cfg.is_file():
        return False
    try:
        text = cfg.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    return bool(re.search(r"(?im)^\s*networkingMode\s*=\s*mirrored\s*$", text))


def ensure_wsl_mirrored_config() -> tuple[bool, str | None]:
    """Write .wslconfig with mirrored networking if missing. Returns (wrote_new, note)."""
    cfg = wsl_config_path()
    if wsl_mirrored_mode_enabled():
        return False, None
    if cfg.is_file():
        return False, (
            f"WSL mirrored networking not enabled in {cfg}. "
            "Add under [wsl2]: networkingMode=mirrored then run: wsl --shutdown"
        )
    try:
        cfg.write_text(WSLCONFIG_TEMPLATE, encoding="ascii")
    except OSError as exc:
        return False, f"Could not write {cfg}: {exc}"
    return True, (
        f"Wrote {cfg} with networkingMode=mirrored. "
        "Run once: wsl --shutdown — then re-run uv run sim."
    )


def wsl_cmd_prefix(config: dict[str, Any]) -> list[str]:
    mav_cfg = config.get("control", {}).get("mavlink", {})
    px4_cfg = mav_cfg.get("px4_wsl", {})
    distro = os.environ.get("AIGP_WSL_DISTRO", "").strip() or str(
        px4_cfg.get("wsl_distro", "")
    ).strip()
    cmd = ["wsl"]
    if distro:
        cmd.extend(["-d", distro])
    return cmd


def wsl_run_checked(
    cmd: list[str],
    *,
    timeout_s: float = 15.0,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout_s,
        check=False,
    )


def wsl_available(config: dict[str, Any]) -> tuple[bool, str]:
    try:
        out = wsl_run_checked([*wsl_cmd_prefix(config), "-e", "true"], timeout_s=10.0)
        if out.returncode == 0:
            return True, ""
        return False, out.stderr.strip() or "wsl -e true failed"
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)


def px4_wsl_binary_ready(config: dict[str, Any]) -> bool:
    if sys.platform != "win32":
        return False
    try:
        px4_test = (
            "test -x ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo OK"
        )
        out = wsl_run_checked(
            [*wsl_cmd_prefix(config), "-e", "bash", "-c", px4_test],
            timeout_s=20.0,
        )
        return out.stdout.strip() == "OK"
    except (OSError, subprocess.SubprocessError):
        return False


def launch_px4_script_path() -> Path:
    return ROOT / "scripts" / "launch_px4_wsl.sh"


def stop_stale_px4_wsl(config: dict[str, Any]) -> None:
    if sys.platform != "win32":
        return
    stop_script = ROOT / "scripts" / "stop_px4_wsl.sh"
    if not stop_script.is_file():
        try:
            subprocess.run(
                [
                    *wsl_cmd_prefix(config),
                    "-e",
                    "bash",
                    "-c",
                    "pkill -INT -f 'px4_sitl_default/bin/px4' 2>/dev/null; "
                    "pkill -f 'make px4_sitl' 2>/dev/null; true",
                ],
                capture_output=True,
                timeout=10.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            pass
        return
    try:
        wslpath = wsl_run_checked(
            [*wsl_cmd_prefix(config), "-e", "wslpath", "-u", str(stop_script)],
            timeout_s=15.0,
        )
        script_wsl = wslpath.stdout.strip()
        if wslpath.returncode == 0 and script_wsl:
            wsl_run_checked(
                [*wsl_cmd_prefix(config), "-e", "bash", script_wsl],
                timeout_s=15.0,
            )
    except (OSError, subprocess.SubprocessError):
        pass


def hud_trace_config_warnings(config: dict[str, Any]) -> list[str]:
    mav = config.get("control", {}).get("mavlink", {})
    hud = mav.get("position_hud", {})
    trace = mav.get("position_trace", {})
    warnings: list[str] = []
    if bool(hud.get("enabled")) and str(hud.get("data_source", "trace")).strip() == "trace":
        if not bool(trace.get("enabled")):
            warnings.append(
                "position_hud.data_source=trace but position_trace.enabled is false; "
                "HUD will stay idle unless trace store is created another way."
            )
    return warnings


def run_static_mavlink_checks(
    config: dict[str, Any],
) -> tuple[list[str], list[str], list[str]]:
    """Static MAVLink prereqs for preflight (no live UDP). Returns errors, warnings, passes."""
    errors: list[str] = []
    warnings: list[str] = []
    passes: list[str] = []
    flags = _preflight_mavlink_flags(config)

    script = launch_px4_script_path()
    if script.is_file():
        passes.append(f"PX4 launch script present: {script.name}")
    else:
        errors.append(f"Missing {script}")

    if sys.platform != "win32":
        warnings.append("MAVLink WSL/PX4 static checks skipped (not Windows)")
        warnings.extend(hud_trace_config_warnings(config))
        return errors, warnings, passes

    if flags["ensure_wsl"]:
        ok, detail = wsl_available(config)
        if ok:
            passes.append("WSL is available")
        else:
            errors.append(f"WSL not available: {detail}")

    if flags["ensure_px4_binary"]:
        if px4_wsl_binary_ready(config):
            passes.append("PX4 SITL binary found in WSL")
        else:
            errors.append(
                "PX4 binary not found at ~/PX4-Autopilot/build/px4_sitl_default/bin/px4. "
                f"Build once: {PX4_BUILD_HINT}"
            )

    if flags["ensure_wsl_mirrored"]:
        if wsl_mirrored_mode_enabled():
            passes.append("WSL mirrored networking enabled in .wslconfig")
        else:
            wrote, note = ensure_wsl_mirrored_config()
            if wrote and note:
                warnings.append(note)
            elif note:
                errors.append(note)
            else:
                errors.append("WSL mirrored networking is not enabled in .wslconfig")

    warnings.extend(hud_trace_config_warnings(config))
    return errors, warnings, passes


def ensure_mavlink_prerequisites(config: dict[str, Any]) -> None:
    """Run before sim launch on Windows; raises SystemExit on hard failure."""
    if os.environ.get("AIGP_SKIP_MAVLINK_PREREQ", "").strip() == "1":
        print("[launcher] AIGP_SKIP_MAVLINK_PREREQ=1 — skipping MAVLink prerequisite checks.")
        return

    flags = _preflight_mavlink_flags(config)
    if not flags["run_before_sim"]:
        return

    if sys.platform != "win32":
        return

    errors, warnings, _passes = run_static_mavlink_checks(config)
    for line in warnings:
        print(f"[launcher] [WARN] {line}")
    if errors:
        for line in errors:
            print(f"[launcher] [FAIL] {line}", file=sys.stderr)
        raise SystemExit(
            "[launcher] MAVLink prerequisites failed. Fix the items above, "
            "or run: uv run prerun"
        )

    if flags["stop_stale_px4"]:
        stop_stale_px4_wsl(config)

    print("[launcher] MAVLink prerequisites OK (WSL, PX4 binary, networking).")
