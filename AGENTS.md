# AGENTS.md

## Tooling

- **Python:** `uv` for all operations (`uv run`, `uv sync`). No bare `python3`.
- **Linter:** Ruff (`uvx ruff check --fix`). Runs via lefthook pre-commit on staged `*.{py,toml}`.
- **Ruff config:** line-length=100, rules `E F W I UP RUF`, `known-first-party = ["src"]`. See `pyproject.toml [tool.ruff]`.
- **Python version:** 3.12 (`.python-version`). `requires-python >= 3.10`.
- **CI:** `.github/workflows/ruff.yml` runs `uvx ruff check .` on PRs to main.

## Running

```bash
uv run sim                              # One command: .env.local, UE5 (if configured), main.py
uv run sim low-end                      # Low-res, reduced telemetry, attitude_four_motion
uv run sim 3rd-person                   # FlyWithMe camera instead of FPV
uv run sim-very-soft                    # Gentle landing profile
uv run sim vjoy                         # Manual vJoy control + GUI
uv run preflight                        # Safety check before launch
uv run calibrate                        # Depth calibration with manual GUI
uv run main.py                          # Run drone client (needs simulator running)
```

After Unreal starts (or if launch is skipped), the launcher **autostarts** `main.py` once AirSim RPC accepts connections on the configured host/port, up to `simulator.rpc_ready_timeout_seconds`.

**Ctrl+C** (SIGINT) while `uv run sim` is running stops the drone client (`main.py`) and, if this launcher started Unreal, terminates that editor process as well. **SIGTERM** (e.g. some IDE Stop actions) is not wired to that cleanup, so Unreal may stay open unless you stop it yourself.

### Environment Variables

Set via `.env.local` (loaded by `sim_launch.py` and `launch.sh`) or inline:
- `PROJECT_PATH` — path to `.uproject`. Required for UE5 launch.
- `AIGP_LOW_END=1` — low-end mode override.
- `AIGP_LANDING_PROFILE` — override landing profile (`very_soft`, `faster_soft`).
- `AIGP_ENABLE_TRACE=1` — enable flight path trace line.
- `AIGP_PAUSE_BEFORE_EXIT=1` — pause before client exit.
- `AIRSIM_PORT` — set by launcher, forwarded to main.py.

### Output Artifacts

- `logs/landing_telemetry.csv` — landing telemetry (when `landing.telemetry_log.enabled=true`).
- `logs/latency_tuning_recommendation.json` — autotuner results.
- `logs/vision_frames/` — debug frame dumps (when `vision.save_debug_frames=true`).

## Project Layout

- `main.py` — Entry point. Connects to AirSim RPC, loads algorithm from config, runs it.
- `sim.config.json` — Runtime config (algorithm name, sim ports, waypoints, control limits, vision, landing profiles).
- `.env.local` — `PROJECT_PATH` to UE5 project. Loaded by `uv run sim` / `launch.sh` / `launch.ps1`. Not committed.
- `src/config.py` — Reads `sim.config.json` from project root.
- `src/sim_launch.py` — Launcher script. Entry point for `uv run sim`, `uv run sim-very-soft`, `uv run sim-low-end`, `uv run calibrate`.
- `src/preflight.py` — Preflight safety check. Entry point for `uv run preflight`.
- `src/landing_telemetry.py` — Optional CSV samples during landing.
- `src/control/algorithms/` — Pluggable flight algorithms via `@register("name")` decorator.
- `src/vision/` — Vision subsystem: `feed.py` (FPV capture), `depth_perception.py` (MiDaS ONNX), `processing.py`, `frame_metrics.py`, `mapping.py`.
- `models/midas_v21_small.onnx` — Monocular depth estimation model.
- `airsim/` — Vendored AirSim Python RPC client. **Do not modify.**
- `msgpackrpc/` — Custom msgpack-rpc shim for Python 3.12 compat. **Do not modify.**
- `simple_airsim/` — Empty. Unused.
- `opensrc/` — Gitignored external sources. Not part of the project.
- `scripts/` — Install, launch, depth calibration, and model download scripts.
- `manual_flight_gui.py` — Manual flight GUI. Requires `[project.optional-dependencies] manual` (`uv sync --extra manual`).

## Registered Algorithms

All registered in `src/control/algorithms/__init__.py`:

| Name | File | Description |
|------|------|-------------|
| `six_directions` | `six_directions.py` | 6-axis test pattern (default baseline) |
| `attitude_four_motion` | `attitude_four_motion.py` | 4-direction motion with calibration |
| `opencv_landing` | `opencv_landing.py` | Vision-guided landing |
| `vision_guided_control` | `vision_guided_control.py` | Red-circle detector pursuit |
| `autonomous_explore` | `autonomous_explore.py` | Depth-based obstacle avoidance + ring/target pursuit |

Active algorithm set in `sim.config.json` → `"algorithm"`. Currently `"autonomous_explore"`.

## Adding a New Algorithm

1. Create `src/control/algorithms/my_algo.py`
2. Extend `Algorithm`, implement `run(self, client: airsim.MultirotorClient)`
3. Decorate with `@register("my_algo")`
4. **Import at bottom of `algorithms/__init__.py`** to trigger registration: `importlib.import_module("src.control.algorithms.my_algo")`
5. Set `"algorithm": "my_algo"` in `sim.config.json`
6. Access vision via `self.latest_frame()` and `self.vision_stats()` (set by `set_vision_feed`)

## Coordinate Frame

**NED** (North-East-Down). Negative z = above ground. Drone at 5m altitude → `z = -5.0`.

## Algorithm Config Sections in sim.config.json

Each algorithm has its own top-level config key matching its name (e.g. `"autonomous_explore"`, `"attitude_four_motion"`, `"vision_guided_control"`). These are read by the algorithm constructor via `self._config`.

Key top-level config keys:
- `"control"` — `command_rate_hz`, `latency_tuning`, `max_speed_ms`, `max_altitude_m`
- `"vision"` — FPV feed: `enabled`, `fps`, `fov_degrees`, `resolution`, `depth` (ONNX model)
- `"landing"` — `profile`, `descent_speed_ms`, safety caps, telemetry toggle
- `"safety"` — `algorithm_timeout_seconds`
- `"low_end_profile"` — overrides applied when `AIGP_LOW_END=1`
- `"waypoints"` — NED coordinate list

## Ruff Exclusions

Ruff excludes: `.venv/`, `airsim/`, `msgpackrpc/`, `opensrc/`, `.claude/`, `.sisyphus/`, `.ruff_cache/`. Do not lint vendored code.
