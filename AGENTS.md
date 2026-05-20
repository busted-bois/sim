# AGENTS.md

This repo uses multiple AI coding tools (OpenCode, Claude, Cursor, Copilot). Instruction files are symlinked from this single source of truth:

- `AGENTS.md` ← canonical
- `CLAUDE.md` → symlink to `AGENTS.md`
- `.claude/CLAUDE.md` → symlink to `AGENTS.md`
- `.cursor/rules/AGENTS.md` → symlink to `AGENTS.md`

**When updating instructions, edit `AGENTS.md` only. Never edit the symlinks directly.** If adding/removing tools, update the symlinks accordingly.

## Tooling

- **Python:** `uv` for all operations (`uv run`, `uv sync`). No bare `python3`.
- **Linter:** Ruff (`uvx ruff check --fix`). Runs via lefthook pre-commit on staged `*.{py,toml}`.
- **Ruff config:** line-length=100, rules `E F W I UP RUF`, `known-first-party = ["src"]`. See `pyproject.toml [tool.ruff]`.
- **Python version:** 3.12 (`.python-version`). `requires-python >= 3.10`.
- **Tests:** `uv run python -m unittest discover -s tests -v`. No pytest.
- **CI:** `.github/workflows/ruff.yml` runs `uvx ruff check .` on PRs to main; `.github/workflows/tests.yml` runs `unittest` and `uv run verify-sim-physics-metadata`. Both enable `setup-uv` dependency caching. Optional: `.github/workflows/extract-simulator-specs.yml` (`workflow_dispatch`) when repo secret `UE_PROJECT_PATH` is set on a Windows runner.

## Simulator physics (120 Hz)

- **Unreal:** fixed async physics step and substepping set in Colosseum (`PROJECT_PATH`). Optional merge: `scripts/unreal_default_engine_physics_120hz_fragment.ini` → `Config/DefaultEngine.ini`.
- **Snapshot:** `docs/simulator_specs.json` from `uv run extract-simulator-specs`. With `simulator.specification_required: true`, `uv run preflight` and `uv run sim` validate it.
- **Vision / conformant:** `vision.fov_degrees` is **horizontal** FOV (see `src/vision/intrinsics.py`). With `simulator.specification_profile: official_conformant`, preflight pins FOV and 640×360; keep defaults and `profiles.official_conformant` aligned. `camera.pitch_up_degrees` is pilot "up"; UE camera `Pitch` in settings JSON negates it.

## Running

```bash
uv run sim                              # One command: .env.local, UE5, PX4-SITL (WSL, auto on Windows), main.py
uv run sim low-end                      # Low-res, reduced telemetry, attitude_four_motion
uv run sim 3rd-person                   # FlyWithMe camera instead of FPV
uv run sim-very-soft                    # Gentle landing profile
uv run preflight                        # Safety check before launch
uv run verify-sim-physics-metadata      # Assert docs/simulator_specs.json documents 120 Hz physics
uv run extract-simulator-specs          # Regenerate snapshot from Unreal (needs PROJECT_PATH + UE)
uv run check-mavlink                    # Sniff UDP 14540/14550 for MAVLink frames (--duration 0 = forever)
uv run sim-mavlink                      # Launch UE with PX4Multirotor settings (needs PX4-SITL in WSL); Ctrl+C to stop
uv run sim-mavlink probe                # Same, but auto-run check-mavlink for 60s after launch
uv run mavlink-all                      # All-in-one: UE + PX4-SITL (WSL) + probe; logs to logs/mavlink/<ts>/
uv run check-mavlink --decode-attitude    # Also decode ATTITUDE roll/pitch/yaw
uv run attitude-listen                  # ATTITUDE-only UDP listener (Layer 1, no pymavlink)
uv run main.py                          # Run drone client (needs MAVLink HEARTBEAT)
uv run highres-imu-smoke                # Probe HIGHRES_IMU against running MAVLink endpoint
uv run sim-highres-imu-smoke            # Launch sim first, then probe HIGHRES_IMU
uv run timesync-smoke                   # Probe TIMESYNC against running MAVLink endpoint
uv run sim-timesync-smoke               # Launch sim first, then probe TIMESYNC
uv run attitude-smoke                   # Short MAVLink SET_ATTITUDE_TARGET stream
uv run sim-attitude-smoke               # Same via sim_launch (simulator settings + UE if configured)
```

**Flight control is MAVLink-only.** `main.py` uses `PymavlinkFlightClient` (`SET_POSITION_TARGET_LOCAL_NED`, `SET_ATTITUDE_TARGET`, `HIGHRES_IMU`, `TIMESYNC`). `sim-mavlink` / `mavlink-all` verify the bridge; they do not run `main.py`.

After Unreal starts (or if launch is skipped), the launcher **autostarts** `main.py` once a MAVLink HEARTBEAT is seen, up to `simulator.rpc_ready_timeout_seconds`. On Windows with `control.mavlink.auto_start_px4` (default true), `uv run sim` also starts PX4-SITL in WSL after the HIL TCP listener is up (same prerequisites as `mavlink-all`: mirrored `.wslconfig`, built `~/PX4-Autopilot`). Set `AIGP_AUTO_PX4=0` to start PX4 manually.

**Ctrl+C** (SIGINT) stops `main.py` and (if launcher started it) the UE process. **SIGTERM** is not wired to cleanup — Unreal may stay open.

### Environment Variables

Set via `.env.local` (loaded by `sim_launch.py` and `launch.sh`) or inline:
- `PROJECT_PATH` — path to `.uproject`. Required for UE5 launch.
- `AIGP_LOW_END=1` — low-end mode override.
- `AIGP_LANDING_PROFILE` — override landing profile (`very_soft`, `faster_soft`).
- `AIGP_ENABLE_TRACE=1` — enable flight path trace line.
- `AIGP_PAUSE_BEFORE_EXIT=1` — pause before client exit.
- `AIGP_MAVLINK_ENDPOINT` — override MAVLink UDP listen endpoint.
- `AIGP_AUTO_PX4` — set to `0` to skip auto-starting PX4-SITL in WSL during `uv run sim`.
- `SIMULATOR_RPC_PORT` — simulator API port (launcher / legacy tools).

### Output Artifacts

- `logs/landing_telemetry.csv` — landing telemetry (when `landing.telemetry_log.enabled=true`).
- `logs/latency_tuning_recommendation.json` — autotuner results.
- `logs/vision_frames/` — debug frame dumps (when `vision.save_debug_frames=true`).

## Project Layout

- `main.py` — Entry point. MAVLink client + algorithm loop.
- `sim.config.json` — Runtime config (algorithm, MAVLink, vision, landing).
- `.env.local` — `PROJECT_PATH` to UE5 project.
- `src/config.py` — Reads `sim.config.json`.
- `src/sim_launch.py` — Launcher (`uv run sim`, smokes, mavlink orchestration).
- `src/control/main_loop.py` — Algorithm timeout and optional tick loop.
- `src/control/mavlink_client.py` — `PymavlinkFlightClient`.
- `src/control/flight_client.py` — Protocol + MAVLink setpoint types.
- `src/control/algorithms/` — Pluggable algorithms (`@register`).
- `src/vision/feed.py` — Vision stub (phase 1; no camera transport).
- `opensrc/` — Gitignored external sources.

## Registered Algorithms

| Name | File | Description |
|------|------|-------------|
| `mavlink_jitter` | `mavlink_jitter.py` | MAVLink stress-test pattern |
| `six_directions` | `six_directions.py` | 6-axis SET_POSITION_TARGET demo |
| `attitude_four_motion` | `attitude_four_motion.py` | 4-direction attitude control |
| `opencv_landing` | `opencv_landing.py` | Vision landing (needs vision phase 2) |
| `vision_guided_control` | `vision_guided_control.py` | Target pursuit (needs vision phase 2) |
| `autonomous_explore` | `autonomous_explore.py` | Depth exploration (needs vision phase 2) |

Default algorithm in `sim.config.json`: `mavlink_jitter`.

## Coordinate Frame

**NED** (North-East-Down). Negative z = above ground.

### MAVLink `SET_POSITION_TARGET_LOCAL_NED`

- **`MAV_FRAME_LOCAL_NED`**: fixed origin; `x` north, `y` east, `z` down.
- **`MAV_FRAME_BODY_NED`**: body frame; `x` forward, `y` right, `z` down.

Helpers: `src/control/setpoints.py`, `PymavlinkFlightClient`. Checklist: `docs/set_position_target_pr_test_plan.md`.

## Ruff Exclusions

Ruff excludes: `.venv/`, `opensrc/`, `.claude/`, `.sisyphus/`, `.ruff_cache/`.
