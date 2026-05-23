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
- **Snapshot:** `docs/simulator_specs.json` from `uv run extract-simulator-specs`. With `simulator.specification_required: true`, `uv run preflight` and `uv run sim` validate it. AirSim RPC does not read live `PhysicsSettings` from Python.
- **Vision / conformant:** `vision.fov_degrees` is **horizontal** FOV (see `src/vision/intrinsics.py`). With `simulator.specification_profile: official_conformant`, preflight pins FOV and 640×360; keep defaults and `profiles.official_conformant` aligned. `camera.pitch_up_degrees` is pilot "up"; AirSim `Pitch` and `set_front_camera_pose` negate it for UE.

## Running

```bash
uv run sim                              # One command: .env.local, UE5 (if configured), main.py
# With control.transport "mavlink" (Windows): uv run sim also auto-starts PX4-SITL in WSL, then main.py
uv run sim low-end                      # Low-res, reduced telemetry, attitude_four_motion
uv run sim 3rd-person                   # FlyWithMe camera instead of FPV
uv run sim-very-soft                    # Gentle landing profile
uv run sim vjoy                         # Manual vJoy control + GUI
uv run preflight                        # Safety check before launch (--static-only or AIGP_PREFLIGHT_STATIC_ONLY=1 skips live UDP)
uv run prerun                           # Static MAVLink/config checks only (no UE); then uv run sim
uv run verify-sim-physics-metadata      # Assert docs/simulator_specs.json documents 120 Hz physics
uv run extract-simulator-specs          # Regenerate snapshot from Unreal (needs PROJECT_PATH + UE)
uv run calibrate                        # Depth calibration with manual GUI
uv run check-mavlink                    # Sniff UDP 14540/14550 for MAVLink frames (--duration 0 = forever)
uv run sim-mavlink                      # Launch UE with PX4Multirotor settings (needs PX4-SITL in WSL); Ctrl+C to stop
uv run sim-mavlink probe                # Same, but auto-run check-mavlink for 60s after launch
uv run mavlink-all                      # All-in-one: UE + PX4-SITL (WSL) + probe; logs to logs/mavlink/<ts>/. Requires WSL mirrored networking.
uv run sim-restore-simpleflight         # Manually restore SimpleFlight settings.json from backup (uv run sim does this automatically)
uv run check-mavlink --decode-attitude    # Also decode ATTITUDE roll/pitch/yaw
uv run attitude-listen                  # ATTITUDE-only UDP listener (Layer 1, no pymavlink)
uv run position-print                   # Print current NED position once and exit
uv run position-print --stream          # Repeat until Ctrl+C; --duration 30 for timed stream
uv run position-print --transport airsim  # Force SimpleFlight RPC (no PX4 required)
uv run python scripts/smoke_attitude_integration.py  # Smoke (decode + UDP inject)
uv run --group dev pytest tests/ -q     # Unit tests (MAVLink ATTITUDE, etc.)
uv run main.py                          # Run drone client (needs simulator running)
uv run highres-imu-smoke                # Probe HIGHRES_IMU against running MAVLink endpoint
uv run sim-highres-imu-smoke            # Launch sim first, then probe HIGHRES_IMU
uv run timesync-smoke                   # Probe TIMESYNC against running MAVLink endpoint
uv run sim-timesync-smoke               # Launch sim first, then probe TIMESYNC
uv run attitude-smoke                   # Short MAVLink SET_ATTITUDE_TARGET stream
uv run sim-attitude-smoke               # Same via sim_launch (AirSim settings + UE if configured)
uv run tracking-smoke                   # LocalTracker + MAVLink (needs PX4-SITL / sim-mavlink)
uv run integration-smoke                # unittest + exploration-smoke + attitude integration (offline)
```

**MAVLink branch test (Windows):** set `control.transport` to `"mavlink"` in `sim.config.json`, build PX4 once in WSL (`make px4_sitl none_iris`), then run **`uv run sim` only** — do not chain `sim-mavlink` + `sim` (that launches two UE instances). Optional: `uv run prerun` before a long UE load. First time after enabling WSL mirrored networking: `wsl --shutdown` once. Launcher runs idempotent MAVLink prereqs (WSL, PX4 binary, `.wslconfig`, stale PX4 stop) then auto-starts PX4-SITL.

**MAVLink probe commands** (`sim-mavlink`, `mavlink-all`, etc.) verify the MAVLink bridge only — they do **not** run `main.py`. With `control.transport: "airsim"`, `uv run sim` uses SimpleFlight RPC (default autonomous path).

`uv run sim` auto-restores SimpleFlight settings from `~/Documents/AirSim/settings.simpleflight.bak.json` if leftover PX4Multirotor config detected — **skipped** when `control.transport` is `mavlink`. Use `uv run sim-restore-simpleflight` to force manually.

After Unreal starts (or if launch is skipped), the launcher **autostarts** `main.py` once AirSim RPC accepts connections, up to `simulator.rpc_ready_timeout_seconds`.

**Ctrl+C** (SIGINT) stops `main.py` and (if launcher started it) the UE process. **SIGTERM** is not wired to cleanup — Unreal may stay open.

### Environment Variables

Set via `.env.local` (loaded by `sim_launch.py` and `launch.sh`) or inline:
- `PROJECT_PATH` — path to `.uproject`. Required for UE5 launch.
- `AIGP_LOW_END=1` — low-end mode override.
- `AIGP_LANDING_PROFILE` — override landing profile (`very_soft`, `faster_soft`).
- `AIGP_ENABLE_TRACE=1` — enable flight path trace line.
- `AIGP_PAUSE_BEFORE_EXIT=1` — pause before client exit.
- `AIGP_SKIP_MAVLINK_INTEGRATION=1` — skip UDP loopback integration tests.
- `AIGP_SKIP_MAVLINK_PREREQ=1` — skip MAVLink WSL/PX4 prerequisite phase in `uv run sim`.
- `AIGP_PREFLIGHT_STATIC_ONLY=1` — `uv run preflight` skips live UDP/RPC probes (same as `--static-only`).
- `AIRSIM_PORT` — set by launcher, forwarded to main.py.

### Output Artifacts

- `logs/landing_telemetry.csv` — landing telemetry (when `landing.telemetry_log.enabled=true`).
- `logs/position_trace.csv` — MAVLink `LOCAL_POSITION_NED` path (when `control.mavlink.position_trace.enabled=true`).
- `logs/tracking_state.csv` — fused LOCAL_NED tracker (when `control.mavlink.tracking.enabled=true`; use with `uv run sim-mavlink`).
- On-screen position/time — AirSim `simPrintLogMessage` when `control.mavlink.position_hud.enabled=true`. `data_source: "tracking"` shows fused `LocalTracker` state (MAVLink + display RPC); `data_source: "trace"` uses `position_trace` (auto-enables trace store when HUD on).
- `logs/latency_tuning_recommendation.json` — autotuner results.
- `logs/vision_frames/` — debug frame dumps (when `vision.save_debug_frames=true`).

## Project Layout

- `main.py` — Entry point. Connects to AirSim RPC, loads algorithm from config, runs it.
- `sim.config.json` — Runtime config (algorithm name, sim ports, waypoints, control limits, vision, landing profiles).
- `.env.local` — `PROJECT_PATH` to UE5 project. Loaded by `uv run sim` / `launch.sh` / `launch.ps1`. Not committed.
- `src/config.py` — Reads `sim.config.json` from project root.
- `src/sim_launch.py` — Launcher. Entry point for `uv run sim`, `uv run sim-very-soft`, `uv run sim-low-end`, `uv run calibrate`, `uv run sim-attitude-smoke`, `uv run sim-timesync-smoke`.
- `src/preflight.py` — Preflight safety check. Entry points: `uv run preflight`, `uv run prerun` (static-only).
- `src/mavlink_prereq.py` — MAVLink WSL/PX4 prerequisites; used by `sim_launch` and preflight static checks.
- `src/landing_telemetry.py` — Optional CSV samples during landing.
- `src/position_trace.py` — MAVLink `LOCAL_POSITION_NED` trace (estimator fixes, CSV on shutdown).
- `src/tracking/` — Arm-gated LOCAL_NED origin, HIGHRES_IMU propagation, UDP vision sync, `TrackingSnapshot` API.
- `src/vision/udp_video.py` — UDP port 5600 chunked JPEG reassembly (`sim_time_ns` header).
- `src/position_hud.py` — On-screen HUD via AirSim `simPrintLogMessage` (display-only RPC; `position_hud.data_source` trace or tracking).
- `src/control/` — Flight client abstraction (`flight_client.py`), AirSim adapter, MAVLink client (`mavlink_client.py`), primitives, command rate, IMU, timesync.
- `src/control/algorithms/` — Pluggable flight algorithms via `@register("name")` decorator. Auto-discovered from `*.py` in the directory.
- `src/vision/` — Vision subsystem: `feed.py` (FPV capture), `intrinsics.py` (pinhole K), `depth_perception.py` (MiDaS ONNX), `processing.py`.
- `models/midas_v21_small.onnx` — Monocular depth estimation model.
- `airsim/` — Vendored AirSim Python RPC client. **Do not modify.**
- `msgpackrpc/` — Custom msgpack-rpc shim for Python 3.12 compat. **Do not modify.**
- `simple_airsim/` — Empty. Unused.
- `opensrc/` — Gitignored external sources. Not part of the project.
- `scripts/` — Install, launch, depth calibration, and model download scripts.
- `manual_flight_gui.py` — Manual flight GUI. Requires `uv sync --extra manual`.

## Registered Algorithms

Auto-discovered from `src/control/algorithms/*.py`:

| Name | File | Description |
|------|------|-------------|
| `six_directions` | `six_directions.py` | 6-axis test pattern (default baseline) |
| `attitude_four_motion` | `attitude_four_motion.py` | 4-direction motion with calibration |
| `opencv_landing` | `opencv_landing.py` | Vision-guided landing |
| `vision_guided_control` | `vision_guided_control.py` | Red-circle detector pursuit |
| `autonomous_explore` | `autonomous_explore.py` | Depth-based obstacle avoidance + ring/target pursuit |

Active algorithm set in `sim.config.json` → `"algorithm"`. Currently `"autonomous_explore"`.

### Exploration mapping (`autonomous_explore.exploration`)

Leg/altitude/panorama mapping plus optional `exploration.slam` (grid, landmarks, loop closure). See `docs/exploration.md` and `src.control.exploration`.

## Adding a New Algorithm

1. Create `src/control/algorithms/my_algo.py`
2. Extend `Algorithm`, implement `run(self, client: FlightClient)`
3. Decorate with `@register("my_algo")`
4. File is auto-discovered — no manual import needed (auto-scanned from `*.py` in directory)
5. Set `"algorithm": "my_algo"` in `sim.config.json`
6. Access vision via `self.latest_frame()` and `self.vision_stats()` (set by `set_vision_feed`)

## Coordinate Frame

**NED** (North-East-Down). Negative z = above ground. Drone at 5m altitude → `z = -5.0`.

### MAVLink `SET_POSITION_TARGET_LOCAL_NED`

- **`MAV_FRAME_LOCAL_NED`**: origin is a fixed ground point (typically arm position); `x` north, `y` east, `z` down.
- **`MAV_FRAME_BODY_NED`**: origin is the vehicle; `x` forward, `y` right, `z` down.

Helpers on `PymavlinkFlightClient` and `FlightClient` protocol. Checklist: `docs/set_position_target_pr_test_plan.md`.

### Internal NED mapping (`NedEnvironmentMap`)

- Module: `src/control/ned_environment.py` — ingests `LOCAL_POSITION_NED` + `ATTITUDE`, exposes LOCAL/BODY transforms and spawn-relative XY.
- MAVLink: `PymavlinkFlightClient.get_ned_environment()` (telemetry pump requests ATTITUDE when `control.mavlink.attitude.enabled`).
- Algorithms: `Algorithm.ned_environment(client)`, `ned_environment_health(client)`; `SensorSnapshot.ned` in `latest_sensor_snapshot()`.
- AirSim: `AirSimAdapter.get_ned_environment()` refreshes from RPC each call.
- Setpoint helper: `plan_body_velocity()`; debug: `NedEnvironmentMap.export_snapshot_json()`.
- Preflight (MAVLink): warns if ATTITUDE stream missing (`control.mavlink.attitude.require_stream` to hard-fail).
- Docs: `docs/ned_coordinate_mapping.md`.

## Algorithm Config Sections in sim.config.json

Each algorithm has its own top-level config key matching its name (e.g. `"autonomous_explore"`, `"attitude_four_motion"`). Read by algorithm constructor via `self._config`.

Key top-level config keys:
- `"control"` — `command_rate_hz`, `latency_tuning`, `max_speed_ms`, `max_altitude_m`
- `"vision"` — FPV feed: `enabled`, `fps`, `fov_degrees`, `resolution`, `depth` (ONNX model)
- `"landing"` — `profile`, `descent_speed_ms`, safety caps, telemetry toggle
- `"safety"` — `algorithm_timeout_seconds`
- `"low_end_profile"` — overrides applied when `AIGP_LOW_END=1`
- `"waypoints"` — NED coordinate list
- `"autonomous_explore"."exploration"` — mapping scheduler (panorama, legs, altitude layers); see `docs/exploration.md`

## Ruff Exclusions

Ruff excludes: `.venv/`, `airsim/`, `msgpackrpc/`, `opensrc/`, `simple_airsim/`, `.claude/`, `.sisyphus/`, `.ruff_cache/`. Do not lint vendored code.
