# AIGP Drone Challenge

Autonomous drone navigating the Colosseum (UE5/AirSim) simulator for the [AI Grand Prix](https://theaigrandprix.com).

## Quick Start

**Prerequisites:** Python 3.10+, [uv](https://docs.astral.sh/uv/)

See `How to Launch Drone on Unreal Engine 5.4` for the full step-by-step startup flow.
For migrating the maze maps from AirsimSimulation, see `docs/AIRSIM_MAP_MIGRATION.md`.

**One-shot maze deploy (clone repo + patch `sim.config.json`):**

```bash
powershell -ExecutionPolicy Bypass -File ".\scripts\deploy_maze_sim.ps1"
```

If UE 4.16 is under a non-default folder, set `EPIC_GAMES_PATH` and re-run the script, or edit `simulator.maze_colosseum_path` in `sim.config.json`. UE 4.x may use **`UE4Editor.exe`** instead of `UnrealEditor.exe`; the launcher accepts either under `Engine\\Binaries\\Win64\\`.

For a quick return run after setup:

```bash
uv run sim
```

For smoother flight on lower-end laptops (reduced telemetry/logging):

```bash
uv run sim low-end
```

To launch the **AirsimSimulation** maze (UE 4.16 project with `MapMaze`) while keeping the same flight algorithm, set paths then run:

1. Clone the maze repo (if needed): `powershell -ExecutionPolicy Bypass -File ".\scripts\extract_airsim_maps.ps1"`
2. In `sim.config.json`, optionally set `simulator.maze_colosseum_path` to your **UE 4.16** `UnrealEditor.exe`. If unset, `uv run sim maze` tries Epic’s default folders (`UE_4.16`, or any engine folder whose name contains `4.16`). Optionally set `simulator.maze_project_path` to `VehilceAdvanced.uproject`; if empty, the launcher uses `opensrc/AirsimSimulation/VehilceAdvanced.uproject` when that file exists.
3. Or use env vars (override config): `MAZE_COLLOSSEUM_PATH`, `MAZE_PROJECT_PATH`. You can also set `EPIC_GAMES_PATH` if games are installed outside `C:\\Program Files\\Epic Games`.

```bash
uv run sim maze
```

To quickly test different algorithms in maze mode without editing `sim.config.json`, set:

```bash
$env:AIGP_MAZE_ALGORITHM="maze_straight_collision"  # maze-specific override
# or
$env:AIGP_ALGORITHM="attitude_four_motion"          # global override (wins in all modes)
```

Equivalent dedicated entrypoint:

```bash
uv run sim-maze
```

`uv run sim` (without `maze`) still uses `simulator.colosseum_path` and `PROJECT_PATH` for Colosseum / BlocksV2.

`uv run sim maze` **only** starts UE **4.16** + the maze `.uproject` (default `opensrc/AirsimSimulation/VehilceAdvanced.uproject`). Project path resolution order is: `MAZE_PROJECT_PATH` -> `simulator.maze_project_path` -> `PROJECT_PATH` -> default under `opensrc/`. It still does **not** use `simulator.colosseum_path` for the simulator process.

To run with the old third-person chase camera:

```bash
uv run sim 3rd-person
```

## How to Launch Drone on Unreal Engine 5.4

From a fresh Cursor session, use this exact sequence.

1. **Open project in Cursor**
   - `File -> Open Folder...`
   - Choose: `C:\Users\yourName\Downloads\Drone Project\anduril-agp_drone-challenge`

2. **Open terminal in Cursor**
   - Press ``Ctrl+` ``
   - Run:
     ```bash
     cd "C:\Users\yourName\Downloads\Drone Project\anduril-agp_drone-challenge"
     ```

3. **One-time path fix (safe to rerun)**
   - Run:
     ```bash
     powershell -ExecutionPolicy Bypass -File ".\scripts\fix_project_path.ps1"
     ```

4. **Install/sync dependencies**
   - Run:
     ```bash
     uv sync
     ```

5. **(Optional) Verify lint is clean**
   - Run:
     ```bash
     uvx ruff check --fix
     ```

6. **Preferred launch (new): one command**
   - Run:
     ```bash
     uv run sim
     ```
   - This now loads `.env.local`, attempts to auto-fix `PROJECT_PATH` on Windows when missing, launches UE5 if configured, waits for startup, then runs `main.py`.

7. **Optional safety check before launch (new)**
   - Run:
     ```bash
     uv run preflight
     ```
   - For maze mode readiness checks (UE 4.16 editor + maze `.uproject`):
     ```bash
     uv run preflight maze
     ```

8. **Optional gentler landing profile (new)**
   - Run:
     ```bash
     uv run sim-very-soft
     ```

9. **Optional low-end smooth-flight mode (new)**
   - Run:
     ```bash
     uv run sim low-end
     ```
   - This launches a lower-resolution simulator window, keeps `attitude_four_motion`, disables heavy telemetry/vision paths by default, and prints basic flight logs only.

10. **Optional third-person camera mode (new)**
   - Run:
     ```bash
     uv run sim 3rd-person
     ```
   - This keeps the old third-person (`FlyWithMe`) viewport mode while still running the same drone stack.

11. **Optional maze launch mode (new)**
   - Configure `simulator.maze_colosseum_path` (UE 4.16 editor) and optionally `simulator.maze_project_path` (see Quick Start above).
   - Run:
     ```bash
     uv run sim maze
     ```
   - Launches the maze `.uproject` with `-map=/Game/MazeCreator/Maps/MapMaze` when the map files exist on disk. AirSim in that repo is older than the Colosseum stack; if RPC or `takeoff` errors appear, treat that as an AirSim version mismatch, not a launcher bug.

12. **Manual mode (if you start simulator yourself)**
   - Start your AirSim/Unreal environment first and wait until loaded.
   - Then run only the drone controller:
     ```bash
     uv run main.py
     ```

13. **Confirm successful start in terminal output**
    - Look for:
      - `Connected!`
      - `Algorithm: attitude_four_motion`
      - phase logs like `phase_start name=stabilize`

## Operations checklist

| Step | What to do |
| ---- | ---------- |
| **Start** | From repo root: `uv run preflight` (optional), then `uv run sim`. Ensure `.env.local` has `PROJECT_PATH` to your `.uproject`. |
| **Low-end smooth mode** | Use `uv run sim low-end` to prioritize smoother control on weaker hardware by reducing runtime logging and telemetry load. |
| **Stop** | In the terminal running the client, press `Ctrl+C`. Then close Unreal. Closing Unreal first may disconnect the client with errors; that is usually harmless. |
| **Reset** | Restart the Unreal/AirSim session if the drone or API state acts stuck; run `uv run sim` again. |
| **Softer landing** | `uv run sim-very-soft` uses the gentle landing profile. |
| **Landing telemetry** | With `landing.telemetry_log.enabled` true in `sim.config.json`, each run writes `logs/landing_telemetry.csv` (`t_s`, `altitude_m`, `vz_ms`, `command`) during the landing phase for tuning. |

### Common errors

| Symptom | Likely cause | Fix |
| ------- | ------------ | --- |
| Unreal asks for a project file first | `PROJECT_PATH` missing or wrong | Set `PROJECT_PATH` in `.env.local` or `simulator.project_path`, or run `pwsh scripts/fix_project_path.ps1`. |
| Connection / RPC errors | Simulator not up or wrong port | Start Unreal; check `simulator.airsim_port` matches AirSim. |
| Preflight warns “AirSim RPC not reachable” | Normal if UE is not running yet | Start the sim, or set `preflight.require_airsim_reachable` to `false` (default) to only warn. |

## Project Structure

```
main.py                          # Entry point — AirSim RPC client
sim.config.json                  # Runtime config
src/
  config.py                      # Config loader
  landing_telemetry.py           # Optional CSV samples during landing
  control/algorithms/            # ← custom algorithms go here
    __init__.py                  # Algorithm base class + registry
    six_directions.py            # Default: 6-direction test pattern
airsim/                          # Vendored AirSim Python client
msgpackrpc/                      # Custom msgpack-rpc shim (Python 3.12 compat)
scripts/                         # Install/launch scripts
```

## Writing a Custom Algorithm

1. Create `src/control/algorithms/my_algo.py`
2. Extend `Algorithm`, implement `run(client)`
3. Add `@register("my_algo")` decorator
4. Set `"algorithm": "my_algo"` in `sim.config.json`

```python
from src.control.algorithms import Algorithm, register

@register("my_algo")
class MyAlgo(Algorithm):
    def run(self, client):
        client.takeoffAsync().join()
        client.moveByVelocityAsync(1.0, 0.0, 0.0, 5.0).join()
        client.hoverAsync().join()
```

## Configuration

**`sim.config.json`** top-level sections:

| Section     | Purpose                                                            |
| ----------- | ------------------------------------------------------------------ |
| `algorithm` | Name of registered algorithm to run                                |
| `simulator` | UE5/AirSim paths and ports                                         |
| `control`   | `command_rate_hz`, optional `latency_tuning` (`commands_per_frame`, `max_command_rate_hz`), `max_speed_ms`, `max_altitude_m` |
| `vision`    | FPV feed config (`enabled`, `camera_name`, `fps`, `min_fps`, `fov_degrees`, `compress`, optional `resolution` or `width/height`, startup auto-tune knobs, optional debug frame dumps) |
| `waypoints` | NED coordinate waypoints                                           |
| `logging`   | Log level, telemetry logging toggle                                |
| `landing`   | Landing profile, safety caps, optional CSV telemetry during landing |
| `preflight` | Optional strict AirSim reachability before flight |
| `safety`    | Algorithm timeout / failsafe                                       |

**`.env.local`** — `PROJECT_PATH` pointing to your UE5 Colosseum project.

Within `simulator`, normal launch uses `colosseum_path` and optional `project_path`. For `uv run sim maze`, set `maze_colosseum_path` (UE 4.16 `UnrealEditor.exe`) and optionally `maze_project_path`; overrides: `MAZE_COLLOSSEUM_PATH`, `MAZE_PROJECT_PATH`.

**Coordinate frame:** NED (North-East-Down). Negative z = above ground. Drone at 5m altitude → `z = -5.0`.

### Latency tuning notes

- Keep `vision.fps` and `control.command_rate_hz` coordinated so commands are based on fresh pixels.
- Start from `vision.fps=18`, `vision.fov_degrees=100`, and `vision.resolution=[640, 360]` for a low-latency baseline before pushing quality or field-of-view higher.
- Startup auto-adaptation is enabled by default (`vision.startup_autotune_enabled=true`): during the first few seconds, runtime capture throughput is measured and `vision.fps` is clamped down (never below `vision.min_fps`) so slower machines avoid frame-age buildup.
- `control.latency_tuning.commands_per_frame=2.0` means up to two control updates per captured frame.
- Runtime logs from `attitude_four_motion` now include loop timing (`avg_ms`, `max_ms`, `overruns`) and vision drops (`sched_drop`, `consumer_drop`) for tuning.
- Optional auto-tuner pass: `control.latency_tuning.autotuner` (disabled by default) runs for `duration_seconds` and prints recommended `vision.fps` and `control.command_rate_hz` from measured overrun/drop ratios.
- Set `control.latency_tuning.autotuner.output_json.enabled=true` to write results to `logs/latency_tuning_recommendation.json` (or a custom path) for run-to-run comparisons.

### Quick test tuning (low-end mode)

- `low_end_profile` in `sim.config.json` controls `uv run sim low-end` behavior (algorithm, reduced command rate, simulator resolution, and lightweight logging/telemetry).
- `six_directions` settings (`duration_s`, `speed_ms`, `direction_labels`) are used by normal runs and can be overridden by low-end mode for fast sanity checks.

## 👥 Team

### Advisors

- Aneesh Saxena
- Tushar Shrivastav

### Simulation & Network Communication

- Sameer Faisal
- Yat Chun Wong
- Ryan Yang
- Trung Nguyen

### Algorithms & Autonomy

- Kunal Shrivastav
- Samyak Kakatur
- David Vayntrub
- Ram Rao
