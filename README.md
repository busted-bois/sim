# AIGP Drone Challenge

Autonomous drone navigating the Colosseum (UE5/AirSim) simulator for the [AI Grand Prix](https://theaigrandprix.com).

## Quick Start

**Prerequisites:** Python 3.10+, [uv](https://docs.astral.sh/uv/)

See `How to Launch Drone on Unreal Engine 5.4` for the full step-by-step startup flow.

For a quick return run after setup:

```bash
uv run sim
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

8. **Optional gentler landing profile (new)**
   - Run:
     ```bash
     uv run sim-very-soft
     ```

9. **Manual mode (if you start simulator yourself)**
   - Start your AirSim/Unreal environment first and wait until loaded.
   - Then run only the drone controller:
     ```bash
     uv run main.py
     ```

10. **Confirm successful start in terminal output**
    - Look for:
      - `Connected!`
      - `Algorithm: attitude_four_motion`
      - phase logs like `phase_start name=stabilize`

## Operations checklist

| Step | What to do |
| ---- | ---------- |
| **Start** | From repo root: `uv run preflight` (optional), then `uv run sim`. Ensure `.env.local` has `PROJECT_PATH` to your `.uproject`. |
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
| `vision`    | FPV feed config (`enabled`, `camera_name`, `fps`, `fov_degrees`, optional debug frame dumps) |
| `waypoints` | NED coordinate waypoints                                           |
| `logging`   | Log level, telemetry logging toggle                                |
| `landing`   | Landing profile, safety caps, optional CSV telemetry during landing |
| `preflight` | Optional strict AirSim reachability before flight |
| `safety`    | Algorithm timeout / failsafe                                       |

**`.env.local`** — `PROJECT_PATH` pointing to your UE5 Colosseum project.

**Coordinate frame:** NED (North-East-Down). Negative z = above ground. Drone at 5m altitude → `z = -5.0`.

### Latency tuning notes

- Keep `vision.fps` and `control.command_rate_hz` coordinated so commands are based on fresh pixels.
- `control.latency_tuning.commands_per_frame=2.0` means up to two control updates per captured frame.
- Runtime logs from `attitude_four_motion` now include loop timing (`avg_ms`, `max_ms`, `overruns`) and vision drops (`sched_drop`, `consumer_drop`) for tuning.
- Optional auto-tuner pass: `control.latency_tuning.autotuner` runs for `duration_seconds` and prints recommended `vision.fps` and `control.command_rate_hz` from measured overrun/drop ratios.
- Set `control.latency_tuning.autotuner.output_json.enabled=true` to write results to `logs/latency_tuning_recommendation.json` (or a custom path) for run-to-run comparisons.

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
