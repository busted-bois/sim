# UE 5.4 + BlocksV2 + AirSim (SimpleFlight RPC)

This is the **default** runtime path for the AIGP drone — autonomous flight via AirSim's msgpack-rpc, no PX4 in the loop. Use this for `main.py` algorithm runs.

## Prerequisites

- Windows host with UE 5.4 / Colosseum installed.
- Repo cloned, `uv sync` already run (see [`scripts/install.ps1`](../../../../scripts/install.ps1)).
- `.env.local` exists at repo root and contains `PROJECT_PATH=<full path to BlocksV2.uproject>`.
  - If not, run: `powershell -ExecutionPolicy Bypass -File ".\scripts\fix_project_path.ps1"` — it searches the machine and writes the file.

## `sim.config.json` settings that matter

The repo defaults already match this combo. Verify these keys (do not change unless instructed):

```json
{
  "simulator": {
    "map_name": "BlocksV2",
    "map_asset": "/Game/FlyingCPP/Maps/FlyingExampleMapV2",
    "specification_profile": "official_conformant",
    "airsim_port": 41451,
    "physics_update_hz": 120,
    "windowed": true,
    "res_x": 1280,
    "res_y": 720
  },
  "control": { "transport": "airsim" }
}
```

Valid values for `control.transport`: `"airsim"` (SimpleFlight RPC, this combo), `"mavlink"` (PX4), or `"auto"` (probes for a MAVLink HEARTBEAT and falls back to `airsim`). Anything else triggers a warning and coerces to `airsim` — see `src/mavlink_endpoints.py:_coerce_transport_with_guardrails`.

If `control.transport` is currently `"mavlink"` (the repo ships with it set that way for the jitter test), flip it to `"airsim"` for SimpleFlight runs. The launcher will also detect leftover PX4Multirotor `settings.json` and auto-restore the SimpleFlight backup on `uv run sim`.

**Algorithm must be RPC-compatible.** If `"algorithm"` is currently `"mavlink_jitter"` (MAVLink-only), change it to one of: `autonomous_explore` (standard autonomous build), `six_directions` (baseline test), `attitude_four_motion` (4-direction calibration), `vision_guided_control` (red-circle pursuit), or `opencv_landing` (vision-guided landing). The launcher will refuse to run a MAVLink-only algorithm with `transport="airsim"`.

## `~/Documents/AirSim/settings.json`

Must declare a **SimpleFlight** vehicle, not PX4Multirotor. See [`airsim-settings.md`](airsim-settings.md) for the exact shape. The launcher writes the right file as a side effect of `uv run sim`; manual edits are only needed if you've been hand-editing it.

Quick restore from backup if you suspect drift:

```bash
uv run sim-restore-simpleflight
```

## Launch

```bash
uv run sim
```

One command — loads `.env.local`, launches UE5 (if `PROJECT_PATH` is set), waits for AirSim RPC on `:41451`, then autostarts `main.py` with the algorithm named in `sim.config.json` (currently `autonomous_explore` for the standard build).

Variants:

- `uv run sim low-end` — lower resolution, `attitude_four_motion` algorithm, reduced telemetry.
- `uv run sim 3rd-person` — `FlyWithMe` chase camera instead of FPV.
- `uv run sim-very-soft` — gentle landing profile.
- `uv run preflight` — sanity check (RPC reachable, MAVLink optional). Safe before launch.

## Verification

In the launcher terminal, expect:

1. `[launch] Unreal started`
2. `Connected!`
3. `Algorithm: autonomous_explore` (or whatever is set in `sim.config.json`)
4. Periodic `phase_start name=...` lines

In the Unreal viewport, the drone should be visible at the start pose and arm on the first algorithm tick.

Stop with `Ctrl+C` in the terminal running the launcher. `SIGINT` cleans up both `main.py` and the UE process. Closing UE first usually works but may leave the client logging RPC disconnect errors.

## Common pitfalls

- **Unreal opens to the project picker** — `PROJECT_PATH` is wrong. Re-run `fix_project_path.ps1`.
- **`AirSim RPC not reachable`** — UE hasn't finished loading. The launcher waits up to `simulator.rpc_ready_timeout_seconds` (120 s default). If it gives up, raise the timeout or check UE didn't crash silently.
- **Drone falls through the floor / no physics** — `physics_update_hz` mismatch. Re-run `uv run extract-simulator-specs` and confirm `docs/simulator_specs.json` shows 120 Hz, then `uv run verify-sim-physics-metadata`.

More symptoms in [`troubleshooting.md`](troubleshooting.md).
