# Simulator troubleshooting

Generic symptom → cause → fix table for all `(engine, course, transport)` combinations. For combo-specific gotchas, see the individual runbook.

## Launcher / Unreal

| Symptom                                                  | Likely cause                                            | Fix                                                                                                       |
| -------------------------------------------------------- | ------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| Unreal opens to the project picker dialog                | `PROJECT_PATH` missing or stale                         | `powershell -ExecutionPolicy Bypass -File ".\scripts\fix_project_path.ps1"` (rewrites `.env.local`).      |
| `Failed to load map: /Game/.../<MapName>`                | Wrong/stale `-map=/Game/...` in `simulator.extra_ue_args` | Open the project in UE editor → right-click the umap → Copy Reference. Update the `-map=/Game/...` entry in `simulator.extra_ue_args` (and keep `simulator.map_asset` aligned if used elsewhere). |
| Drone spawns inside terrain / falls through floor        | Physics rate mismatch                                   | `uv run extract-simulator-specs` then `uv run verify-sim-physics-metadata`. Expect 120 Hz.                |
| Unreal launches but viewport stays black for >60 s       | First-run shader compile on this machine                | Wait. First launch can take 5+ minutes. Subsequent launches are fast.                                     |
| `uv run sim` returns immediately with no Unreal window   | `PROJECT_PATH` valid but UE binary missing              | Verify `simulator.colosseum_path` in `sim.config.json` points to a real `UnrealEditor.exe`.               |

## AirSim RPC (transport=airsim)

| Symptom                                                  | Likely cause                                            | Fix                                                                                                       |
| -------------------------------------------------------- | ------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `AirSim RPC not reachable` from preflight                | UE still loading                                        | Wait. Preflight is informational; the launcher itself waits up to `rpc_ready_timeout_seconds` (120 s).    |
| `main.py` never prints `Connected!`                      | Wrong port, or `settings.json` has PX4Multirotor        | `uv run sim-restore-simpleflight`. Confirm `ApiServerPort: 41451` matches `simulator.airsim_port: 41451`. |
| RPC connects but `simGetImages` returns empty            | Camera not in `Recording.Cameras` of `settings.json`    | Re-run `uv run sim` so the launcher rewrites `settings.json` with the camera block.                       |
| Algorithm times out at the safety cap                    | `safety.algorithm_timeout_seconds` too low for the run  | Raise it in `sim.config.json` only after confirming the algorithm isn't actually stuck.                   |

## MAVLink / PX4 (transport=mavlink)

| Symptom                                                  | Likely cause                                            | Fix                                                                                                       |
| -------------------------------------------------------- | ------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `WSL mirrored networking is NOT enabled`                 | `.wslconfig` missing `networkingMode=mirrored`          | `pwsh scripts/dev-mavlink.ps1 -EnableMirrored` then `wsl --shutdown`. Re-run.                              |
| `PX4 binary missing at ~/PX4-Autopilot/build/...`        | PX4 never built in WSL                                  | Run the [`setup-px4-mavlink-bridge`](../../setup-px4-mavlink-bridge/SKILL.md) skill (one-time setup).      |
| `HIL listener never came up on :4560`                    | UE crashed, or `settings.json` is SimpleFlight, not PX4 | Check `logs/mavlink/<ts>/ue.log`. If launcher should have written PX4 settings but didn't, run `uv run sim-mavlink` to force the write. |
| `PX4 exited early` / `bind: address already in use`      | Stale PX4 process in WSL                                | `wsl -e bash -c "pkill -INT -f 'px4 -i 0'; pkill -f 'make px4_sitl'"` then re-run.                         |
| MAVLink packets > 0 but ATTITUDE decoded = 0 for 90 s    | PX4 preflight failing, or QGC stealing UDP `:14550`     | Close QGC. Read PX4 console for preflight errors (sensor calibration, EKF). `mavlink-all` exits code 2 here. |
| `check-mavlink` shows packets only on `:14540` not `:14550` | Endpoint candidates ordering / QGC not running        | Normal — the launcher reads on `:14550` by default but `auto_discover_endpoints` will fall through.       |
| `HIGHRES_IMU` smoke test times out                       | IMU stream not enabled by PX4, or rate too low          | Confirm `control.mavlink.highres_imu.enabled: true`. PX4 must be armed-ready (not necessarily armed).     |
| `TIMESYNC` stable window never fills                     | Wall-clock noise from a busy WSL host                   | Increase `control.mavlink.timesync.max_offset_jitter_ms`. Don't run other heavy WSL workloads concurrently.|

## Vision / FPV feed

| Symptom                                                  | Likely cause                                            | Fix                                                                                                       |
| -------------------------------------------------------- | ------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| Frames are upside down or rolled                         | `camera.pitch_up_degrees` sign confusion (UE negates)   | Read the FOV note in `AGENTS.md` "Vision / conformant". Keep `pitch_up_degrees` as the pilot "up" value.  |
| FOV looks too narrow / too wide                          | `vision.fov_degrees` is **horizontal**, not diagonal    | If targeting the official_conformant profile, must be 90° at 640×360.                                     |
| `frame_age` warnings spam the console                    | Vision capture slower than `vision.fps` setting         | Drop `vision.fps` or enable `vision.startup_autotune_enabled: true`.                                      |

## Always-useful diagnostics

```bash
uv run preflight                          # high-level health check
uv run verify-sim-physics-metadata        # 120 Hz physics sanity
uv run check-mavlink --duration 20        # MAVLink packet flow (mavlink combos)
uv run check-mavlink --decode-attitude    # + ATTITUDE decode
```

Logs to look at:

- `logs/mavlink/<ts>/ue.log`, `px4.log`, `probe.log` — written by `mavlink-all`
- `logs/landing_telemetry.csv` — when `landing.telemetry_log.enabled=true`
- `logs/vision_frames/` — when `vision.save_debug_frames=true`
