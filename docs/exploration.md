# Exploration (`autonomous_explore.exploration`)

Leg-based XY coverage, altitude layers, periodic 360° panoramas, and optional `exploration.slam` (pose, log-odds grid, landmarks, loop closure). Config in `sim.config.json`.

## Scheduler keys

| Key | Default | Role |
|-----|---------|------|
| `panorama_enabled` | `true` | Periodic full yaw scan |
| `panorama_interval_s` | `15` | Min seconds between panoramas |
| `panorama_creep_speed_ms` | `0.4` | Forward speed while scanning |
| `leg_enabled` | `true` | XY leg headings |
| `leg_duration_s` | `10` | Cruise time before turn |
| `altitude_layers_m` | `[3.5, 5.0, 6.5]` | Hold altitudes (m AGL) |
| `legacy_scan_enabled` | auto | ±30° search when panorama off |

## SLAM (`exploration.slam`)

| Key | Default | Role |
|-----|---------|------|
| `log_odds_enabled` | `true` | Free/occupied grid from depth rays |
| `frontier_enabled` | `true` | Steer toward unknown cells |
| `loop_closure_*` | see config | Revisit spawn → panorama |
| `imu_yaw_assist_gain` | `0.85` | Gyro assist when depth missing |
| `export_path` | `logs/slam/exploration_map_{timestamp}.json` | End-of-run SLAM map JSON |
| `include_ned_payload` | `false` | When false, SLAM export does not embed internal NED mapping |

## Internal mapping (`exploration.internal_mapping`)

Separate from SLAM. Implemented in `src/internal_mapping.py`: threaded CSV of fused NED pose/velocity (~20 Hz).

| Key | Default | Role |
|-----|---------|------|
| `enabled` | `false` | Start background CSV logger |
| `path` | `logs/internal_mapping_{timestamp}.csv` | Output path |
| `sample_hz` | `20` | Sample rate |
| `data_source` | `tracking` when MAVLink tracking enabled, else `rpc` | `tracking` uses `getTrackingSnapshot()` with NED/RPC fallback |
| `log_imu` | `false` | Append raw HIGHRES_IMU columns (`xacc`…`zgyro`) |

CSV columns include `roll_rad`, `pitch_rad`, `tracking_status`, `imu_rate_hz`, and optional raw IMU fields. Not coupled to `exploration.slam`.

## Reuse

`src.control.exploration`: `ExplorationScheduler`, `ExplorationSlam`, `apply_wander_move`, `parse_exploration_settings`, `parse_slam_settings`.

SLAM pose uses **fused tracking** (`getTrackingSnapshot()` → ATTITUDE + HIGHRES_IMU + LOCAL_POSITION_NED) with NED/RPC fallback. Depth and landmark bearings use full attitude plus the 20° camera mount via `src.tracking.camera`. Scheduler / fly-through still use `NedEnvironmentMap` when available. See `docs/ned_coordinate_mapping.md`.
