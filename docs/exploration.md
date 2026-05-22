# Exploration scheduler (`autonomous_explore.exploration`)

Structured default mapping for the drone challenge: boustrophedon-style legs, altitude layers, periodic 360° panoramas, and a lightweight SLAM-inspired layer (pose + coarse grid + landmarks + loop closure). Inspired by [simultaneous localization and mapping](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) (estimate pose **x** and map **m** from controls and observations).

Config lives under `autonomous_explore.exploration` in `sim.config.json`.

## Scheduler keys

| Key | Default | Role |
|-----|---------|------|
| `panorama_enabled` | `true` | Periodic full yaw scan (observation sweep) |
| `panorama_interval_s` | `15` | Min seconds between panoramas |
| `panorama_yaw_rate_dps` | `45` | Yaw rate during panorama |
| `panorama_creep_speed_ms` | `0.4` | Forward speed while scanning |
| `leg_enabled` | `true` | XY leg headings after each leg duration |
| `leg_duration_s` | `10` | Straight-leg cruise time before turn |
| `leg_turn_deg` | `90` | Heading change per leg |
| `leg_turn_rate_dps` | `35` | Turn rate during `LEG_TURN` |
| `altitude_layers_m` | `[3.5, 5.0, 6.5]` | NED hold altitudes (m above ground) |
| `altitude_layer_dwell_s` | `18` | Time at each layer before next |
| `vertical_depth_bias_*` | see config | Climb/descend from upper/lower depth clearance |
| `legacy_scan_enabled` | auto | `false` when panorama on; ±30° search if panorama off |

## SLAM-inspired layer (`exploration.slam`)

| Key | Default | Role |
|-----|---------|------|
| `enabled` | `true` | Pose/grid/landmark updates |
| `grid_cell_m` | `1.0` | Occupancy-style visit grid resolution |
| `loop_closure_enabled` | `true` | Revisit spawn → trigger panorama |
| `loop_closure_radius_m` | `2.5` | Distance to spawn for closure |
| `loop_closure_min_path_m` | `8.0` | Min path length before closure counts |
| `active_exploration_enabled` | `true` | Bias yaw toward least-visited bearings |
| `active_exploration_gain_deg_s` | `12` | Max yaw bias from active exploration |
| `landmark_enabled` | `true` | Register blue/red detections in map frame |
| `log_odds_enabled` | `true` | Probabilistic free/occupied grid cells |
| `frontier_enabled` | `true` | Steer toward unknown cells adjacent to free space |
| `imu_yaw_assist_gain` | `0.85` | Gyro yaw-rate assist when depth is lost |
| `export_path` | `logs/exploration_map.json` | End-of-run map JSON (path, cells, landmarks) |

**Localization:** AirSim NED position + yaw each tick.  
**Mapping:** log-odds grid (free/occupied), depth-ray updates, visual landmarks with depth-based range.  
**Loop closure:** return near spawn after sufficient path → extra `PANORAMA_360`.  
**Active exploration:** frontier cells first, then least-visited bearings.  
**IMU assist:** `zgyro` bias on wander when MiDaS depth is unavailable.  
**Export:** `logs/exploration_map.json` with path, cells, landmarks, metrics.

## Runtime behavior

- Defer panorama ~2s after losing a target; `reset_panorama_timer` on blue/red lock.
- Logs include `slam_path=…m cells=…` during wander.

## Reuse

Import from `src.control.exploration`: `ExplorationScheduler`, `ExplorationSlam`, `apply_wander_move`, `parse_exploration_settings`, `parse_slam_settings`.
