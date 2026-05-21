# Exploration scheduler (`autonomous_explore.exploration`)

Structured default mapping: boustrophedon-style legs, altitude layer cycling, and periodic 360° panorama scans. Config lives under `autonomous_explore.exploration` in `sim.config.json`.

## Keys

| Key | Default | Role |
|-----|---------|------|
| `panorama_enabled` | `true` | Periodic full yaw scan |
| `panorama_interval_s` | `10` | Min seconds between panoramas |
| `panorama_yaw_rate_dps` | `45` | Yaw rate during panorama |
| `panorama_creep_speed_ms` | `0` | Forward speed while scanning (0 = spin in place) |
| `leg_enabled` | `true` | XY leg headings after each leg duration |
| `leg_duration_s` | `10` | Straight-leg cruise time before turn |
| `leg_turn_deg` | `90` | Heading change per leg |
| `leg_turn_rate_dps` | `35` | Turn rate during `LEG_TURN` |
| `altitude_layers_m` | `[3.5, 5.0, 6.5]` | NED hold altitudes (m above ground) |
| `altitude_layer_dwell_s` | `18` | Time at each layer before next |
| `vertical_depth_bias_*` | see config | Climb/descend from upper/lower depth clearance |
| `legacy_scan_enabled` | auto | `false` when panorama on; ±30° search if panorama off |

## Runtime behavior

- **Defer panorama** for ~2s after losing a target so pursuit is not interrupted.
- **`reset_panorama_timer`** on blue/red detection.
- **Legacy scan** (`legacy_scan_enabled`) overrides depth yaw with ±30° sweep; off by default when panorama is enabled.

## Reuse in other algorithms

Import `parse_exploration_settings`, `ExplorationScheduler`, and `apply_wander_move` from `src.control.exploration`. Copy the `exploration` block into the algorithm’s config section (e.g. future `yolo_explore`).
