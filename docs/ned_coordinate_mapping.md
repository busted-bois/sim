# NED coordinate mapping

Internal mapping for MAVLink NED frames used by flight algorithms. Spec reference: [mavlink.io common messages](https://mavlink.io/en/messages/common.html).

## Frames

| Frame | `MAV_FRAME` | Origin | Axes |
|-------|-------------|--------|------|
| **LOCAL_NED** | 1 | Fixed ground point (typically arm position) | x north, y east, z down |
| **BODY_NED** | 8 | Vehicle center | x forward, y right, z down |

Altitude above ground is **negative z** (e.g. 5 m AGL → `z = -5.0`).

## MAVLink messages

| Message | ID | Role |
|---------|-----|------|
| `LOCAL_POSITION_NED` | 32 | Filtered position (m) and velocity (m/s) in LOCAL_NED |
| `ATTITUDE` | 30 | `roll`, `pitch`, `yaw` (rad); yaw is heading in NED |
| `SET_POSITION_TARGET_LOCAL_NED` | 84 | Outbound setpoints; `coordinate_frame` selects LOCAL vs BODY |

## API

- Module: [`src/control/ned_environment.py`](../src/control/ned_environment.py)
- Class: `NedEnvironmentMap` — ingest telemetry, `snapshot()`, `transform_vector()`, spawn-relative XY
- MAVLink client: `PymavlinkFlightClient.get_ned_environment()`
- Algorithms: `Algorithm.ned_environment(client)` on [`Algorithm`](../src/control/algorithms/__init__.py)

### Body → local (yaw-only default)

For horizontal vectors at yaw ψ:

- `v_n = v_xb·cos(ψ) − v_yb·sin(ψ)`
- `v_e = v_xb·sin(ψ) + v_yb·cos(ψ)`

Set `control.mavlink.ned_environment.use_full_attitude: true` for full roll/pitch/yaw rotation.

### Config (`sim.config.json`)

```json
"control": {
  "mavlink": {
    "attitude": {
      "enabled": true,
      "require_stream": false
    },
    "ned_environment": {
      "enabled": true,
      "use_full_attitude": false,
      "spawn_relative_enabled": true,
      "max_position_staleness_ms": 500,
      "max_attitude_staleness_ms": 500
    }
  }
}
```

### Health and export

- `NedEnvironmentMap.get_health()` — position/attitude freshness (for algorithms and preflight).
- `export_snapshot_json(path)` — debug dump under `logs/` when needed.
- `plan_body_velocity(mapper, vx, vy, vz)` — BODY command with LOCAL equivalent.

AirSim (`AirSimAdapter.get_ned_environment()`) refreshes from RPC each call so SimpleFlight and MAVLink share the same API.

Exploration SLAM grids use spawn-relative LOCAL NED XY; see [`exploration.md`](exploration.md).
