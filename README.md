# AIGP Drone Challenge

Autonomous drone navigating the Colosseum (UE5/AirSim) simulator for the [AI Grand Prix](https://theaigrandprix.com).
AirSim Python RPC client for drone control, pluggable flight algorithms.

## Quick Start

**Prerequisites:** Python 3.10+, [uv](https://docs.astral.sh/uv/)

```bash
# Install
bash scripts/install.sh          # or pwsh scripts/install.ps1

# Configure — copy and edit
cp .env.local.example .env.local # set PROJECT_PATH to your UE5 install

# Launch simulator + drone
bash scripts/launch.sh           # or pwsh scripts/launch.ps1
```

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

---

## Project Structure

```
main.py                          # Entry point — AirSim RPC client
sim.config.json                  # Runtime config
src/
  comms/                         # MAVLink code (for future PX4 use)
    mavlink_parser.py            # MAVLink v2 encode/decode (no pymavlink)
    telemetry.py                 # Threaded UDP telemetry listener
    state.py                     # DroneState dataclass (NED frame)
    command.py                   # Velocity command sender (SET_POSITION_TARGET_LOCAL_NED)
  control/
    algorithms/                  # ← custom algorithms go here
      __init__.py                # Algorithm base class + registry
      six_directions.py          # Default: 6-direction test pattern
  planning/
    waypoint.py                  # WaypointFollower + VelocityCommand
  config.py                      # Config loader
airsim/                          # Vendored AirSim Python client
scripts/                         # Install/launch scripts
```

## Writing a Custom Algorithm

1. Create `src/control/algorithms/my_algo.py`
2. Extend `Algorithm`, implement `run(client)` receiving an `airsim.MultirotorClient`
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

`run()` receives the AirSim `MultirotorClient` and has full control over the flight. Use `moveByVelocityAsync(vx, vy, vz, duration).join()` for timed velocity commands (NED frame).

## Configuration

**`sim.config.json`** top-level sections:

| Section | Purpose |
|---|---|
| `algorithm` | Name of registered algorithm to run |
| `simulator` | UE5/AirSim paths and ports (`airsim_port` for RPC) |
| `control` | `command_rate_hz`, `max_speed_ms`, `max_altitude_m` |
| `waypoints` | NED coordinate waypoints |
| `logging` | Log level, telemetry logging toggle |

**`.env.local`** — `PROJECT_PATH` pointing to your UE5 Colosseum project.

Coordinate frame: **NED** (North-East-Down). Negative z = above ground. Drone at 5m altitude → `z = -5.0`.

## Key Modules

- **airsim/** — Vendored AirSim Python RPC client (`msgpackrpc` on port 41451)
- **control/algorithms/** — Pluggable flight algorithms via decorator registry
- **comms/** — MAVLink code retained for future PX4 SITL integration

| Field | Default | Description |
|---|---|---|
| `command_rate_hz` | `50` | Control loop frequency (Hz) |
| `max_speed_ms` | `10.0` | Maximum ground speed (m/s) |
| `max_altitude_m` | `50.0` | Ceiling altitude (meters) |

### `waypoints`

Array of `{x, y, z}` coordinates in NED frame (**N**orth **E**ast **D**own — negative z is above ground).

### `logging`

| Field | Default | Description |
|---|---|---|
| `level` | `INFO` | Python log level |
| `telemetry_log_enabled` | `false` | Log every received telemetry packet |

## Prerequisites

- Python 3.10+
- [uv](https://docs.astral.sh/uv/)

## Quick Start

```bash
# Install dependencies and git hooks
pwsh scripts/install.ps1        # PowerShell
bash scripts/install.sh         # Bash

# Launch simulator and run main.py
pwsh scripts/launch.ps1         # PowerShell
bash scripts/launch.sh          # Bash
```

## Development Notes

**Coordinate frame:** NED (North-East-Down). A drone at 5m altitude has `z = -5.0`.

**Protocol:** MAVLink v2 over UDP. Custom parser in `src/comms/mavlink_parser.py` — no pymavlink dependency.

**Telemetry:** Threaded UDP listener (not asyncio). The client runs a background thread that continuously receives and parses MAVLink messages, updating `DroneState` in real time.

**Dependencies:** Managed with `uv`. See `pyproject.toml`. No pymavlink, matplotlib, or pytest in the dep list.
