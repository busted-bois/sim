# AIGP Drone Challenge

Autonomous drone that navigates waypoints in the Colosseum (UE5) simulator. Built for the [AI Grand Prix](https://theaigrandprix.com) competition.

Uses MAVLink v2 over UDP for telemetry, a custom wire-protocol parser (no pymavlink runtime dependency), and threaded listeners for real-time state updates.

## Project Structure

```
├── main.py                       # Entry point — connects to simulator, prints telemetry
├── sim.config.json               # All runtime configuration
├── pyproject.toml                # Python project config (uv)
├── launch.bat                    # One-command launch (Windows)
├── scripts/
│   ├── launch.py                 # Cross-platform launch script
│   └── heartbeat_stress_test.py  # Telemetry reliability test
├── src/
│   ├── config.py                 # Config loader (plain json.load)
│   ├── comms/
│   │   ├── mavlink_parser.py     # MAVLink v2 wire protocol (encode/decode)
│   │   ├── telemetry.py          # Threaded UDP telemetry client
│   │   ├── state.py              # DroneState dataclass
│   │   └── mock_simulator.py     # Fake telemetry for development
│   ├── control/                  # Flight control (placeholder)
│   ├── perception/               # Computer vision (placeholder)
│   └── planning/
│       └── waypoint.py           # Waypoint following algorithm
├── packages/
│   └── simple_airsim/            # AirSim Python client (git submodule)
└── airsim/                       # Vendored AirSim client
```

## Configuration

All settings live in `sim.config.json`. The file is loaded at startup by `src/config.py` with plain `json.load()` (no schema validation).

### `telemetry`

UDP connection to the simulator's MAVLink endpoint.

| Field | Default | Description |
|---|---|---|
| `host` | `127.0.0.1` | Simulator IP address |
| `port` | `14550` | UDP port for telemetry stream |
| `timeout_seconds` | `1.0` | Receive timeout per packet |
| `heartbeat_interval_seconds` | `1.0` | How often heartbeats are sent |

### `simulator`

Colosseum / UE5 launch parameters.

| Field | Default | Description |
|---|---|---|
| `colosseum_path` | `C:\Program Files\...` | Path to UnrealEditor-Cmd.exe |
| `project_path` | `""` | Path to the UE5 project file |
| `map_name` | `BlocksV2` | Which map to load |
| `airsim_port` | `41451` | AirSim RPC port |
| `startup_delay_seconds` | `30` | Wait time before connecting (sim boot) |

### `control`

Flight parameters.

| Field | Default | Description |
|---|---|---|
| `command_rate_hz` | `50` | Control loop frequency (Hz) |
| `max_speed_ms` | `10.0` | Maximum ground speed (m/s) |
| `max_altitude_m` | `50.0` | Ceiling altitude (meters) |

### `waypoints`

Array of `{x, y, z}` coordinates in NED frame. NED means **N**orth **E**ast **D**own, so negative z values are above ground. Example:

```json
[
  {"x": 10.0, "y": 0.0, "z": -5.0},
  {"x": 10.0, "y": 10.0, "z": -5.0},
  {"x": 0.0, "y": 10.0, "z": -5.0},
  {"x": 0.0, "y": 0.0, "z": -5.0}
]
```

### `logging`

| Field | Default | Description |
|---|---|---|
| `level` | `INFO` | Python log level (`DEBUG`, `INFO`, `WARNING`, `ERROR`) |
| `telemetry_log_enabled` | `false` | Log every received telemetry packet |

## Quick Start

Requires Python 3.10+ and [uv](https://docs.astral.sh/uv/).

```bash
# Install dependencies
uv sync

# Run with mock simulator (no UE5 needed)
uv run python -m src.comms.mock_simulator &
uv run python main.py

# Run with real simulator (Windows)
launch.bat
```

## Development Notes

**Coordinate frame:** NED (North-East-Down). X points north, Y points east, Z points down. A drone hovering at 5 meters has `z = -5.0`.

**Protocol:** MAVLink v2 over UDP port 14550. The parser in `src/comms/mavlink_parser.py` handles wire encoding and decoding with raw struct packing. No pymavlink dependency.

**Telemetry:** Threaded UDP listener (not asyncio). The client runs a background thread that continuously receives and parses MAVLink messages, updating `DroneState` in real time.

**Dependencies:** Managed with `uv`. See `pyproject.toml`. No pymavlink, matplotlib, or pytest in the dep list.

**Team**

Advisors: Aneesh Saxena, Tushar Shrivastav

Simulation & Network Communication: Kunal Shrivastav, Samyak Kakatur, Yat Chun Wong, Ryan Yang, Trung Nguyen

Algorithms & Autonomy: Kunal Shrivastav, Samyak Kakatur, David Vayntrub, Ram Rao
