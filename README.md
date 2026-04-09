# AIGP Drone Challenge

Autonomous drone that navigates waypoints in the Colosseum (UE5) simulator. Built for the [AI Grand Prix](https://theaigrandprix.com) competition.

Uses MAVLink v2 over UDP for telemetry, a custom wire-protocol parser (no pymavlink runtime dependency), and threaded listeners for real-time state updates.

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
├── main.py                       # Entry point
├── sim.config.json               # Runtime configuration
├── pyproject.toml                # Python project config (uv)
├── Makefile                      # make install / make start
├── scripts/
│   ├── install.sh                # Setup venv + git hooks
│   ├── launch.bat                # One-command launch (Windows)
│   ├── launch.py                 # Cross-platform launch script
│   └── heartbeat_stress_test.py  # Telemetry reliability test
├── src/
│   ├── config.py                 # Config loader
│   ├── comms/
│   │   ├── mavlink_parser.py     # MAVLink v2 wire protocol
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

All settings live in `sim.config.json`, loaded at startup by `src/config.py`.

### `telemetry`

| Field | Default | Description |
|---|---|---|
| `host` | `127.0.0.1` | Simulator IP address |
| `port` | `14550` | UDP port for telemetry stream |
| `timeout_seconds` | `1.0` | Receive timeout per packet |
| `heartbeat_interval_seconds` | `1.0` | How often heartbeats are sent |

### `simulator`

| Field | Default | Description |
|---|---|---|
| `colosseum_path` | `C:\Program Files\...` | Path to UnrealEditor-Cmd.exe |
| `project_path` | `""` | Path to the UE5 project file |
| `map_name` | `BlocksV2` | Which map to load |
| `airsim_port` | `41451` | AirSim RPC port |
| `startup_delay_seconds` | `30` | Wait time before connecting |

### `control`

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
make install

# Run with real simulator (Windows)
make start
```

## Development Notes

**Coordinate frame:** NED (North-East-Down). A drone at 5m altitude has `z = -5.0`.

**Protocol:** MAVLink v2 over UDP. Custom parser in `src/comms/mavlink_parser.py` — no pymavlink dependency.

**Telemetry:** Threaded UDP listener (not asyncio). The client runs a background thread that continuously receives and parses MAVLink messages, updating `DroneState` in real time.

**Dependencies:** Managed with `uv`. See `pyproject.toml`. No pymavlink, matplotlib, or pytest in the dep list.
