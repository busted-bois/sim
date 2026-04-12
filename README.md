# AIGP Drone Challenge

Autonomous drone navigating the Colosseum (UE5/AirSim) simulator for the [AI Grand Prix](https://theaigrandprix.com).

## Quick Start

**Prerequisites:** Python 3.10+, [uv](https://docs.astral.sh/uv/)

```bash
pwsh scripts/install.ps1         # install deps (or bash scripts/install.sh)
cp .env.local.example .env.local # set PROJECT_PATH to your UE5 install
pwsh scripts/launch.ps1          # launch simulator + drone (or bash scripts/launch.sh)
```

## Project Structure

```
main.py                          # Entry point — AirSim RPC client
sim.config.json                  # Runtime config
src/
  config.py                      # Config loader
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
| `control`   | `command_rate_hz` (50), `max_speed_ms` (10), `max_altitude_m` (50) |
| `waypoints` | NED coordinate waypoints                                           |
| `logging`   | Log level, telemetry logging toggle                                |

**`.env.local`** — `PROJECT_PATH` pointing to your UE5 Colosseum project.

**Coordinate frame:** NED (North-East-Down). Negative z = above ground. Drone at 5m altitude → `z = -5.0`.

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
