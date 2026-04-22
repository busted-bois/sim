# Branch Testing Notes

Quick checks for this branch's dual backend support (`airsim` and `mavlink`).

## 1) Sync deps

```bash
uv sync
```

## 2) AirSim baseline (quick health check)

In `sim.config.json`:

```json
"backend": "airsim"
```

Run:

```bash
uv run sim
```

Expected:
- Unreal launches
- terminal shows `Connected!`
- algorithm runs and lands

## 3) MAVLink smoke test

In `sim.config.json`:

```json
"backend": "mavlink",
"mavlink": {
  "connection": "udp:127.0.0.1:14550",
  "source_system": 255,
  "source_component": 0
}
```

Start your MAVLink endpoint first (PX4 SITL / ArduPilot SITL), then run:

```bash
uv run python main.py
```

Expected:
- `Connected! MAVLink target=...`
- arm/takeoff/algorithm activity

## Common gotchas

- No endpoint on `udp:127.0.0.1:14550` -> `MAVLink heartbeat timeout (15s)`.
- Use `uv run python main.py` for MAVLink branch tests (explicit Python entry).
- `uv run sim` is still the best baseline check for the AirSim path.
