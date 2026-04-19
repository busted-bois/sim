# AGENTS.md

## Tooling

- **Python:** `uv` for all operations (`uv run`, `uv sync`). No bare `python3`.
- **Linter:** Ruff (`uvx ruff check --fix`). Runs via lefthook pre-commit on staged `*.{py,toml}`.
- **Python version:** 3.12 (`.python-version`). `requires-python >= 3.10`.

## Running

```bash
uv run sim                              # One command: .env.local, UE5 (if configured), main.py
uv run main.py                          # Run drone client (needs simulator running)
uv run scripts/launch.py                # Same as uv run sim
bash scripts/launch.sh                  # Shell loads .env.local then runs launch.py
```

After Unreal starts (or if launch is skipped), the launcher **autostarts** `main.py` once AirSim RPC accepts connections on the configured host/port, up to `simulator.rpc_ready_timeout_seconds`.

**Ctrl+C** (SIGINT) while `uv run sim` is running stops the drone client (`main.py`) and, if this launcher started Unreal, terminates that editor process as well. **SIGTERM** (e.g. some IDE Stop actions) is not wired to that cleanup, so Unreal may stay open unless you stop it yourself.

## Project Layout

- `main.py` — Entry point. Connects to AirSim RPC, loads algorithm from config, runs it.
- `sim.config.json` — Runtime config (algorithm name, sim ports, waypoints, control limits).
- `.env.local` — `PROJECT_PATH` to UE5 project. Loaded by `uv run sim` / `launch.sh` / `launch.ps1`. Not committed.
- `src/config.py` — Reads `sim.config.json` from project root.
- `src/control/algorithms/` — Pluggable flight algorithms via `@register("name")` decorator.
- `airsim/` — Vendored AirSim Python RPC client. **Do not modify.**
- `msgpackrpc/` — Custom msgpack-rpc shim for Python 3.12 compat. **Do not modify.**
- `opensrc/` — Gitignored external sources. Not part of the project.
- `scripts/` — Install (`install.sh`) and launch (`launch.sh`, `launch.py`) scripts.

## Adding a New Algorithm

1. Create `src/control/algorithms/my_algo.py`
2. Extend `Algorithm`, implement `run(self, client: airsim.MultirotorClient)`
3. Decorate with `@register("my_algo")`
4. Set `"algorithm": "my_algo"` in `sim.config.json`
5. New module must be imported at bottom of `algorithms/__init__.py` to trigger registration

## Coordinate Frame

**NED** (North-East-Down). Negative z = above ground. Drone at 5m altitude → `z = -5.0`.
