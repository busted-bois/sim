# UE 5.4 + custom course (pending `course_sync` support)

Course swapping is intended to be a **deterministic script**, not an AI edit. Do **not** hand-edit `map_asset`. On branches that include `src/course_sync.py`, the launcher stages the chosen course's actors into the map for you. On this branch, treat custom-course support as pending until the prerequisite check below passes.

> **Prerequisite — the course system must be present on your branch.**
> This runbook uses `src/course_sync.py` and the launcher's `map=` override, which originated on the `babblewall` branch and is **not yet merged** into `main` / `sim_config_ai_skill`. Verify before relying on it:
>
> ```powershell
> Test-Path src\course_sync.py            # must be True
> Get-ChildItem course -Directory -Name   # lists candidate course tokens
> ```
>
> If `course_sync.py` is missing, the course system hasn't landed yet — that merge is a tracked dependency, not something to reinvent in this skill. Until then, only the maps already cooked into the uproject are available (see [`ue54-blocksv2-airsim.md`](ue54-blocksv2-airsim.md)).

## How it works

Every course reuses the **same** UE map, `FlyingExampleMapV2`. A "course" is just a set of External Actors (gates, obstacles) stored at `course/<token>/FlyingExampleMapV2/`. On launch, `course_sync.py` copies the chosen course's actors into the uproject's `Content/__ExternalActors__/FlyingCPP/Maps/FlyingExampleMapV2`, replacing whatever was there. So `map_name` / `map_asset` never change — only the actor content does. This is why a course swap is mechanical and needs no AI.

## Swap the course

```powershell
uv run sim map=<course>
```

Only run this command after the prerequisite check above passes.

- `map=` overrides `sim.config.json`; if repeated, last wins; `map=none` or empty = no staging (keep current).
- The token must match a folder under `course/` (matched case-insensitively) **and** that folder must be in the extracted layout `course/<token>/FlyingExampleMapV2/...`.

### Available course tokens (as of the `babblewall` work)

| token   | course                     | status                                                                 |
| ------- | -------------------------- | ---------------------------------------------------------------------- |
| `main`  | stock BlocksV2 (restore)   | ready — extracted layout                                               |
| `wall`  | wall course                | ready — extracted layout                                               |
| `UE5.4` | UE 5.4 square-gate course  | **not ready** — still `course/UE5.4/Content.zip`; extract to `course/UE5.4/FlyingExampleMapV2/` first |
| `UE416` | UE 4.16 course content     | **not ready** — still `Content.zip`; needs extraction                  |

A token only works if `course/<token>/FlyingExampleMapV2/` exists. Run `Get-ChildItem course -Directory -Name` for the live list on your checkout; confirm the subfolder with `Test-Path course\<token>\FlyingExampleMapV2`.

Examples:

```powershell
uv run sim map=wall    # wall course (ready)
uv run sim map=main    # restore stock BlocksV2
```

## Persisting the choice in config

To avoid passing `map=` every launch, set it in `sim.config.json` → `simulator.map`:

```json
{ "simulator": { "map": "wall" } }
```

`"none"` (the default) disables staging. A CLI `map=` arg overrides this for that run.

## MAVLink + custom course

`course_sync` runs on the `uv run sim` path. The staged actors are a real file copy that persists in the uproject, so for a MAVLink run on a custom course: stage once with `uv run sim map=<course>` (Ctrl+C once UE has loaded), then launch `uv run mavlink-all`. Both transports open the same `FlyingExampleMapV2`, now carrying the staged course. (Wiring `map=` directly into the MAVLink launch path is a possible follow-up — it is not there today.)

## Adding a new course

1. Export the course's External Actors and place them at `course/<name>/FlyingExampleMapV2/<actors>` (an extra nested `FlyingExampleMapV2/FlyingExampleMapV2/` level is also accepted — UE export quirk handled by `course_sync._flying_map_source_root`).
2. `uv run sim map=<name>` stages and launches it.

No new file in **this skill** is needed for a new course — the course content lives under `course/` at the repo root, and this one runbook covers every swap.

## gate_search_tokens

If the new course's gate actors use different names, update `sim.config.json` → `simulator.gate_search_tokens` (comma-separated substrings the runtime matches against scene actors via the AirSim scene-actor RPC), e.g. `"gate,ring,torus,hoop,square"`.

## Verification

- `uv run sim map=<course>` prints `Course map "<token>": replaced FlyingExampleMapV2 (N files) -> ...`.
- The UE viewport shows the new course geometry, not stock BlocksV2.
- `uv run sim map=main` restores stock and the geometry reverts — proves the swap is real.

## Common pitfalls

- **`map=<x>: no matching folder under course/`** — token doesn't match a `course/` subfolder. Run `Get-ChildItem course -Directory -Name` for valid tokens.
- **`Course map "<x>" has no FlyingExampleMapV2 folder`** — the token exists but is still a `Content.zip` (e.g. `UE5.4`, `UE416`). Extract it to `course/<x>/FlyingExampleMapV2/` first.
- **`course_sync safety: refuse to delete...`** — destination isn't the expected `FlyingExampleMapV2` folder inside the uproject; check `PROJECT_PATH` points at the real `.uproject`.
- **Course staged but UE still shows old geometry** — UE was already running; staging happens at launch, before UE opens. Restart UE.
- **`PROJECT_PATH is unset` warning, sync skipped** — set `PROJECT_PATH` in `.env.local` (see [`ue54-blocksv2-airsim.md`](ue54-blocksv2-airsim.md)).
