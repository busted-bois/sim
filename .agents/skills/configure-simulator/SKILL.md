---
name: configure-simulator
description: Configure the AIGP drone simulator end-to-end for a chosen combination of Unreal Engine version, course/map, and control transport (AirSim RPC vs MAVLink/PX4). Use when the user says things like "set up the simulator", "configure UE5 + MAVLink", "switch to PX4 mode", "use the square gate course", "swap to AirSim transport", "I need the simulator running with X". Returns exact step-by-step commands and the precise `sim.config.json` / `~/Documents/AirSim/settings.json` deltas to apply. Does not cover the one-time PX4 build inside WSL — see the `setup-px4-mavlink-bridge` skill for that.
metadata:
  applies-to: ai-grand-prix_drone-challenge
  combos-covered: ue5.4+blocksv2+airsim, ue5.4+blocksv2+mavlink, ue5.4+custom-course+airsim, ue5.4+custom-course+mavlink
---

# Configure Simulator

This skill gets the simulator into a specific, reproducible configuration. **Every supported combination of `(engine, course, transport)` resolves to one reference runbook** in [`references/`](references/). Load only the reference you need.

## How to use this skill

1. Decide the three dimensions below from the user's request. If any are missing, ask the user before reading a reference — guessing wastes context.
2. Resolve the combination to one reference file using the matrix.
3. Read that one reference and execute it.
4. If the chosen transport is `mavlink` and PX4 has never been built on this machine, run the [`setup-px4-mavlink-bridge`](../setup-px4-mavlink-bridge/SKILL.md) skill first.

## The three dimensions

| Dimension     | Values                                                    | Default in this repo                    |
| ------------- | --------------------------------------------------------- | --------------------------------------- |
| **engine**    | `ue5.4` (Colosseum fork), `ue4.27` (legacy upstream AirSim) | `ue5.4`                                 |
| **course**    | `blocksv2`, `custom` (any other UE map asset)             | `blocksv2`                              |
| **transport** | `airsim` (SimpleFlight RPC), `mavlink` (PX4-SITL)         | `airsim`                                |

`engine=ue4.27` and `engine=ue4.16` exist only as historical pointers — this codebase targets UE 5.4. See [`references/legacy-ue4-airsim.md`](references/legacy-ue4-airsim.md) before promising any UE 4.x setup.

## Combination → reference matrix

| engine  | course   | transport | Reference file                                                  | One-liner to run after setup     |
| ------- | -------- | --------- | --------------------------------------------------------------- | -------------------------------- |
| ue5.4   | blocksv2 | airsim    | [`references/ue54-blocksv2-airsim.md`](references/ue54-blocksv2-airsim.md)   | `uv run sim`                     |
| ue5.4   | blocksv2 | mavlink   | [`references/ue54-blocksv2-mavlink.md`](references/ue54-blocksv2-mavlink.md) | `uv run mavlink-all`             |
| ue5.4   | custom   | airsim    | [`references/ue54-custom-course.md`](references/ue54-custom-course.md)   | `uv run sim map=<course>`        |
| ue5.4   | custom   | mavlink   | [`references/ue54-custom-course.md`](references/ue54-custom-course.md)   | stage with `uv run sim map=<course>`, then `uv run mavlink-all` |
| ue4.27  | any      | airsim    | [`references/legacy-ue4-airsim.md`](references/legacy-ue4-airsim.md)         | not supported by this repo       |
| ue4.16  | any      | any       | [`references/legacy-ue4-airsim.md`](references/legacy-ue4-airsim.md)         | not supported by this repo       |

Cross-cutting references (load when relevant):

- [`references/airsim-settings.md`](references/airsim-settings.md) — exact `~/Documents/AirSim/settings.json` shapes for SimpleFlight vs PX4Multirotor.
- [`references/troubleshooting.md`](references/troubleshooting.md) — common symptom → cause → fix table.

## Why one skill, not one per combo

Each combination is the same four-step recipe (prereqs → `sim.config.json` deltas → `settings.json` deltas → launch command) with different values. A single skill keeps the routing logic in one place and lets agents discover any supported combo from a single `description` keyword set. The per-combo specifics live in `references/` and load only on demand — same context cost as separate skills, less drift.

## Where new files go (course / combo / settings)

All bundled content for this skill lives under `.agents/skills/configure-simulator/references/`. Convention:

| What you're adding                                                | Where it goes                                                          |
| ----------------------------------------------------------------- | ---------------------------------------------------------------------- |
| A new course (gates/obstacles for the existing map)               | **No new skill file.** Drop the actors at `course/<name>/FlyingExampleMapV2/` (repo root) and swap with `uv run sim map=<name>`. See [`references/ue54-custom-course.md`](references/ue54-custom-course.md). |
| A course needing unique *launch* steps (custom physics, lighting) | `references/courses/<course-name>.md`. Add a row to the matrix above pointing at it. |
| A new transport (e.g. ROS 2 bridge)                               | `references/transports/<transport>.md`. Add a matrix column.            |
| A new engine version with a different launcher                    | `references/<engine-tag>-<course>-<transport>.md`. Add matrix rows.    |
| Cross-cutting reference (e.g. `settings.json` shapes)             | `references/<topic>.md` at the top level of `references/`.             |

**Do not fork this SKILL.md per combo.** The router stays singular; new combos = new rows + new reference files only.

Course content (the External Actors that define each course) lives under `course/<token>/` at the repo root, **not** in this skill — `src/course_sync.py` stages it into the uproject's `FlyingExampleMapV2` on launch. The skill only stores the **instructions** for swapping; the swap itself is the deterministic `uv run sim map=<course>` script, not an AI edit. See [`references/ue54-custom-course.md`](references/ue54-custom-course.md).

If a new combination genuinely needs different prereqs (e.g. a new engine version with a different launcher), add a row to the matrix above and drop a new file in `references/`. Do **not** fork this skill.

## Argument parsing

The user rarely names all three dimensions explicitly. Map natural phrases:

- "MAVLink mode", "PX4 mode", "use the PX4 bridge" → `transport=mavlink`
- "regular sim", "default sim", "SimpleFlight", "RPC mode" → `transport=airsim`
- "BlocksV2", "the default map", "current map", "stock" → `course=blocksv2` (i.e. `map=main`)
- "square gate", "ring course", "wall course", "custom map" → `course=custom`. Resolve to the matching `course/<token>` name and swap with `uv run sim map=<token>` — do **not** hand-edit `map_asset`. See [`references/ue54-custom-course.md`](references/ue54-custom-course.md).
- "UE5", "Unreal 5", "Colosseum" → `engine=ue5.4`
- "UE4", "legacy AirSim", "old Unreal" → `engine=ue4.27` (and warn it's unsupported here)

If the user only said "set up the simulator", default to `(ue5.4, blocksv2, airsim)` and confirm before running anything destructive (writing `settings.json`, launching UE).

## Verification

After running any combo's launch command, verify success per its reference. Generic checks that apply everywhere:

- `uv run preflight` — sanity check before flight.
- `uv run verify-sim-physics-metadata` — confirms 120 Hz physics is recorded in `docs/simulator_specs.json`.
- For MAVLink combos: `uv run check-mavlink --duration 20` should show non-zero MAVLink packets and (if `--decode-attitude`) non-zero ATTITUDE decodes.

Do not declare success on log lines alone — confirm the drone actually appears in the Unreal viewport and (for autonomous runs) `main.py` prints `Connected!` followed by phase logs.
