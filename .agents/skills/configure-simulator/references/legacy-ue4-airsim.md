# Legacy UE 4.x + upstream AirSim (NOT SUPPORTED HERE)

**Read this before promising any UE 4.x setup.** This codebase targets UE 5.4 / Colosseum exclusively. None of the launchers (`uv run sim`, `uv run mavlink-all`, etc.) are tested against UE 4.x. The repo's vendored `airsim/` Python RPC client is shimmed for Python 3.12 compatibility against the Colosseum fork, not stock UE 4.27 AirSim.

If the user is asking for UE 4.x explicitly, surface this before going further:

> "This repo targets UE 5.4 + Colosseum. UE 4.x AirSim isn't on the supported runtime path here. Do you want to switch to UE 5.4, or are you trying to use this project's algorithms against an external UE 4.x install?"

## If the user still wants UE 4.x guidance

For reference only — point them at the upstream projects, do not try to wire them into this repo's launchers:

| Version    | Project                                                                | Status                                |
| ---------- | ---------------------------------------------------------------------- | ------------------------------------- |
| UE 4.27    | [microsoft/AirSim](https://github.com/microsoft/AirSim)                | Archived 2022, last stable AirSim release. |
| UE 4.16    | Never an AirSim release target. Earliest supported was UE 4.18 / 4.20. | Mention only as historical curiosity. |
| UE 5.4     | [CodexLabsLLC/Colosseum](https://github.com/CodexLabsLLC/Colosseum)    | What this repo uses.                  |

Practical guidance for an external UE 4.27 AirSim install:

1. Build the upstream AirSim UE plugin against UE 4.27 from source.
2. Generate a fresh `settings.json` from upstream AirSim — the shape is similar to [`airsim-settings.md`](airsim-settings.md), but `SettingsVersion`, vehicle types, and supported camera params differ slightly. Do not copy this repo's `settings.json` verbatim.
3. Use upstream AirSim's own Python client (`pip install airsim`) — **not** this repo's vendored `airsim/` directory.
4. Do not call `uv run sim` or any other launcher from this repo against UE 4.x. The launchers assume Colosseum-specific UE command-line args and will misconfigure UE 4.x at best.

## What to recommend instead

For 95% of cases where someone says "UE 4", the right answer is "use UE 5.4 + Colosseum". Walk them through [`ue54-blocksv2-airsim.md`](ue54-blocksv2-airsim.md) — same conceptual workflow, supported by the project.

## What you should not do

- Do not write a `sim.config.json` profile that claims UE 4.x compatibility.
- Do not edit the vendored `airsim/` directory to support UE 4.x — it's marked `Do not modify.` in `AGENTS.md`.
- Do not add a UE-4-targeting matrix row to the parent [`SKILL.md`](../SKILL.md) without explicit user direction. The row that's there now points back to this file precisely to keep agents from inventing a path.
