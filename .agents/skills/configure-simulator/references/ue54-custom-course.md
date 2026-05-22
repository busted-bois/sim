# UE 5.4 + custom course (any map other than BlocksV2)

Use this when the user wants a course other than the default `BlocksV2` — for example a square-gate ring course, an obstacle course, or any custom map asset in the same Colosseum project.

This file covers **what changes** vs the default. For everything else — prereqs, launch, verification — follow the matching transport reference:

- Transport = `airsim` → also read [`ue54-blocksv2-airsim.md`](ue54-blocksv2-airsim.md)
- Transport = `mavlink` → also read [`ue54-blocksv2-mavlink.md`](ue54-blocksv2-mavlink.md)

## What the user needs to tell you

To configure a custom course you need **two pieces** of information:

1. **`map_name`** — the short name UE uses to identify the level (e.g. `SquareGateCourse`). Used by the launcher for diagnostics and trace overlays.
2. **`map_asset`** — the full UE asset path to the umap, of the form `/Game/<Folder>/Maps/<MapName>` (e.g. `/Game/FlyingCPP/Maps/SquareGateCourse`). This is what UE actually opens.

If the user gives you only one, ask for the other. Both must match what the `.uproject` contains. To list available maps:

```powershell
# from the .uproject's Content/ directory:
Get-ChildItem -Path . -Filter *.umap -Recurse | Select-Object -Expand FullName
```

## `sim.config.json` deltas

Edit `sim.config.json` → `"simulator"`:

```json
{
  "simulator": {
    "map_name": "SquareGateCourse",
    "map_asset": "/Game/FlyingCPP/Maps/SquareGateCourse"
  }
}
```

Optional but recommended for any gate-style course:

```json
{
  "simulator": {
    "gate_search_tokens": "gate,ring,torus,hoop,square"
  }
}
```

`gate_search_tokens` is a comma-separated list of substrings the runtime uses to identify gate actors in the UE world via AirSim's scene-actor RPC. Add the dominant geometry word for the course (e.g. `square`, `ring`).

Course-specific waypoints, if the user has them, go in the top-level `"waypoints"` array (NED, negative z = above ground):

```json
{
  "waypoints": [
    { "x": 5.0, "y": 0.0, "z": -3.0 },
    { "x": 10.0, "y": 5.0, "z": -3.0 }
  ]
}
```

## What stays the same

- `physics_update_hz: 120` and `specification_profile: official_conformant` — keep these.
- `airsim_port: 41451` — keep.
- The `~/Documents/AirSim/settings.json` shape is determined by **transport**, not by course. See [`airsim-settings.md`](airsim-settings.md).

## Launch

Same as the base transport reference:

- Transport = `airsim` → `uv run sim`
- Transport = `mavlink` → `uv run mavlink-all`

The launcher reads `simulator.map_name` / `simulator.map_asset` and passes them to UE as command-line args, so no extra step is needed.

## Verification

Standard transport-specific verification (see the base references). Course-specific:

- UE viewport shows the expected level geometry, not BlocksV2.
- For autonomous runs with vision, gates appear in the FPV feed; if `pursue_blue_rings` or `pursue_red_targets` is enabled in `sim.config.json` → `autonomous_explore`, the drone reacts to them.
- Trace overlay (`simulator.trace.enabled: true`) draws the flight path on the actual course terrain.

## Common pitfalls

- **`Failed to load map: /Game/.../SquareGateCourse`** — the asset path is wrong or the map isn't cooked. Open the project in the UE editor once to verify the path; the asset browser shows the canonical path under "Copy Reference".
- **Map loads but drone spawns inside geometry** — the `pawn_asset`'s default spawn transform doesn't match this course. Either move the player start in UE, or override the spawn pose via AirSim `settings.json`'s `Vehicles.<name>.X/Y/Z`.
- **`gate_search_tokens` finds nothing** — the actors in this course don't include any of the tokens in their names. Open the UE outliner and add a matching substring, or change the tokens to match what's there.
