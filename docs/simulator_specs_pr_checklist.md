# Simulator snapshot PR checklist

Use when changing Colosseum maps, gate meshes, physics, or `docs/simulator_specs.json`.

- [ ] After `uv run extract-simulator-specs`, run `uv run verify-sim-physics-metadata` and `uv run verify-sim-gate-reference` (or `uv run preflight`).
- [ ] `gate_reference.dimensions_m` width × height is ~1.5 × 1.5 m (official opening), not ~5.5 m from an oversized level actor.
- [ ] If the map gate actor was rescaled, confirm extraction still picks the opening-sized mesh or fix actor scale in UE before committing the snapshot.
