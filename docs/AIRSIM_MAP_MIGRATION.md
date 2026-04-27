# AirsimSimulation Map Extraction and Integration

Use this guide with the upstream maze project:

- `https://github.com/xinglin-yu/AirsimSimulation.git`

## 1) Deploy maze files and config (recommended)

From repository root, this refreshes the GitHub clone **and** writes `simulator.maze_project_path` plus `simulator.maze_colosseum_path` when UE 4.16 is auto-detected:

```bash
powershell -ExecutionPolicy Bypass -File ".\scripts\deploy_maze_sim.ps1"
```

`sim.config.json` ships with the usual Epic path to UE 4.16 (`UE_4.16\\Engine\\Binaries\\Win64\\UnrealEditor.exe`). If your install differs, run the deploy script (it searches `EPIC_GAMES_PATH` and `Program Files\\Epic Games`) or edit `maze_colosseum_path` manually.

### Extract only (no config write)

```bash
powershell -ExecutionPolicy Bypass -File ".\scripts\extract_airsim_maps.ps1"
```

This clones or refreshes `opensrc/AirsimSimulation` and prints discovered `.umap` files.

## 2) Launch the maze with UE 4.16 (`uv run sim maze`)

The maze lives in **`VehilceAdvanced.uproject`** (engine association UE 4.16). `uv run sim maze` always uses **UE 4.16** + that maze project only — it does **not** fall back to `simulator.colosseum_path` or `PROJECT_PATH` (BlocksV2). Normal `uv run sim` is unchanged.

Paths used for maze runs (separate from Colosseum):

| Setting | Purpose |
| ------- | ------- |
| `simulator.maze_colosseum_path` | Full path to **UE 4.16** `UnrealEditor.exe` (optional if auto-detect works) |
| `simulator.maze_project_path` | Absolute path, or path **relative to this repo root**, to `VehilceAdvanced.uproject` |

Default in repo config is `opensrc/AirsimSimulation/VehilceAdvanced.uproject` after running the extract script.

If `maze_project_path` is empty, the launcher still tries  
`opensrc/AirsimSimulation/VehilceAdvanced.uproject` when that file exists.

If `maze_colosseum_path` is missing or the path does not exist on disk, the launcher resolves **UE 4.16** from Epic’s **`%ProgramData%\Epic\UnrealEngineLauncher\LauncherInstalled.dat`**, then `EPIC_GAMES_PATH`, `Program Files\Epic Games` on **C:** and extra drives **D: through Z:**, and any engine folder whose name contains **`4.16`**. It looks for **`UnrealEditor.exe`** or **`UE4Editor.exe`** under `Engine\Binaries\Win64\` (UE 4.x typically uses `UE4Editor.exe`).

Environment overrides (optional):

- `MAZE_COLLOSSEUM_PATH` — same as `maze_colosseum_path`
- `MAZE_PROJECT_PATH` — same as `maze_project_path`

Example `sim.config.json` fragment:

```json
"maze_colosseum_path": "C:\\Program Files\\Epic Games\\UE_4.16\\Engine\\Binaries\\Win64\\UnrealEditor.exe",
"maze_project_path": "opensrc/AirsimSimulation/VehilceAdvanced.uproject"
```

Then:

```bash
uv run sim maze
```

The launcher prints the resolved `UnrealEditor` and `.uproject`, then the on-disk paths checked for `MapMaze.umap` / `Map.umap`.

## 3) Migrate maze into Colosseum / BlocksV2 (optional)

If you prefer UE 5.4 + BlocksV2 instead of opening the old project:

1. Open the source project `VehilceAdvanced.uproject` in a UE version that can read it.
2. In Content Browser, go to `Content/MazeCreator/Maps/`.
3. Right-click `MapMaze.umap` and choose `Asset Actions -> Migrate`.
4. Select your target Unreal project's `Content/` directory.
5. In target project, open the map once, then run `Fix Up Redirectors` on `Content/MazeCreator`.

Do not manually copy `.umap/.uasset` files between projects; migration preserves dependencies.

After migration, you can use `simulator.extra_ue_args` with `-map=/Game/MazeCreator/Maps/MapMaze` on the **same** project as `PROJECT_PATH`, or keep using `uv run sim maze` with `maze_project_path` pointing at the project that contains the maze.

## 4) Known compatibility risks

- Source project targets UE 4.16 with AirSim v1.1.8.
- The Python AirSim client in this repo may not match that server version; RPC or API errors are possible when pointing at the old sim.
- If direct migration fails, first open and resave in an intermediate UE4 version before moving to UE5.
