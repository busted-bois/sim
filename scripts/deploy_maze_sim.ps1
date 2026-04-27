param(
    [string]$WorkspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

Write-Host "== deploy_maze_sim: refresh maze repo =="
& (Join-Path $PSScriptRoot "extract_airsim_maps.ps1") -WorkspaceRoot $WorkspaceRoot

$mazeUproject = Join-Path $WorkspaceRoot "opensrc\AirsimSimulation\VehilceAdvanced.uproject"
if (-not (Test-Path $mazeUproject)) {
    throw "Missing $mazeUproject after extract. Check git/network and try again."
}
$mapMaze = Join-Path $WorkspaceRoot "opensrc\AirsimSimulation\Content\MazeCreator\Maps\MapMaze.umap"
if (-not (Test-Path $mapMaze)) {
    Write-Warning "MapMaze.umap not found at $mapMaze"
}

$configPath = Join-Path $WorkspaceRoot "sim.config.json"
$patchScript = Join-Path $PSScriptRoot "patch_maze_config.py"
Push-Location $WorkspaceRoot
try {
    $editor = (
        uv run python -c "from src.sim_launch import _guess_ue_416_editor; print(_guess_ue_416_editor() or '')"
    ).Trim()
    if (-not $editor) {
        Write-Warning "UE 4.16 UnrealEditor.exe not found (LauncherInstalled.dat, Epic folders, EPIC_GAMES_PATH)."
        Write-Warning "Install UE 4.16 from Epic Launcher, then re-run, or set simulator.maze_colosseum_path manually."
    } else {
        Write-Host "== deploy_maze_sim: found UE 4.16 at $editor"
    }
    if ($editor) {
        uv run python $patchScript --config $configPath --editor $editor
    } else {
        uv run python $patchScript --config $configPath
    }
} finally {
    Pop-Location
}

Write-Host "== deploy_maze_sim: updated $configPath"
Write-Host "Next: uv run sim maze"
