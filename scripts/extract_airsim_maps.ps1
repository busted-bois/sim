param(
    [string]$RepoUrl = "https://github.com/xinglin-yu/AirsimSimulation.git",
    [string]$WorkspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$CloneDirName = "opensrc/AirsimSimulation"
)

$ErrorActionPreference = "Stop"

function Assert-Command([string]$Name) {
    if (-not (Get-Command $Name -ErrorAction SilentlyContinue)) {
        throw "Required command '$Name' was not found in PATH."
    }
}

Assert-Command "git"

$clonePath = Join-Path $WorkspaceRoot $CloneDirName
$cloneParent = Split-Path -Parent $clonePath
if (-not (Test-Path $cloneParent)) {
    New-Item -Path $cloneParent -ItemType Directory | Out-Null
}

if (-not (Test-Path $clonePath)) {
    Write-Host "Cloning AirsimSimulation into $clonePath ..."
    git clone --depth 1 $RepoUrl $clonePath
} else {
    Write-Host "Repo already exists at $clonePath; fetching latest master ..."
    git -C $clonePath fetch origin master --depth 1
    git -C $clonePath checkout master | Out-Null
    git -C $clonePath reset --hard origin/master | Out-Null
}

$mapGlobs = @(
    "Content/MazeCreator/Maps/*.umap",
    "Content/VehicleAdvCPP/Maps/*.umap",
    "Content/StarterContent/Maps/*.umap"
)

Write-Host ""
Write-Host "Discovered Unreal map packages:"
$foundAny = $false
foreach ($pattern in $mapGlobs) {
    $matches = Get-ChildItem -Path (Join-Path $clonePath $pattern) -File -ErrorAction SilentlyContinue
    foreach ($m in $matches) {
        $relative = $m.FullName.Substring($clonePath.Length + 1).Replace("\", "/")
        Write-Host " - $relative"
        $foundAny = $true
    }
}

if (-not $foundAny) {
    throw "No map packages were found. Verify the repo layout has not changed."
}

Write-Host ""
Write-Host "Primary maze migration roots:"
Write-Host " - Content/MazeCreator/Maps/MapMaze.umap"
Write-Host " - Content/MazeCreator/Maps/Map.umap"
Write-Host " - Content/MazeCreator/Blueprints/"
Write-Host " - Content/MazeCreator/Materials/"
Write-Host " - Content/MazeCreator/Meshes/"

Write-Host ""
Write-Host "Next Unreal Editor step:"
Write-Host " - Open the source project, right-click MapMaze.umap, then Asset Actions -> Migrate."
