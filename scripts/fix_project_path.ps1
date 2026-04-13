$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Push-Location $ProjectRoot

function Get-ProjectPathFromEnvFile {
    param([string]$Path)
    if (-not (Test-Path $Path)) { return $null }
    $line = Get-Content $Path | Where-Object { $_ -match '^\s*PROJECT_PATH\s*=' } | Select-Object -First 1
    if (-not $line) { return $null }
    return ($line -split "=", 2)[1].Trim()
}

function Set-ProjectPathInEnvFile {
    param([string]$Path, [string]$ProjectPath)
    $content = @("PROJECT_PATH=$ProjectPath")
    Set-Content -Path $Path -Value $content -Encoding UTF8
}

$envLocal = Join-Path $ProjectRoot ".env.local"
$current = Get-ProjectPathFromEnvFile -Path $envLocal
Write-Host "Current PROJECT_PATH: $current"
if ($current -and (Test-Path $current)) {
    Write-Host "Looks good (file exists). No changes made."
    Pop-Location
    exit 0
}

Write-Host "Current PROJECT_PATH is missing/invalid; searching for BlocksV2.uproject..."
$searchRoots = @(
    "E:\\source\\repos",
    (Join-Path $env:USERPROFILE "source\\repos"),
    "E:\\",
    "C:\\"
) | Where-Object { $_ -and (Test-Path $_) } | Select-Object -Unique

$found = $null
foreach ($root in $searchRoots) {
    try {
        $hit = Get-ChildItem -Path $root -Filter "BlocksV2.uproject" -File -Recurse -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($hit) {
            $found = $hit.FullName
            break
        }
    } catch {
        # ignore access denied and keep searching
    }
}

if (-not $found) {
    Write-Host "Could not find BlocksV2.uproject on this machine."
    Write-Host "Fix: set PROJECT_PATH in .env.local to the full path of your Colosseum .uproject."
    Pop-Location
    exit 1
}

Write-Host "Found: $found"
Set-ProjectPathInEnvFile -Path $envLocal -ProjectPath $found
Write-Host "Updated .env.local"

Pop-Location
