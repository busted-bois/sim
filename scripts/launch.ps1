$ProjectRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Push-Location $ProjectRoot

Write-Host "AIGP Drone Challenge - Launch Script"
Write-Host "====================================="
uv run scripts/launch.py @args

Pop-Location
