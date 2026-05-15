#requires -Version 5.1
<#
.SYNOPSIS
    One-command MAVLink dev session: launches Unreal Engine, PX4-SITL (in WSL),
    and the check-mavlink probe. Saves all logs to logs/mavlink/<timestamp>/.

.DESCRIPTION
    Requires WSL2 mirrored networking mode so traffic stays on loopback (no
    firewall rule needed). Verifies prerequisites, then runs three coordinated
    processes; Ctrl+C tears them all down cleanly and restores SimpleFlight
    settings.json from backup.

.PARAMETER EnableMirrored
    Write %USERPROFILE%\.wslconfig with mirrored networking and exit. You then
    run `wsl --shutdown` and re-run this script.

.PARAMETER SkipMirroredCheck
    Skip the .wslconfig check. Use only if you have mirrored mode set up via
    a non-standard path.

.PARAMETER WslDistro
    WSL distro name (default: WSL's default distro).

.EXAMPLE
    pwsh scripts/dev-mavlink.ps1
    uv run mavlink-all
#>
[CmdletBinding()]
param(
    [switch]$EnableMirrored,
    [switch]$SkipMirroredCheck,
    [string]$WslDistro = ""
)

$ErrorActionPreference = 'Stop'
$RepoRoot = Split-Path $PSScriptRoot -Parent
$WslArgs = @()
if ($WslDistro) { $WslArgs = @("-d", $WslDistro) }

# --------------------------------------------------------------------------
# Mirrored mode handling
# --------------------------------------------------------------------------
function Test-MirroredMode {
    $cfg = "$env:USERPROFILE\.wslconfig"
    if (-not (Test-Path $cfg)) { return $false }
    $text = Get-Content $cfg -Raw
    return $text -match '(?im)^\s*networkingMode\s*=\s*mirrored\s*$'
}

if ($EnableMirrored) {
    $cfg = "$env:USERPROFILE\.wslconfig"
    if (Test-Path $cfg) {
        Write-Host "[ORCH] $cfg already exists. Refusing to overwrite. Edit it manually:"
        Write-Host "       Add under [wsl2]:  networkingMode=mirrored"
        exit 1
    }
    Set-Content -Path $cfg -Encoding ascii -Value "[wsl2]`r`nnetworkingMode=mirrored"
    Write-Host "[ORCH] Wrote $cfg with mirrored networking."
    Write-Host "[ORCH] Now run:  wsl --shutdown"
    Write-Host "[ORCH] Then re-run this script."
    exit 0
}

if (-not $SkipMirroredCheck -and -not (Test-MirroredMode)) {
    Write-Host "[ORCH] WSL mirrored networking is NOT enabled in $env:USERPROFILE\.wslconfig."
    Write-Host "[ORCH] Required so PX4 in WSL can reach AirSim on Windows via 127.0.0.1."
    Write-Host "[ORCH] Add to $env:USERPROFILE\.wslconfig :"
    Write-Host "         [wsl2]"
    Write-Host "         networkingMode=mirrored"
    Write-Host "[ORCH] Then run: wsl --shutdown"
    Write-Host "[ORCH] Or rerun with -EnableMirrored to write the file for you."
    exit 1
}

# --------------------------------------------------------------------------
# Prerequisites
# --------------------------------------------------------------------------
Write-Host "[ORCH] Checking WSL is up..."
& wsl @WslArgs -e true 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "[ORCH] FAIL: 'wsl -e true' did not succeed. Is WSL installed?"
    exit 1
}

$px4BinCheck = & wsl @WslArgs -e bash -c "test -x ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo OK"
if ($px4BinCheck -ne "OK") {
    Write-Host "[ORCH] FAIL: PX4 binary missing at ~/PX4-Autopilot/build/px4_sitl_default/bin/px4."
    Write-Host "[ORCH] Build it once:  wsl -e bash -c 'cd ~/PX4-Autopilot && make px4_sitl none_iris'"
    exit 1
}

# Friendly nudge: firewall rule no longer needed under mirrored mode.
$rule = Get-NetFirewallRule -DisplayName "AirSim HIL TCP 4560" -ErrorAction SilentlyContinue
if ($rule) {
    Write-Host "[ORCH] Note: 'AirSim HIL TCP 4560' firewall rule is still present but unnecessary."
    Write-Host "[ORCH]       Remove it (admin) when convenient:"
    Write-Host "[ORCH]         Remove-NetFirewallRule -DisplayName 'AirSim HIL TCP 4560'"
}

# --------------------------------------------------------------------------
# Run directory + log files
# --------------------------------------------------------------------------
$Ts = Get-Date -Format "yyyy-MM-dd_HHmmss"
$LogDir = Join-Path $RepoRoot "logs/mavlink/$Ts"
New-Item -ItemType Directory -Path $LogDir -Force | Out-Null
$UeLog = Join-Path $LogDir "ue.log"
$Px4Log = Join-Path $LogDir "px4.log"
$ProbeLog = Join-Path $LogDir "probe.log"
$ManifestPath = Join-Path $LogDir "run.json"
"" | Set-Content $UeLog
"" | Set-Content $Px4Log
"" | Set-Content $ProbeLog
Write-Host "[ORCH] Logs: $LogDir"

# --------------------------------------------------------------------------
# Launch plan from Python (writes settings.json as a side effect)
# --------------------------------------------------------------------------
Write-Host "[ORCH] Resolving UE launch plan..."
Push-Location $RepoRoot
try {
    # Capture stdout only. uv writes build status to stderr; using 2>&1 in PS 5.1
    # wraps stderr lines as ErrorRecords and trips $ErrorActionPreference='Stop'.
    $planLines = & uv run python -c "from src.sim_launch import print_mavlink_launch_plan; print_mavlink_launch_plan()"
    $planJson = $planLines | Where-Object { $_ -match '^\s*\{' } | Select-Object -Last 1
    if (-not $planJson) {
        Write-Host "[ORCH] FAIL: print_mavlink_launch_plan emitted no JSON. Output was:"
        $planLines | ForEach-Object { Write-Host "  $_" }
        exit 1
    }
} finally {
    Pop-Location
}

$Plan = $planJson | ConvertFrom-Json
if ($Plan.error) {
    Write-Host "[ORCH] FAIL: $($Plan.error)"
    exit 1
}
$HilPort = if ($Plan.hil_tcp_port) { [int]$Plan.hil_tcp_port } else { 4560 }

# --------------------------------------------------------------------------
# Manifest
# --------------------------------------------------------------------------
$manifest = [ordered]@{
    started_at = (Get-Date).ToString("o")
    log_dir = $LogDir
    ue_executable = $Plan.colosseum
    ue_args = $Plan.args
    settings_path = $Plan.settings_path
    airsim_port = $Plan.airsim_port
    hil_tcp_port = $HilPort
    px4_command = "PX4_SIM_HOST_ADDR=127.0.0.1 make px4_sitl none_iris"
    wsl_distro = if ($WslDistro) { $WslDistro } else { "(default)" }
}
$manifest | ConvertTo-Json -Depth 5 | Set-Content $ManifestPath -Encoding utf8

# --------------------------------------------------------------------------
# Cleanup
# --------------------------------------------------------------------------
$Script:Procs = @()
$Script:CleanupDone = $false
$Script:EventSubs = @()

function Invoke-Cleanup {
    if ($Script:CleanupDone) { return }
    $Script:CleanupDone = $true
    Write-Host ""
    Write-Host "[ORCH] Stopping all processes..."

    # Kill PX4 inside WSL (the make target spawns a px4 binary).
    try {
        & wsl @WslArgs -e bash -c "pkill -INT -f 'px4 -i 0' 2>/dev/null; pkill -f 'make px4_sitl' 2>/dev/null; true" 2>$null | Out-Null
    } catch {}

    foreach ($p in $Script:Procs) {
        if ($null -ne $p -and -not $p.HasExited) {
            # Kill($true) (kill-tree) needs .NET 5+; PS 5.1 silently ignores the
            # bool. Use taskkill /T to walk the tree on legacy frameworks.
            try { & taskkill /F /T /PID $p.Id 2>$null | Out-Null } catch {}
            try { $p.Kill() } catch {}
        }
    }

    foreach ($sub in $Script:EventSubs) {
        try { Unregister-Event -SourceIdentifier $sub.Name -ErrorAction SilentlyContinue } catch {}
    }

    # Restore SimpleFlight settings.json from backup so `uv run sim` works.
    Push-Location $RepoRoot
    try {
        & uv run python -c "from src.sim_launch import restore_simpleflight_settings; print('restored' if restore_simpleflight_settings() else 'no-backup')" 2>$null | ForEach-Object {
            Write-Host "[ORCH] settings.json restore: $_"
        }
    } finally {
        Pop-Location
    }

    Write-Host "[ORCH] Logs saved to: $LogDir"
}

$Script:EventSubs += Register-EngineEvent -SourceIdentifier PowerShell.Exiting -Action { Invoke-Cleanup }

# --------------------------------------------------------------------------
# Process spawning helper (file-only logging; orchestrator status to console)
# --------------------------------------------------------------------------
function ConvertTo-Win32ArgString {
    # Quote args per Win32 CommandLineToArgvW rules. Required because PS 5.1 /
    # .NET Framework 4.x ProcessStartInfo lacks ArgumentList (PS 7 / .NET Core+
    # have it). See https://learn.microsoft.com/en-us/cpp/cpp/main-function-command-line-args
    param([string[]]$ArgList)
    $parts = foreach ($a in $ArgList) {
        if ([string]::IsNullOrEmpty($a)) {
            '""'
        } elseif ($a -match '[\s"]') {
            $escaped = $a -replace '(\\*)"', '$1$1\"' -replace '(\\+)$', '$1$1'
            '"' + $escaped + '"'
        } else {
            $a
        }
    }
    return ($parts -join ' ')
}

function Start-LoggedProcess {
    <#
    Spawns a process, captures stdout+stderr to $LogPath, and pushes each line
    into $State.RecentLines (a ConcurrentQueue). Caller polls via Drain-Lines
    in the main loop for any progress checks (avoids cross-runspace variable
    pitfalls of Register-ObjectEvent -Action runspaces).
    #>
    param(
        [Parameter(Mandatory)] [string]$Exe,
        [Parameter(Mandatory)] [string[]]$ArgList,
        [Parameter(Mandatory)] [string]$LogPath,
        [Parameter(Mandatory)] [hashtable]$State
    )

    if (-not $State.ContainsKey("RecentLines")) { $State.RecentLines = [System.Collections.Concurrent.ConcurrentQueue[string]]::new() }

    $psi = [System.Diagnostics.ProcessStartInfo]::new()
    $psi.FileName = $Exe
    $psi.Arguments = ConvertTo-Win32ArgString -ArgList $ArgList
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true

    $proc = [System.Diagnostics.Process]::new()
    $proc.StartInfo = $psi

    $msgData = @{
        LogPath = $LogPath
        State = $State
    }

    $handler = {
        if ($null -eq $EventArgs.Data) { return }
        $line = $EventArgs.Data
        try {
            Add-Content -Path $Event.MessageData.LogPath -Value $line -ErrorAction SilentlyContinue
        } catch {}
        $st = $Event.MessageData.State
        $st.RecentLines.Enqueue($line)
        # Keep buffer bounded so a never-drained queue doesn't grow unboundedly
        while ($st.RecentLines.Count -gt 1024) {
            [string]$drop = ""
            [void]$st.RecentLines.TryDequeue([ref]$drop)
        }
    }

    $sub1 = Register-ObjectEvent -InputObject $proc -EventName OutputDataReceived -Action $handler -MessageData $msgData
    $sub2 = Register-ObjectEvent -InputObject $proc -EventName ErrorDataReceived -Action $handler -MessageData $msgData
    $Script:EventSubs += $sub1
    $Script:EventSubs += $sub2

    [void]$proc.Start()
    $proc.BeginOutputReadLine()
    $proc.BeginErrorReadLine()
    $Script:Procs += $proc
    return $proc
}

function Drain-Lines {
    param([hashtable]$State)
    $out = New-Object System.Collections.Generic.List[string]
    $tmp = ""
    while ($State.RecentLines.TryDequeue([ref]$tmp)) { $out.Add($tmp) }
    return $out
}

# --------------------------------------------------------------------------
# Phase 1: launch UE
# --------------------------------------------------------------------------
try {
    Write-Host "[ORCH] Phase 1: launching Unreal Engine..."
    $UeArgs = @($Plan.args | ForEach-Object { [string]$_ })
    $UeState = @{}
    $UeProc = Start-LoggedProcess -Exe $Plan.colosseum -ArgList $UeArgs -LogPath $UeLog -State $UeState

    Write-Host "[ORCH] Waiting (up to 120s) for AirSim HIL TCP :$HilPort to listen..."
    $deadline = (Get-Date).AddSeconds(120)
    $hilReady = $false
    while ((Get-Date) -lt $deadline) {
        if ($UeProc.HasExited) {
            Write-Host "[ORCH] FAIL: UE exited early (code $($UeProc.ExitCode)). See $UeLog"
            exit 1
        }
        $hits = (& netstat -ano -p TCP) | Select-String ":$HilPort\s.*LISTENING"
        if ($hits) { $hilReady = $true; break }
        Start-Sleep -Seconds 1
    }
    if (-not $hilReady) {
        Write-Host "[ORCH] FAIL: HIL listener never came up on :$HilPort. Check $UeLog."
        exit 1
    }
    Write-Host "[ORCH] HIL listener ready on :$HilPort."

    # ----------------------------------------------------------------------
    # Phase 2: launch PX4 in WSL
    # ----------------------------------------------------------------------
    Write-Host "[ORCH] Phase 2: launching PX4-SITL in WSL..."
    $px4ScriptWin = Join-Path $PSScriptRoot "launch_px4_wsl.sh"
    $px4ScriptWslRaw = & wsl @WslArgs -e wslpath -u $px4ScriptWin
    $px4ScriptWsl = ($px4ScriptWslRaw | Out-String).Trim()
    if (-not $px4ScriptWsl) {
        Write-Host "[ORCH] FAIL: could not translate $px4ScriptWin to a WSL path."
        exit 1
    }

    $Px4State = @{}
    $Px4ArgList = @() + $WslArgs + @("-e", "bash", $px4ScriptWsl)
    $Px4Proc = Start-LoggedProcess -Exe "wsl" -ArgList $Px4ArgList -LogPath $Px4Log -State $Px4State

    Write-Host "[ORCH] Waiting (up to 90s) for PX4 to connect to AirSim..."
    $deadline = (Get-Date).AddSeconds(90)
    $px4Connected = $false
    $connLine = ""
    while ((Get-Date) -lt $deadline) {
        if ($Px4Proc.HasExited) {
            Write-Host "[ORCH] FAIL: PX4 exited early (code $($Px4Proc.ExitCode)). Tail of px4.log:"
            Get-Content $Px4Log -Tail 30
            exit 1
        }
        foreach ($line in (Drain-Lines -State $Px4State)) {
            if ($line -match "Simulator connected on TCP port") {
                $px4Connected = $true
                $connLine = $line
            }
        }
        if ($px4Connected) { break }
        Start-Sleep -Milliseconds 500
    }
    if (-not $px4Connected) {
        Write-Host "[ORCH] FAIL: PX4 did not log 'Simulator connected on TCP port'. Tail of px4.log:"
        Get-Content $Px4Log -Tail 30
        exit 1
    }
    Write-Host "[ORCH] PX4 connected: $($connLine.Trim())"

    # ----------------------------------------------------------------------
    # Phase 3: probe
    # ----------------------------------------------------------------------
    Write-Host "[ORCH] Phase 3: starting check-mavlink probe (Ctrl+C here to stop everything)..."
    $ProbeState = @{}
    Push-Location $RepoRoot
    try {
        $ProbeProc = Start-LoggedProcess -Exe "uv" -ArgList @("run", "check-mavlink", "--duration", "0", "--quiet") -LogPath $ProbeLog -State $ProbeState
    } finally {
        Pop-Location
    }

    Write-Host ""
    Write-Host "[ORCH] All three processes running. Ctrl+C to stop."
    Write-Host "[ORCH] Live logs: $LogDir\{ue,px4,probe}.log"
    Write-Host ""

    # ----------------------------------------------------------------------
    # Idle loop — drain probe lines for live console feedback (throttled),
    # exit on Ctrl+C or any process death.
    # ----------------------------------------------------------------------
    $lastProbeMirror = [DateTime]::MinValue
    while ($true) {
        Start-Sleep -Milliseconds 500
        foreach ($line in (Drain-Lines -State $ProbeState)) {
            if ($line -match "^\[check-mavlink\] tick:") {
                $now = [DateTime]::Now
                if (($now - $lastProbeMirror).TotalSeconds -ge 5) {
                    $lastProbeMirror = $now
                    Write-Host "[PROBE] $line"
                }
            }
            elseif ($line -match "^\[check-mavlink\] (PASS|FAIL|=== summary|interrupted|WARN|probing|passive)") {
                Write-Host "[PROBE] $line"
            }
            elseif ($line -match "^\s*:\d+\s+pkts=") {
                # Final per-port summary line
                Write-Host "[PROBE] $line"
            }
        }
        if ($UeProc.HasExited)    { Write-Host "[ORCH] UE exited (code $($UeProc.ExitCode)). Stopping.";    break }
        if ($Px4Proc.HasExited)   { Write-Host "[ORCH] PX4 exited (code $($Px4Proc.ExitCode)). Stopping.";  break }
        if ($ProbeProc.HasExited) { Write-Host "[ORCH] Probe exited (code $($ProbeProc.ExitCode)). Stopping."; break }
    }
}
finally {
    Invoke-Cleanup
}
