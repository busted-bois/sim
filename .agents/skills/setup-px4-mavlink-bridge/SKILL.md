---
name: setup-px4-mavlink-bridge
description: One-time setup of the PX4-SITL + WSL bridge required for any MAVLink-mode simulator run in this repo. Use when the user wants to enable MAVLink/PX4 mode for the first time on a machine, when `uv run mavlink-all` fails with "PX4 binary missing" or "WSL mirrored networking is NOT enabled", or when reprovisioning a teammate's box. Covers installing WSL2, enabling mirrored networking, cloning and building PX4-Autopilot inside WSL, and verifying the bridge end-to-end. Does not configure the simulator itself — see the `configure-simulator` skill for per-run config.
compatibility: Windows 11 host with WSL2 available; admin shell needed for one-time firewall/WSL configuration. PX4 build needs ~10 GB free in WSL and ~30 minutes on first build.
metadata:
  applies-to: ai-grand-prix_drone-challenge
  frequency: one-time-per-machine
---

# Set up the PX4 ↔ MAVLink Bridge

This skill makes `transport=mavlink` actually work on a fresh machine. Run once per workstation; subsequent MAVLink runs only need [`configure-simulator`](../configure-simulator/SKILL.md).

The bridge is: **Windows host (UE 5.4 + AirSim PX4Multirotor)** ⇄ TCP `:4560` HIL ⇄ **WSL2 Ubuntu (PX4-SITL `none_iris`)** ⇄ UDP `:14540/14550/14580` (mirrored loopback).

## Pre-flight check — is the bridge already set up?

Run these three checks first. If all three pass, the bridge is built and you're done — skip to [`configure-simulator`](../configure-simulator/SKILL.md).

```powershell
# 1. WSL2 present
wsl -e true; if ($?) { "WSL OK" } else { "WSL MISSING" }

# 2. Mirrored networking enabled in .wslconfig
Select-String -Path "$env:USERPROFILE\.wslconfig" -Pattern '^\s*networkingMode\s*=\s*mirrored' -ErrorAction SilentlyContinue

# 3. PX4 binary built in WSL
wsl -e bash -c "test -x ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo PX4 OK || echo PX4 MISSING"
```

Any missing piece → continue with the matching step below.

## Step 1: WSL2

Install only if `wsl -e true` failed.

```powershell
# Admin PowerShell:
wsl --install -d Ubuntu
# reboot when prompted, finish Ubuntu first-run user setup
wsl --update
wsl --version   # confirm WSL2, kernel >= 5.15
```

Set Ubuntu as the default distro if multiple are installed:

```powershell
wsl --set-default Ubuntu
```

## Step 2: Mirrored networking

This makes WSL share the Windows loopback so PX4 in WSL can dial AirSim on the host without a firewall rule. The repo's orchestrator (`scripts/dev-mavlink.ps1`) refuses to run without it.

Easiest path — let the script write `.wslconfig` for you:

```powershell
pwsh scripts/dev-mavlink.ps1 -EnableMirrored
wsl --shutdown
```

Manual equivalent — create or edit `%USERPROFILE%\.wslconfig` and add:

```ini
[wsl2]
networkingMode=mirrored
```

Then `wsl --shutdown` and wait ~10 seconds before next WSL command.

Verify:

```powershell
Select-String -Path "$env:USERPROFILE\.wslconfig" -Pattern 'networkingMode\s*=\s*mirrored'
```

## Step 3: Clone and build PX4

Inside WSL:

```bash
sudo apt update
sudo apt install -y git make cmake gcc-arm-none-eabi python3-pip

# default location the launcher checks; override with $PX4_DIR if needed:
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh    # installs full toolchain; ~5-10 min, prompts may appear

# Initial build of the SITL target the launcher uses.
# This compiles + boots PX4 once. Press Ctrl+C after it prints "ready for takeoff" or
# after the iris model loads — we only need the binary.
make px4_sitl none_iris
```

The launcher specifically requires the binary at:

```
~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

Verify:

```bash
test -x ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo "PX4 built"
```

**Custom PX4 location?** The build helper at [`scripts/launch_px4_wsl.sh`](../../../scripts/launch_px4_wsl.sh) honors `PX4_DIR` env var, but the orchestrator's binary check is hardcoded to the default path above. Either symlink or stick with the default for the smoothest experience.

## Step 4: End-to-end verification

From the Windows side, repo root:

```powershell
uv run mavlink-all
```

Expect the orchestrator to print, in order:

1. `[ORCH] Phase 1: launching Unreal Engine...`
2. `[ORCH] HIL listener ready on :4560.`
3. `[ORCH] Phase 2: launching PX4-SITL in WSL...`
4. `[ORCH] PX4 connected: ... Simulator connected on TCP port 4560`
5. `[ORCH] Phase 3: starting check-mavlink probe...`
6. `[PROBE] [check-mavlink] tick: N pkts, M mavlink, K ATTITUDE decoded` (all > 0)
7. `[ORCH] ATTITUDE gate PASS: decoded=...`

`Ctrl+C` to tear down all three processes; the orchestrator restores SimpleFlight `settings.json` from backup automatically.

## Tear down / start over

| What                          | Command                                                                                  |
| ----------------------------- | ---------------------------------------------------------------------------------------- |
| Kill a stuck PX4 in WSL       | `wsl -e bash -c "pkill -INT -f 'px4 -i 0'; pkill -f 'make px4_sitl'"`                    |
| Wipe and rebuild PX4          | `wsl -e bash -c "cd ~/PX4-Autopilot && make clean && make px4_sitl none_iris"`           |
| Remove WSL distro entirely    | `wsl --unregister Ubuntu` (**destroys** the distro and any PX4 source/build in it)       |
| Remove old firewall rule      | `Remove-NetFirewallRule -DisplayName 'AirSim HIL TCP 4560'` (admin; only if present)     |

## Failure modes

| Symptom                                                               | Fix                                                                                            |
| --------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `wsl -e true` fails                                                   | WSL not installed — Step 1.                                                                    |
| Script says "WSL mirrored networking is NOT enabled"                  | Step 2. Don't forget `wsl --shutdown` after editing `.wslconfig`.                              |
| `make px4_sitl none_iris` fails with missing compiler                 | Re-run `bash ./Tools/setup/ubuntu.sh` — toolchain incomplete.                                  |
| PX4 connects to UE but ATTITUDE decode stays 0                        | Not a bridge issue — PX4 preflight is failing or QGC is grabbing UDP 14550. See [`configure-simulator/references/troubleshooting.md`](../configure-simulator/references/troubleshooting.md). |
| WSL command works in regular `wsl.exe` but fails from orchestrator    | Likely mirrored mode toggled off after a Windows update. Re-verify Step 2.                     |

## What this skill does NOT do

- It does not enable MAVLink mode in `sim.config.json`. That's the [`configure-simulator`](../configure-simulator/SKILL.md) skill's job (set `control.transport: "mavlink"`).
- It does not configure ground stations (QGC). QGC is optional — `mavlink-all` works without it.
- It does not install Unreal Engine 5.4 / Colosseum. That's a separate prerequisite documented in the project README.
