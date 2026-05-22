# `~/Documents/AirSim/settings.json` reference

AirSim's per-user settings file lives at:

- Windows: `%USERPROFILE%\Documents\AirSim\settings.json` (i.e. `C:\Users\<you>\Documents\AirSim\settings.json`)
- Linux/macOS: `~/Documents/AirSim/settings.json`

The shape depends entirely on **transport**. The launcher manages this file as a side effect of `uv run sim` (SimpleFlight) and `uv run sim-mavlink` / `uv run mavlink-all` (PX4Multirotor). **Prefer the launcher** — only edit by hand if you're debugging or the launcher's auto-write isn't applying.

## Backups

The launcher backs up the SimpleFlight version to `~/Documents/AirSim/settings.simpleflight.bak.json` before switching to PX4 mode. To restore manually:

```bash
uv run sim-restore-simpleflight
```

## SimpleFlight (AirSim RPC) shape

For `transport=airsim`. Drone is controlled via msgpack-rpc on `:41451`.

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ViewMode": "FlyWithMe",
  "ClockSpeed": 1.0,
  "ApiServerPort": 41451,
  "Vehicles": {
    "SimpleFlight": {
      "VehicleType": "SimpleFlight",
      "DefaultVehicleState": "Armed",
      "EnableCollisions": true,
      "EnableCollisionPassthrough": false
    }
  }
}
```

Key fields:

| Field             | Value              | Why                                                       |
| ----------------- | ------------------ | --------------------------------------------------------- |
| `SimMode`         | `Multirotor`       | Drone, not car.                                           |
| `ApiServerPort`   | `41451`            | Must match `sim.config.json` → `simulator.airsim_port`.   |
| `VehicleType`     | `SimpleFlight`     | Built-in flight controller. **Not** PX4.                  |

## PX4Multirotor (MAVLink) shape

For `transport=mavlink`. PX4-SITL connects over TCP `:4560` for HIL; control flows over UDP `:14540` ↔ `:14580` / `:14550`.

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ViewMode": "FlyWithMe",
  "ClockSpeed": 1.0,
  "ApiServerPort": 41451,
  "ClockType": "SteppableClock",
  "LocalHostIp": "127.0.0.1",
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "UseTcp": true,
      "TcpPort": 4560,
      "LockStep": true,
      "ControlIp": "remote",
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "LocalHostIp": "0.0.0.0",
      "QgcHostIp": "127.0.0.1",
      "QgcPort": 14550,
      "Parameters": {
        "NAV_RCL_ACT": 0,
        "NAV_DLL_ACT": 0
      }
    }
  }
}
```

Key fields:

| Field                | Value               | Why                                                         |
| -------------------- | ------------------- | ----------------------------------------------------------- |
| `UseTcp` / `TcpPort` | `true` / `4560`     | HIL channel PX4-SITL dials in on. Must be open on host.     |
| `LockStep`           | `true`              | Physics steps in lockstep with PX4 ticks. Required for stable HIL. |
| `ControlIp`          | `"remote"`          | Tells AirSim PX4 lives on a different host/distro.          |
| `LocalHostIp`        | `"0.0.0.0"`         | Bind UDP control ports on all interfaces (so WSL can reach).|
| `QgcPort`            | `14550`             | Where ground-station traffic mirrors to.                    |
| `NAV_RCL_ACT` / `NAV_DLL_ACT` | `0`        | Disable RC-loss and data-link-loss failsafes for SITL.      |

## Common combinations

| Vehicles entries           | Use case                                                                  |
| -------------------------- | ------------------------------------------------------------------------- |
| 1× `SimpleFlight`          | Standard autonomous run. `uv run sim` writes this.                        |
| 1× `PX4Multirotor`         | MAVLink probe / PX4 development. `uv run sim-mavlink` writes this.        |
| 2× (one of each)           | Not supported by this codebase's launcher — pick one.                     |

## Detecting which one is active

```powershell
$s = Get-Content "$env:USERPROFILE\Documents\AirSim\settings.json" | ConvertFrom-Json
$s.Vehicles.PSObject.Properties | ForEach-Object { "$($_.Name) => $($_.Value.VehicleType)" }
```

Should print one line. If it prints `SimpleFlight => SimpleFlight` you're in AirSim/RPC mode. If `PX4 => PX4Multirotor` you're in MAVLink mode.

## What not to do

- **Do not** set `ApiServerPort` to anything other than `41451` unless you also update `sim.config.json` → `simulator.airsim_port` to match.
- **Do not** mix `LockStep: true` with manual physics rate overrides in `sim.config.json` — keep `physics_update_hz: 120` and let the lockstep handle pacing.
- **Do not** commit your local `settings.json` to the repo — it's a user-local file and contains absolute paths.
