# UE 5.4 + BlocksV2 + MAVLink (PX4-SITL via WSL)

MAVLink/PX4 mode is **probe-only** in this repo — it brings UE up with PX4Multirotor settings, starts PX4-SITL inside WSL, and verifies the MAVLink bridge. It does **not** run `main.py` autonomous algorithms (use the AirSim/SimpleFlight combo for that — see [`ue54-blocksv2-airsim.md`](ue54-blocksv2-airsim.md)).

## Prerequisites

- Everything from [`ue54-blocksv2-airsim.md`](ue54-blocksv2-airsim.md) prerequisites section (UE 5.4, `.env.local`, `uv sync`).
- **PX4-SITL built in WSL** with mirrored networking enabled. If this has never been done on this machine, run the [`setup-px4-mavlink-bridge`](../../setup-px4-mavlink-bridge/SKILL.md) skill first — it's a one-time setup.
- Quick sanity check that PX4 is built:
  ```powershell
  wsl -e bash -c "test -x ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 && echo OK"
  ```
  Must print `OK`. If it doesn't, do not continue — run the bridge setup skill.

## `sim.config.json` settings that matter

```json
{
  "simulator": {
    "map_name": "BlocksV2",
    "map_asset": "/Game/FlyingCPP/Maps/FlyingExampleMapV2",
    "physics_update_hz": 120
  },
  "control": {
    "transport": "mavlink",
    "mavlink": {
      "endpoint": "udpin:0.0.0.0:14550",
      "endpoint_candidates": [14540, 14550, 14560, 14580, 5760, 5762],
      "auto_discover_endpoints": true,
      "airsim_profile": {
        "vehicle_name": "Drone1",
        "vehicle_type": "PX4Multirotor",
        "udp_ip": "127.0.0.1",
        "udp_port": 14560,
        "control_port_local": 14540,
        "control_port_remote": 14580,
        "qgc_host_ip": "127.0.0.1",
        "qgc_port": 14550,
        "lock_step": false
      }
    }
  }
}
```

**Algorithm pairing.** MAVLink mode in this repo is **probe-only** — `mavlink-all` does not run `main.py`. If you want `main.py` to start, set `"algorithm"` to `"mavlink_jitter"` (MAVLink-specific) and use `uv run sim` directly. For probe-only flows (the recommended path), the algorithm field is unread.

## `~/Documents/AirSim/settings.json`

Must declare a **PX4Multirotor** vehicle with TCP `4560` for HIL. `uv run sim-mavlink` writes this file for you — do not hand-edit. Full shape in [`airsim-settings.md`](airsim-settings.md).

The orchestrator backs up your SimpleFlight `settings.json` to `~/Documents/AirSim/settings.simpleflight.bak.json` before overwriting. Restore with `uv run sim-restore-simpleflight` when going back to the AirSim combo.

## Launch options (pick one)

| Command                          | What it does                                                                 |
| -------------------------------- | ---------------------------------------------------------------------------- |
| `uv run sim-mavlink`             | UE + PX4 settings only. You start PX4-SITL in WSL yourself.                  |
| `uv run sim-mavlink probe`       | Same, plus auto-runs `check-mavlink` for 60 s after launch.                  |
| `uv run mavlink-all`             | **All-in-one:** UE + PX4-SITL in WSL + probe. Logs to `logs/mavlink/<ts>/`. Recommended. |

`mavlink-all` (implementation: [`scripts/dev-mavlink.ps1`](../../../../scripts/dev-mavlink.ps1)) verifies WSL mirrored networking, waits for the HIL TCP listener on `:4560`, waits for PX4 to log `Simulator connected on TCP port`, then runs the probe with `--decode-attitude`. It exits with code 2 if MAVLink packets arrive but ATTITUDE decode stays 0 for 90 s (`-SkipAttitudeGate` to disable).

## Verification

- Probe output shows ticks like `tick: N pkts, M mavlink, K ATTITUDE decoded` with all three counters growing.
- `uv run check-mavlink --decode-attitude --duration 20` (in a separate terminal, with the sim still up) shows non-zero ATTITUDE decodes.
- For HIGHRES_IMU specifically: `uv run highres-imu-smoke`.
- For TIMESYNC: `uv run timesync-smoke`.

## Switching back to AirSim mode

```bash
uv run sim-restore-simpleflight
```

Then change `control.transport` back to `"airsim"` in `sim.config.json` and launch with `uv run sim`.

## Common pitfalls

- **`WSL mirrored networking is NOT enabled`** — `mavlink-all` refuses to start. Run `pwsh scripts/dev-mavlink.ps1 -EnableMirrored`, then `wsl --shutdown`, then re-run.
- **PX4 exits early with `bind: address already in use`** — another PX4 instance is still running in WSL. Kill it: `wsl -e bash -c "pkill -f 'px4 -i 0'; pkill -f 'make px4_sitl'"`.
- **HIL listener never comes up on `:4560`** — UE crashed or wrong `settings.json`. Check `logs/mavlink/<ts>/ue.log`.
- **MAVLink packets seen but ATTITUDE decoded = 0** — PX4 preflight didn't pass, or QGC is grabbing UDP `:14550`. Close QGC, confirm PX4 says `ready for takeoff`.

More symptoms in [`troubleshooting.md`](troubleshooting.md).
