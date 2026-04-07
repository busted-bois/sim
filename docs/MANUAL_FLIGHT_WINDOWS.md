# Manual flight on Windows (AirSim / Simple AirSim)

This guide is for teammates who already have **Unreal + AirSim (or Colosseum)** connecting from Python (`Connected!` / API control works), and want **manual control**: either **keyboard via vJoy** or a **physical gamepad**.

The project uses **`simple_airsim`** plus **`manual_flight_gui.py`** at the repo root.

---

## 1. What you need installed

| Requirement | Purpose |
|-------------|---------|
| **Windows 10/11** | Same flow as development machine |
| **Python 3.8+** (venv recommended) | Run the GUI and client |
| **Working sim + Python client** | `MultirotorClient` connects (you’ve done this if API scripts work) |
| **PySimpleGUI** | Control window |
| **vJoy** (optional for keyboard) | Virtual joystick the sim reads as a gamepad |
| **pyvjoy + pynput** (optional) | Our script feeds vJoy from WASD + arrows |

Install Python deps for Simple AirSim (includes the above):

```powershell
cd path\to\ai-grand-prix_drone-challenge
python -m venv venv
.\venv\Scripts\activate
pip install -r simple_airsim\requirements.txt
```

---

## 2. vJoy (keyboard → “gamepad”)

AirSim’s **SimpleFlight** manual input is built around **joystick / RC**, not raw keyboard. To fly with **WASD**, we feed a **virtual joystick**:

1. Download and install **[vJoy](https://sourceforge.net/projects/vjoystick/)**.
2. Open **Configure vJoy** (vJoyConf).
3. Enable **at least one device** (usually **device 1**).
4. For that device, enable axes **X, Y, Rx, Ry** (and optionally others). **Apply** and leave **Enable vJoy** checked.

**Important:** Only one program at a time should *own* a given vJoy device. Close other feeders before running our script.

---

## 3. AirSim `settings.json` location (OneDrive!)

AirSim loads **`settings.json`** from:

**`%USERPROFILE%\Documents\AirSim\settings.json`**

If **Documents** is redirected to **OneDrive**, the real path is often:

**`%USERPROFILE%\OneDrive\Documents\AirSim\settings.json`**

A **minimal** file is not enough: you need **`SimMode`**, **`Vehicles`**, and **RC** so the drone uses the right **gamepad index**.

1. Copy the repo template **`simple_airsim\settings.json`** into that folder (merge with any keys your team already uses).
2. Adjust **`Vehicles`** / vehicle name if your sim expects a different name than **`Drone1`** (match what the sim logs when it starts).
3. Restart the **sim binary** after any change.

Template includes **`AllowAPIAlways`**, **`RC.RemoteControlID`**, and **`AllowAPIWhenDisconnected`** for switching between API and manual.

---

## 4. `RemoteControlID` vs vJoy “device 1–16”

These are **different**:

| Concept | Meaning |
|---------|--------|
| **vJoy device 1–16** | Tab/slot in **Configure vJoy**. Matches **`--vjoy-device`** in our launcher (default **1**). |
| **`RemoteControlID`** | **0-based index** in Windows **“Set up USB game controllers”** / **`joy.cpl`** (first listed = **0**, second = **1**, …). |

**Do not** set `RemoteControlID` to **1** just because vJoy says “device 1”. Set it to the **row index of “vJoy Device”** in `joy.cpl`:

- vJoy **first** in the list → `"RemoteControlID": 0`
- **DualSense / Xbox first**, vJoy **second** → `"RemoteControlID": 1`

Restart the sim after editing.

---

## 5. Run manual control

From repo root (venv activated):

```powershell
# Keyboard → vJoy → sim (needs vJoy + pyvjoy + pynput)
python manual_flight_gui.py --vjoy

# Optional: other vJoyConf slot
python manual_flight_gui.py --vjoy --vjoy-device 2

# Verbose logs (keys + axis samples)
python manual_flight_gui.py --vjoy --debug
```

In the **Main Controls** window:

1. Select **Manual Mode** (releases API control to the sim).
2. Use **W/S** throttle, **A/D** yaw, **arrow keys** pitch/roll **when using `--vjoy`**.

**Without `--vjoy`**, WASD is **not** sent to vJoy; use a **USB gamepad** in Manual Mode or add `--vjoy`.

Algorithm **Start / Pause / Stop** only apply when `Manager(..., method=some_function)` is used (e.g. `test_square.py`). For **`manual_flight_gui.py`** there is no script attached; that is expected.

---

## 6. Physical gamepad (no vJoy)

1. Plug in the controller; ensure Windows sees it in **`joy.cpl`**.
2. Run **`python manual_flight_gui.py`** (no `--vjoy`).
3. **Manual Mode** → set **`RemoteControlID`** to that controller’s **list index** (often **0** if it’s the only device).

PS5 may need a Windows/XInput wrapper depending on drivers.

---

## 7. Troubleshooting checklist

| Symptom | Check |
|--------|--------|
| `Cannot acquire vJoy Device ... not VJD_STAT_FREE` | Close other vJoy feeders; reboot; try `--vjoy-device 2` if slot 2 is enabled in vJoyConf. |
| vJoy **Properties** in `joy.cpl` **don’t** move with WASD | Run with `--vjoy --debug`; try **Run as administrator** for the terminal once. |
| Axes move in `joy.cpl` but drone doesn’t | Wrong **`RemoteControlID`** or **`settings.json`** not in **OneDrive `Documents\AirSim`**; restart sim after edits. |
| HUD says API released but no manual input | You’re not using `--vjoy` and have no gamepad, or RC index points at wrong device. |
| Lidar errors on GUI startup | Repo uses **`lidar_names={}`** in `manual_flight_gui.py` so lidar-less maps work; if you customize `Manager`, pass `{}` if you have no lidar sensors. |

---

## 8. Files in this repo (reference)

| Path | Role |
|------|------|
| `manual_flight_gui.py` | Launches PySimpleGUI + optional vJoy keyboard bridge |
| `simple_airsim/simple_airsim/api/virtual_controller.py` | WASD → vJoy axes |
| `simple_airsim/simple_airsim/api/gui_manager.py` | UI, Manual / Algorithm mode, `enableApiControl` |
| `simple_airsim/settings.json` | **Template** to copy into `Documents\AirSim\` (see §3) |
| `test_square.py` | Example **API** flight with `Manager(..., method=square)` |

---

## 9. Competition note

Race rules require **fully autonomous** flight for timed runs. Manual flight is for **development, debugging, and positioning** in sim—not for official competition submissions.
