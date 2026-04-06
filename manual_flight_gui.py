"""
Open Simple AirSim's PySimpleGUI window: telemetry + algorithm controls + Manual Mode.

Manual Mode sets enableApiControl(False). Stock AirSim expects a **physical gamepad**;
to fly from the keyboard you need **vJoy** + this repo's VirtualController:

  1. vJoy: Configure vJoy → enable a device with axes **X, Y, Rx, Ry** (often device #1).
  2. pip install pyvjoy pynput  (already listed in simple_airsim/requirements.txt)
  3. python manual_flight_gui.py --vjoy [--vjoy-device 2]
  4. In the GUI, select **Manual Mode**, then use **W/S** throttle, **A/D** yaw,
     **arrow keys** pitch/roll (while the sim or any window has focus — listener is global).

Use an empty lidar_names dict so the GUI skips getLidarData when your environment
has no lidar sensors.

If keys do nothing in Manual Mode + --vjoy: vJoy is usually fine; AirSim is often
listening to the wrong joystick index. Open Windows "Set up USB game controllers"
and note the list order (e.g. vJoy = 1st → use RemoteControlID 0, PS5 1st → vJoy is 1).
Copy simple_airsim/settings.json into Documents/AirSim/settings.json and try
RemoteControlID 0, then 1, then 2 under Vehicles -> your drone -> RC.

Usage (simulator running):
    python manual_flight_gui.py
    python manual_flight_gui.py --vjoy
    python manual_flight_gui.py --vjoy --debug
"""

import argparse
import logging
import sys
from pathlib import Path

_SIMPLE_AIRSIM_ROOT = Path(__file__).resolve().parent / "simple_airsim"
if _SIMPLE_AIRSIM_ROOT.is_dir():
    sys.path.insert(0, str(_SIMPLE_AIRSIM_ROOT))

from simple_airsim.api import coordinate_system
from simple_airsim.api.gui_manager import GUIManager
from simple_airsim.api.manager import Manager

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simple AirSim control GUI")
    parser.add_argument(
        "--vjoy",
        action="store_true",
        help="Keyboard → vJoy virtual joystick. Requires vJoy driver + pyvjoy + pynput.",
    )
    parser.add_argument(
        "--vjoy-device",
        type=int,
        default=1,
        metavar="N",
        help="vJoy device number (1–16). Use if device 1 is BUSY or you enabled another slot in vJoyConf.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Verbose terminal logs: key events, vJoy axis samples, Manual/Algorithm mode RPC notes.",
    )
    args = parser.parse_args()

    level = logging.DEBUG if args.debug else logging.INFO
    _log_kw = dict(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    if sys.version_info >= (3, 8):
        _log_kw["force"] = True
    logging.basicConfig(**_log_kw)
    logging.getLogger("PIL").setLevel(logging.WARNING)

    with Manager(coordinate_system.AIRSIM, lidar_names={}) as man:
        with GUIManager(
            man,
            10,
            10,
            10,
            3,
            use_virtual_controller=args.vjoy,
            vjoy_device_id=args.vjoy_device,
            virtual_controller_debug=args.debug,
        ) as gui:
            gui.start()
