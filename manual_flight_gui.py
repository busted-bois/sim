"""
Manual flight launcher for Simple AirSim.
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple AirSim manual control GUI")
    parser.add_argument("--vjoy", action="store_true", help="Enable keyboard->vJoy bridge")
    parser.add_argument("--vjoy-device", type=int, default=1, metavar="N", help="vJoy device id")
    parser.add_argument("--debug", action="store_true", help="Verbose keyboard/vJoy logs")
    args = parser.parse_args()

    level = logging.DEBUG if args.debug else logging.INFO
    kwargs = {
        "level": level,
        "format": "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        "datefmt": "%H:%M:%S",
    }
    if sys.version_info >= (3, 8):
        kwargs["force"] = True
    logging.basicConfig(**kwargs)

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


if __name__ == "__main__":
    main()
