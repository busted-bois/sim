"""
Manual flight launcher for Simple AirSim.
"""

import argparse
import logging
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
_SIMPLE_AIRSIM_ROOT = _ROOT / "simple_airsim"
if _SIMPLE_AIRSIM_ROOT.is_dir():
    sys.path.insert(0, str(_SIMPLE_AIRSIM_ROOT))
sys.path.insert(0, str(_ROOT))


def _install_virtual_controller_shim(device_id: int) -> None:
    """Provide simple_airsim.api.virtual_controller (missing upstream at the pinned commit)."""
    import importlib

    from src import manual_virtual_controller as shim

    class _BoundVirtualController(shim.VirtualController):
        def __init__(self) -> None:
            super().__init__(device_id=device_id)

    shim.VirtualController = _BoundVirtualController  # type: ignore[misc]
    sys.modules["simple_airsim.api.virtual_controller"] = shim
    importlib.import_module("simple_airsim.api").virtual_controller = shim  # type: ignore[attr-defined]


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple AirSim manual control GUI")
    parser.add_argument("--vjoy", action="store_true", help="Enable keyboard->vJoy bridge")
    parser.add_argument("--vjoy-device", type=int, default=1, metavar="N", help="vJoy device id")
    parser.add_argument("--debug", action="store_true", help="Verbose keyboard/vJoy logs")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        force=True,
    )

    if args.vjoy:
        _install_virtual_controller_shim(args.vjoy_device)

    from simple_airsim.api import coordinate_system
    from simple_airsim.api.gui_manager import GUIManager
    from simple_airsim.api.manager import Manager

    with Manager(coordinate_system.AIRSIM, lidar_names=None) as man:
        # Upstream SimDrone skips assignment when {} is passed; force-clear here so
        # GUIManager doesn't RPC-query lidars the user's settings.json doesn't define.
        man._drone.lidar_names = {}
        with GUIManager(
            man,
            10,
            10,
            10,
            3,
            use_virtual_controller=args.vjoy,
        ) as gui:
            gui.start()


if __name__ == "__main__":
    main()
