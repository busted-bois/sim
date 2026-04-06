"""
Open Simple AirSim's PySimpleGUI window: telemetry + algorithm controls + Manual Mode.

Manual Mode calls AirSim enableApiControl(False) so you fly with the sim / keyboard
(see simple_airsim README: W/S throttle, A/D yaw, arrow keys pitch/roll).

Usage (simulator running, same as test_square.py):
    python manual_flight_gui.py
"""

import sys
from pathlib import Path

_SIMPLE_AIRSIM_ROOT = Path(__file__).resolve().parent / "simple_airsim"
if _SIMPLE_AIRSIM_ROOT.is_dir():
    sys.path.insert(0, str(_SIMPLE_AIRSIM_ROOT))

from simple_airsim.api import coordinate_system
from simple_airsim.api.gui_manager import GUIManager
from simple_airsim.api.manager import Manager

if __name__ == "__main__":
    with Manager(coordinate_system.AIRSIM) as man:
        with GUIManager(man, 10, 10, 10, 3) as gui:
            gui.start()
