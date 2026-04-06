import sys
from pathlib import Path

# Package layout: repo/simple_airsim/simple_airsim/ — parent folder must be on sys.path.
_SIMPLE_AIRSIM_ROOT = Path(__file__).resolve().parent / "simple_airsim"
if _SIMPLE_AIRSIM_ROOT.is_dir():
    sys.path.insert(0, str(_SIMPLE_AIRSIM_ROOT))

# AirSim Python client (e.g. Colosseum) if your environment needs it:
# sys.path.insert(0, r"E:\source\repos\Colosseum\PythonClient")

from simple_airsim.api import coordinate_system
from simple_airsim.api.drone import Drone
from simple_airsim.api.manager import Manager

def square(drone: Drone):
    drone.takeoff(True)
    for i in range(4):
        drone.move_by(5, 0, 0, True)
        drone.turn_by(0, 0, 90, True)
    drone.land(True)

if __name__ == '__main__':
    with Manager(coordinate_system.AIRSIM, method=square) as man:
        man.start_algo()
        if man.algo_thread is not None:
            man.algo_thread.join()