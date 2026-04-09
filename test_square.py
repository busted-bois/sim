import sys
sys.path.insert(0, r'E:\source\repos\Colosseum\PythonClient')
sys.path.insert(0, r'C:\Users\kunal\Desktop\ai-grand-prix_drone-challenge')

import time
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
        man.start()