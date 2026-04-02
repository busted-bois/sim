import sys
sys.path.insert(0, r'E:\source\repos\Colosseum\PythonClient')
import airsim

# Connect to the simulator
client = airsim.MultirotorClient()
client.confirmConnection()
print("✅ Connected to simulator!")

# Take control
client.enableApiControl(True)
client.armDisarm(True)
print("✅ API control enabled!")

# Take off
print("Taking off...")
client.takeoffAsync().join()
print("✅ Drone is flying!")

# Hover for 3 seconds then land
import time
time.sleep(3)

print("Landing...")
client.landAsync().join()
client.armDisarm(False)
print("✅ Landed!")