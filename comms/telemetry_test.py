import sys
sys.path.insert(0, r'E:\source\repos\Colosseum\PythonClient')
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Take off
client.takeoffAsync().join()

# Read telemetry for 5 seconds
print("Reading telemetry...")
for i in range(10):
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    vel = state.kinematics_estimated.linear_velocity
    orientation = state.kinematics_estimated.orientation
    
    print(f"Position: x={pos.x_val:.2f} y={pos.y_val:.2f} z={pos.z_val:.2f}")
    print(f"Velocity: x={vel.x_val:.2f} y={vel.y_val:.2f} z={vel.z_val:.2f}")
    print("---")
    time.sleep(0.5)

client.landAsync().join()