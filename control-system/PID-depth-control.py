# script example to operate the bluerow using python

import dronekit
from time import sleep
import gii.gii_bluerov as gii_bluerov
from gii.blueROV_config import get_config  # import the config getter

# === Only Specify which BlueROV to use ===
ROV_NAME = "bluerov2"  # Change to "bluerov1" as needed
config = get_config(ROV_NAME)

# === Build connection string, DO NOT TOUCH ===
connection_string = f"{config['rov_ip']}:{config['port']}"
print(f"Connecting to vehicle on: {connection_string}")
autopilot = connect(connection_string, wait_ready=False)

# === Get some vehicle attributes ===
print("Battery: %s" % autopilot.battery)

# Arm to move the robot
autopilot.armed=True
sleep(5.0)
print ("%s" % autopilot.armed)
print(autopilot.location.global_relative_frame.alt)

# PID Constants
Kp = 100
Ki = 25
Kd = 0
setpoint = -1  # Desired depth (e.g., 1 meter)
dt = 0.1      # Time step in seconds

# PID Variables
previous_error = 0
integral = 0
current_depth = autopilot.location.global_relative_frame.alt  # Initial depth

def compute(current_depth):
    global previous_error, integral

    error = setpoint - current_depth
    print(f"Error = {error}")
    integral += error * dt
    derivative = (error - previous_error) / dt

    output = (Kp * error) + (Ki * integral) + (Kd * derivative)
    previous_error = error

    return round(output)  # Output is the control signal (thrust)

# Simulate PID controller over time
for i in range(50000):  # Run for 5 seconds (50 steps * 0.1s)
    
    control_signal = compute(current_depth)
    gii_bluerov.move_rov (autopilot, "z","displacement", control_signal)
    sleep(0.9)
    # Simulate the system response (very basic physics)
    # Let's say thrust directly changes depth per time step
    current_depth = autopilot.location.global_relative_frame.alt  # You may add damping or system model here

    print(f"Time: {i * dt *10:.1f}s | Depth: {current_depth:.3f}m | Control: {control_signal:.3f}")

# Disarm after using it
autopilot.armed=False
sleep(1.0)
print ("%s" % autopilot.armed)