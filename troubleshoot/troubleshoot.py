# script example to operate the bluerov using python

from time import sleep
from dronekit import connect
import gii.gii_bluerov as gii_bluerov
from gii.blueROV_config import get_config  # import the config getter

# === Only Specify which BlueROV to use ===
ROV_NAME = "bluerov2"  # Change this to match the blueROV
config = get_config(ROV_NAME)

# === Build connection string, DO NOT TOUCH ===
connection_string = f"{config['rov_ip']}:{config['port']}"
print(f"Connecting to vehicle on: {connection_string}")
autopilot = connect(connection_string, wait_ready=False)

# === Get some vehicle attributes ===
print("Battery: %s" % autopilot.battery)

# === Arm to move the robot ===
autopilot.armed = True
sleep(5)
print("Armed:", autopilot.armed)
print("Altitude:", autopilot.location.global_relative_frame.alt)

# === Movement ===
#gii_bluerov.move_rov(autopilot, "x", "displacement", -15)
#sleep(10)
#gii_bluerov.move_rov(autopilot, "z", "displacement", -15)
#sleep(20)
#print("Altitude:", autopilot.location.global_relative_frame.alt)

# === Gripper control ===
if config.get("gripper"):
    gii_bluerov.open_gripper_rov(autopilot)
    sleep(0.5)
    gii_bluerov.stop_gripper_rov(autopilot)
    sleep(2.0)
    gii_bluerov.close_gripper_rov(autopilot)
    sleep(3.2)

# === Stop and disarm ===
gii_bluerov.stop_rov(autopilot)

autopilot.armed = False
sleep(1)
print("Disarmed:", autopilot.armed)
