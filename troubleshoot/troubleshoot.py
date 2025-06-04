# script example to operate the bluerov using python

from time import sleep
from dronekit import connect, Vehicle
import gii.gii_bluerov as gii_bluerov
from gii.blueROV_config import get_config  # import the config getter

from dronekit import VehicleMode

# Patch mode availability check
def always_true_mode_available(self, custom_mode, base_mode):
    return True

Vehicle._is_mode_available = always_true_mode_available

# Patch class-level mode mapping (not instance-level)
Vehicle._mode_mapping_bynumber = {
    0: 'STABILIZE',
    2: 'ALT_HOLD',
    3: 'DEPTH_HOLD',
    7: 'CIRCLE',
    10: 'AUTO',
    15: 'GUIDED',
    16: 'SURFACE',
    19: 'MANUAL'
}

# === Only Specify which BlueROV to use ===
ROV_NAME = "bluerov2"  # Change this to match the blueROV
config = get_config(ROV_NAME)

# === Build connection string, DO NOT TOUCH ===
connection_string = f"{config['host_ip']}:{config['port']}"
print(f"Connecting to vehicle on: {connection_string}")
autopilot = connect(connection_string, wait_ready=False)


# === Get some vehicle attributes ===
print("Battery: %s" % autopilot.battery)

# === Arm to move the robot ===
autopilot.mode = VehicleMode("MANUAL")
autopilot.mode = VehicleMode("MANUAL")
sleep(2)

autopilot.armed = True
sleep(5)
autopilot.armed = True
print("Armed:", autopilot.armed)
print("Mode:", autopilot.mode.name)
print("Altitude:", autopilot.location.global_relative_frame.alt)

# === Movement ===
gii_bluerov.move_rov(autopilot, "x", "displacement", 15)
sleep(5)
#gii_bluerov.move_rov(autopilot, "z", "displacement", -15)
#sleep(20)
#print("Altitude:", autopilot.location.global_relative_frame.alt)

# === Gripper control ===
if config.get("gripper"):
    gii_bluerov.open_gripper_rov(autopilot)
    sleep(1)
    gii_bluerov.stop_gripper_rov(autopilot)
    sleep(5.0)
    gii_bluerov.close_gripper_rov(autopilot)
    sleep(3.2)

# === Stop and disarm ===
gii_bluerov.stop_rov(autopilot)

autopilot.armed = False
sleep(1)
print("Armed:", autopilot.armed)
