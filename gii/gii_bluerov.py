
# DRONEKIT LIBRARY TO EASE THE USE OF THE BLUEROV AND LIMIT THE MOVEMENTS
# DO NOT MODIFY WITHOUT SUPERVISION!
# V1.1 IMPROVE DIRECTION USAGE

# limits to avoid crashes 
# DO NOT MODIFY WITHOUT SUPERVISION
max_speed = 1570
min_speed = 1435


# movement function
# 	axis = "x", "y" or "z"
# 	movement_type = "displacement" or "rotation"
#   speed_percentage = speed percentage (-100%-100%) 
def move_rov (autopilot, axis,movement_type, speed_percentage=10):
    if movement_type == "displacement":
        if axis == "x":
            channel_to_move=5
        elif axis == "y":
            channel_to_move=6
        elif axis == "z":
            channel_to_move=3
    if movement_type == "rotation":
        if axis=="x":
            channel_to_move=2
        elif axis=="y":
            channel_to_move=1
        elif axis=="z":
            channel_to_move=4
    desired_speed=1500 + speed_percentage * 5
    if desired_speed>max_speed:
        desired_speed=max_speed
    if desired_speed<min_speed:
        desired_speed=min_speed
    autopilot.channels.overrides[channel_to_move] = desired_speed    


def stop_rov(autopilot):
    # Set the override channels to neutral values
    autopilot.channels.overrides['1'] = 1500
    autopilot.channels.overrides['2'] = 1500
    autopilot.channels.overrides['3'] = 1500
    autopilot.channels.overrides['4'] = 1500
    autopilot.channels.overrides['5'] = 1500
    autopilot.channels.overrides['6'] = 1500
    autopilot.channels.overrides['7'] = 1500


def open_gripper_rov (autopilot):
    autopilot.channels.overrides['7'] = 1550    


def close_gripper_rov (autopilot):
    autopilot.channels.overrides['7'] = 1450   


def stop_gripper_rov (autopilot):
    autopilot.channels.overrides['7'] = 1500   
