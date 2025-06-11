#!/usr/bin/env python

import cv2
import numpy as np
import gi
from dronekit import connect, Vehicle, VehicleMode
from time import sleep
import gii.gii_bluerov as gii_bluerov
from gii.blueROV_config import get_config
from gii.detect_color import Video

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

# === Only Specify which BlueROV to use ===
ROV_NAME = "bluerov2"
config = get_config(ROV_NAME)

# === Build connection string ===
connection_string = f"{config['host_ip']}:{config['port']}"
print(f"Connecting to vehicle on: {connection_string}")
autopilot = connect(connection_string, wait_ready=False)

# === Arm to move the robot ===
autopilot.mode = VehicleMode("MANUAL")
autopilot.mode = VehicleMode("MANUAL")
sleep(2)

autopilot.armed = True
sleep(5)
autopilot.armed = True
print("Armed:", autopilot.armed)
print("Mode:", autopilot.mode.name)

# Initialize video
video = Video()

def check_for_yellow(frame):
    """Check if yellow is detected in the frame"""
    # Convert frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Yellow color range in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    
    yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    total_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter out small contours
            total_area += area
    
    return total_area > 0

def search_pattern():
    try:
        # Initial depth
        current_depth = autopilot.location.global_relative_frame.alt
        target_depth = -0.2  # Target depth in meters
        
        # Move to initial depth
        while current_depth > target_depth:
            gii_bluerov.move_rov(autopilot, "z", "displacement", -20)  # Move down
            sleep(0.1)
            current_depth = autopilot.location.global_relative_frame.alt
            print(f"Current depth: {current_depth:.2f}m")
        
        # Stop vertical movement
        gii_bluerov.move_rov(autopilot, "z", "displacement", 0)
        
        while True:
            # Rotate 360 degrees
            print("Rotating to search for yellow...")
            gii_bluerov.move_rov(autopilot, "z", "rotation", 20)
            for _ in range(90):  # total ~18 seconds (90 × 0.2s)
                if video.frame_available():
                    frame = video.frame()
                    if check_for_yellow(frame):
                        print("Yellow detected during rotation!")
                        gii_bluerov.stop_rov(autopilot)
                        return
                sleep(0.2)  # smaller sleep for real-time processing
            gii_bluerov.move_rov(autopilot, "z", "rotation", 0)

            
           
            # Move forward for 5 seconds
            print("Moving forward...")
            gii_bluerov.move_rov(autopilot, "x", "displacement", 20)  # Move forward
            sleep(5)
            gii_bluerov.move_rov(autopilot, "x", "displacement", 0)  # Stop forward movement
            
            # Check for yellow while moving forward
            if video.frame_available():
                frame = video.frame()
                if check_for_yellow(frame):
                    print("Yellow detected while moving forward!")
                    gii_bluerov.stop_rov(autopilot)
                    return
            
            # If no yellow found, go deeper
            current_depth = autopilot.location.global_relative_frame.alt
            target_depth -= 0.2  # Decrease depth by 0.2m
            
            print(f"Moving to new depth: {target_depth:.2f}m")
            while current_depth > target_depth:
                gii_bluerov.move_rov(autopilot, "z", "displacement", -20)  # Move down
                sleep(0.1)
                current_depth = autopilot.location.global_relative_frame.alt
                print(f"Current depth: {current_depth:.2f}m")
            
            gii_bluerov.move_rov(autopilot, "z", "displacement", 0)  # Stop vertical movement
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        gii_bluerov.stop_rov(autopilot)
        autopilot.armed = False

if __name__ == '__main__':
    print('Initialising stream...')
    waited = 0
    while not video.frame_available():
        waited += 1
        print('\r  Frame not available (x{})'.format(waited), end='')
        cv2.waitKey(30)
    print('\nSuccess!\nStarting search pattern...')
    
    search_pattern() 