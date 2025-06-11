#!/usr/bin/env python

import dronekit
import time
import cv2
import numpy as np
import gi
from dronekit import connect, Vehicle, VehicleMode
from time import sleep
import gii.gii_bluerov as gii_bluerov
from gii.blueROV_config import get_config 

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer
Gst.init(None)

# Patch mode availability check
def always_true_mode_available(self, custom_mode, base_mode):
    return True

Vehicle._is_mode_available = always_true_mode_available

# Patch class-level mode mapping
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

def setup_rov():
    """Setup and arm the ROV"""
    ROV_NAME = "bluerov2"  # Change this to match your blueROV
    config = get_config(ROV_NAME)
    
    connection_string = f"{config['host_ip']}:{config['port']}"
    print(f"Connecting to vehicle on: {connection_string}")
    autopilot = connect(connection_string, wait_ready=False)
    
    # Set to MANUAL mode and arm
    autopilot.mode = VehicleMode("MANUAL")
    sleep(2)
    autopilot.armed = True
    sleep(5)
    
    print("Armed:", autopilot.armed)
    print("Mode:", autopilot.mode.name)
    return autopilot

def setup_video():
    """Setup video capture using GStreamer"""
    port = 5600
    video_source = f'udpsrc port={port}'
    video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
    video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
    video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'
    
    video_pipe = Gst.parse_launch(' '.join([video_source, video_codec, video_decode, video_sink_conf]))
    video_pipe.set_state(Gst.State.PLAYING)
    return video_pipe, video_pipe.get_by_name('appsink0')

def gst_to_opencv(sample):
    """Convert GStreamer sample to OpenCV format"""
    buf = sample.get_buffer()
    caps_structure = sample.get_caps().get_structure(0)
    array = np.ndarray(
        (
            caps_structure.get_value('height'),
            caps_structure.get_value('width'),
            3
        ),
        buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
    return array

def detect_yellow(frame):
    """Detect yellow color in frame and return center coordinates and area"""
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Yellow color range in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    
    yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    total_area = 0
    weighted_sum_x = 0
    weighted_sum_y = 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter out small contours
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                weighted_sum_x += cX * area
                weighted_sum_y += cY * area
                total_area += area
                
                # Draw visualization
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
    
    if total_area > 0:
        center_x = int(weighted_sum_x / total_area)
        center_y = int(weighted_sum_y / total_area)
        return (center_x, center_y), total_area
    
    return None, 0

def search_pattern(autopilot, video_sink):
    """Simple search pattern: rotate and move forward"""
    try:
        while True:
            # Rotate
            print("Rotating...")
            gii_bluerov.move_rov(autopilot, "z", "rotation", 20)
            for _ in range(70):
                sample = video_sink.emit('pull-sample')
                if sample:
                    frame = gst_to_opencv(sample)
                    center, area = detect_yellow(frame)
                    if center is not None:
                        gii_bluerov.move_rov(autopilot, "z", "rotation", 0)
                        return center, area
                sleep(0.1)
            
            # Move forward
            print("Moving forward...")
            gii_bluerov.move_rov(autopilot, "x", "displacement", 20)
            for _ in range(50):
                sample = video_sink.emit('pull-sample')
                if sample:
                    frame = gst_to_opencv(sample)
                    center, area = detect_yellow(frame)
                    if center is not None:
                        gii_bluerov.move_rov(autopilot, "x", "displacement", 0)
                        return center, area
                sleep(0.1)
            
            gii_bluerov.move_rov(autopilot, "x", "displacement", 0)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        gii_bluerov.stop_rov(autopilot)

def main():
    try:
        # Setup ROV and video
        autopilot = setup_rov()
        video_pipe, video_sink = setup_video()
        
        print('Starting yellow object tracking...')
        while True:
            sample = video_sink.emit('pull-sample')
            if sample:
                frame = gst_to_opencv(sample)
                center, area = detect_yellow(frame)
                
                if center is not None:
                    print(f"Yellow object detected at {center} with area {area}")
                    # Move towards the object
                    gii_bluerov.move_rov(autopilot, "x", "displacement", 20)
                else:
                    print("No yellow object detected, starting search pattern...")
                    gii_bluerov.stop_rov(autopilot)
                    center, area = search_pattern(autopilot, video_sink)
                
                # Display frame
                cv2.imshow('Yellow Tracking', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
            sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Cleanup
        gii_bluerov.stop_rov(autopilot)
        autopilot.armed = False
        video_pipe.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()