#!/usr/bin/env python
"""
BlueRov video capture class
"""

import cv2
import gi
import numpy as np
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
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

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """ Get frame to update _new_frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK

    def capture_image(self, frame, filename="captured_image.png"):
        """Capture and save the current frame as an image"""
        if frame is not None:
            # Save the image with the current timestamp in the filename
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            file_name = f"{filename}_{timestamp}.png"
            cv2.imwrite(file_name, frame)
            print(f"Image saved as {file_name}")
        else:
            print("No frame to capture!")

    #Añadido por GPT4
    def detect_red_color(self, frame):
        # Convert frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red color range in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Masks for red color
        mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter out small contours to reduce noise
                # Compute the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # Print centroid and area of the blob
                print(f"Blob found at ({cX}, {cY}) with area: {area}")

                # Optional: Draw the contour and centroid on the frame
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        return frame

    def detect_orange_color(self, frame):
        # Convert frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Orange color range in HSV
        lower_orange = np.array([5, 50, 50])
        upper_orange = np.array([15, 255, 255])

        orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

        # Find contours in the mask
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter out small contours to reduce noise
                # Compute the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # Print centroid and area of the blob
                print(f"Blob found at ({cX}, {cY}) with area: {area}")

                # Optional: Draw the contour and centroid on the frame
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        return frame

    def detect_green_color(self, frame):
        # Convert frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Green color range in HSV
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])

        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

        # Find contours in the mask
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter out small contours to reduce noise
                # Compute the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # Print centroid and area of the blob
                print(f"Blob found at ({cX}, {cY}) with area: {area}")

                # Optional: Draw the contour and centroid on the frame
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        return frame

    def detect_blue_color(self, frame):
        # Convert frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue color range in HSV
        lower_blue = np.array([90, 120, 70])
        upper_blue = np.array([120, 255, 255])

        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

        # Find contours in the mask
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter out small contours to reduce noise
                # Compute the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # Print centroid and area of the blob
                print(f"Blob found at ({cX}, {cY}) with area: {area}")

                # Optional: Draw the contour and centroid on the frame
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        return frame

    def detect_yellow_color(self, frame):

        # Convert frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_yellow = np.array([11, 8, 92])
        upper_yellow = np.array([64, 117, 160])
        
        # Yellow color range in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        
        
        

        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

        # Find contours in the mask
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        total_area = 0
        weighted_sum_x = 0
        weighted_sum_y = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Adjust the minimum area threshold to detect larger portions
                # Compute the centroid of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # Accumulate the weighted sum of centroids
                weighted_sum_x += cX * area
                weighted_sum_y += cY * area

                # Accumulate the total area
                total_area += area

                # Optional: Draw the contour and centroid on the frame
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)

        # Calculate the weighted average of centroids
        if total_area > 0:
            weighted_avg_x = weighted_sum_x / total_area
            weighted_avg_y = weighted_sum_y / total_area
            weighted_center = (int(weighted_avg_x), int(weighted_avg_y))
            print(f"Weighted center of all yellow blobs: {weighted_center}")

            # Draw the weighted center on the frame
            cv2.circle(frame, weighted_center, 7, (0, 0, 255), -1)

        # Calculate the geometrical center of the image
        image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        #print(f"Geometrical center of the image: {image_center}")

        # Draw the geometrical center on the frame
        cv2.circle(frame, image_center, 5, (255, 0, 0), -1)

        print(f"Total area of all yellow blobs: {total_area}")

        return frame




if __name__ == '__main__':
    # Create the video object
    # Add port= if is necessary to use a different one
    video = Video()

    print('Initialising stream...')
    waited = 0
    while not video.frame_available():
        waited += 1
        print('\r  Frame not available (x{})'.format(waited), end='')
        cv2.waitKey(30)
    print('\nSuccess!\nStarting streaming - press "q" to quit.')

    while True:
        # Wait for the next frame to become available
        if video.frame_available():
            # Only retrieve and display a frame if it's new
            frame = video.frame() 
            video.detect_yellow_color(frame)
            #video.detect_green_color(frame)
            #video.detect_orange_color(frame)
            #video.detect_red_color(frame)
            #video.detect_blue_color(frame)
            cv2.imshow('frame', frame)
            #video.capture_image(frame)

        # Allow frame to display, and check if user wants to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Press 'q' to quit
            break
        elif key == ord('c'):  # Press 'c' to capture an image
            video.capture_image(frame)  # Capture the current frame

    
