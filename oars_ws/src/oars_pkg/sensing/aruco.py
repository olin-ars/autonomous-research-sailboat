#!/usr/bin/env python

"""
This code recognizes Aruco markers, calculates their absolute position and heading, and then
publishes that information to a ROS channel.

@author: Elias Gabriel
"""
import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
import cv2
import os
import cv2.aruco as aruco

class ArucoGPS:

    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480

    MIN_LAT = 42.283030
    MIN_LON = -71.313271
    MAX_LAT = 42.292964
    MAX_LON = -71.302482

    def __init__(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.params = aruco.DetectorParameters_create()
        self.cap = cv2.VideoCapture(0)
        self.has_screen = bool(os.environ.get('DISPLAY', None))

    def calculate_heading_position(self):
        """
        Locates an Aruco marker in the video frame and returns its heading and position. The marker
        position is transformed from the camera coodinate space to an artifical latitudinal and
        longitudinal range (based off the size of Lake Waban).
        """
        _, frame = self.cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, _, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)

        if corners:
            center = np.mean(corners[0][0][0::2], 0)
            front = np.mean(corners[0][0][0:2], 0)
            facing = np.subtract(front, center)
            angle = np.rad2deg(np.arctan2(-facing[1], facing[0])) % 360

            center_x = ((center[0] / ArucoGPS.CAMERA_WIDTH) * (ArucoGPS.MAX_LAT - ArucoGPS.MIN_LAT)) + ArucoGPS.MIN_LAT
            center_y = ((center[1] / ArucoGPS.CAMERA_HEIGHT) * (ArucoGPS.MAX_LON - ArucoGPS.MIN_LON)) + ArucoGPS.MIN_LON

            return (center_x, center_y), angle

        return None, None

    def run(self):
        """
        Calculates the current heading and position of the boat via `calculate_heading_position()`,
        and then publishes the information to the `current_position` ROS channel. Position and
        heading is calculated and published at a rate of 10Hz.
        """
        pos_pub = rospy.Publisher('current_position', Point32, queue_size=0)
        heading_pub = rospy.Publisher('current_heading', Float32, queue_size=0)

        rospy.init_node('oops', anonymous=True)
        clock = rospy.Rate(10)

        while not rospy.is_shutdown():
            pos, angle = self.calculate_heading_position()

            if pos and angle:
                pos_pub.publish(Point32(pos[0], pos[1], 0))
                heading_pub.publish(Float32(angle))

            if self.has_screen:
                cv2.imshow('frame', get_frame())
                if cv2.waitKey(1) & 0xFF == ord('q'): break

            clock.sleep()

if __name__ == '__main__':
    arucogps = ArucoGPS()
    try: arucogps.run()
    except rospy.ROSInterruptException: pass