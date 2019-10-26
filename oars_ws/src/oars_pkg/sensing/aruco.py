"""
This code recognizes Aruco markers, calculates their absolute position and heading, and then
publishes that information to a ROS channel.

@Author(s): Elias Gabriel
"""
import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
import cv2
import cv2.aruco as aruco

class ArucoGPS:

    MIN_LAT = 42.283030
    MIN_LON = -71.313271
    MAX_LAT = 42.292964
    MAX_LON = -71.302482

    def __init__(self):
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.params = aruco.DetectorParameters_create()
        self.cap = cv2.VideoCapture(0)
        self.has_screen = bool(os.environ.get('DISPLAY', None))

    def calculate_heading_position():
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, _, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

        if corners:
            center = np.mean(corners[0][0][0::2], 0)
            front = np.mean(corners[0][0][0:2], 0)
            facing = np.subtract(front, center)
            angle = np.rad2deg(np.arctan2(-facing[1], facing[0])) % 360

            # TODO: Remap position to MIN/MAX GPS coords

            return center, angle

        return None, None

    def run():
        """
        Calculates the current heading and position of the boat via `calculate_heading_position()`,
        and then publishes the information to the `current_position` ROS channel. Position and
        heading is calculated and published at a rate of 10Hz.
        """
        pos_pub = rospy.Publisher('current_position', Point32, queue_size=0)
        heading_pub = rospy.Publisher('current_heading', Float32, queue_size=0)
        clock = rospy.Rate(10)

        rospy.init_node('current_position', anonymous=True)
        rospy.init_node('current_heading', anonymous=True)

        while not rospy.is_shutdown():
            pos, angle = calculate_heading_position()

            if pos and angle:
                pos_pub.publish(Point32(pos[0], pos[1], 0))
                heading_pub.publish(Float32(angle))

            if has_screen:
                cv2.imshow('frame', get_frame())
                if cv2.waitKey(1) & 0xFF == ord('q'): break

            clock.sleep()

if __name__ == '__main__':
    arucogps = ArucoGPS()
    try: arucogps.run()
    except rospy.ROSInterruptException: pass