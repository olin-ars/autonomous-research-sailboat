#!/usr/bin/env python

"""
This program implements the short course routing algorithm described in Section 5.1 of Roland Stelzer's Autonomous
Sailboat Navigation paper. This algorithm uses a polar efficiency function to determine the efficiency of all possible
angles that a boat can turn, and then compares the efficiencies to determine the optimal boat heading.
Read the paper here: https://www.dora.dmu.ac.uk/bitstream/handle/2086/7364/thesis-optimized-300dpi.pdf
@Authors: Jane Sieving, Duncan Mazza and Shreya Chowdhary
"""

import numpy as np
import rospy
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Point32


class ShortCoursePlanner:

    def __init__(self, p_c=40):
        self.p_c = p_c  # The beating parameter (which controls the length of a tack)
        self.abs_wind = 0
        self.curr_heading = None
        self.curr_pos = np.array([None, None])
        self.tar_pos = np.array([None, None])
        self.new_heading = None
        self.target_reached = -1  # -1 = no target, 0 = not reached, 1 = reached
        self.clock = 0
        self.timeout = 200

        rospy.init_node('path_planner', anonymous=True)

        self.sub_ch = rospy.Subscriber("current_heading", Float32, self.set_curr_heading, queue_size=1)
        self.sub_cp = rospy.Subscriber("current_position", Point32, self.set_curr_pos, queue_size=1)
        # draws from command topic, so target is only set once
        self.sub_tp = rospy.Subscriber("cmd_tp", Point32, self.set_tar_pos, queue_size=1)

        self.pub_new_heading = rospy.Publisher("target_heading", Float32, queue_size=1)
        self.pub_status = rospy.Publisher("target_status", Int8, queue_size=1)


    def run(self):
        print("Path planner node initialized.")

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_target_status()
            if self.target_reached == 0:
                self.clock += 1
                r.sleep()
                self.publish_heading()
            
            if self.clock > self.timeout:  # give up
                self.target_reached = -1  # admit defeat
                self.tar_pos = np.array([None, None])  # burn your dreams
                print("Timeout in path planner: target unset.", self.target_reached, self.tar_pos)
                self.clock = 0  # wish you could start over

    def publish_target_status(self):
        msg = Int8(self.target_reached)
        self.pub_status.publish(msg)

    def publish_heading(self):
        if self.curr_heading is not None and self.curr_pos.all() is not None and self.tar_pos.all() is not None:
            self.calc_heading()
            msg = Float32(self.new_heading)
            self.pub_new_heading.publish(msg)

    def set_curr_heading(self, msg):
        self.curr_heading = msg.data

    def set_curr_pos(self, msg):
        x, y, _ = msg.x, msg.y, msg.z
        self.curr_pos = np.array([x, y])
        if self.tar_pos.all() is not None:
            path = self.curr_pos - self.tar_pos
            if np.sqrt(path[0] ** 2 + path[1] ** 2) < 5:
                self.target_reached = 1
                print("Target reached in path planner!", self.target_reached)

    def set_tar_pos(self, msg):
        x, y, _ = msg.x, msg.y, msg.z
        tp = np.array([x, y])  # temporary position variable
        if (self.tar_pos != tp).any():  # if target changes, reset the target_reached flag
            self.tar_pos = tp
            self.target_reached = 0
            print("Target changed in path planner!", self.target_reached)
        if (x, y) is (None, None): # if target is none, unset it
            self.target_reached = -1
            print("Target reset in path planner.", self.target_reached)


    def _get_polar_efficiency(self, alpha):
        """
        Calculates the hypothetical speed for a given angle alpha, using the polar efficiency function.

        :param wind_angle: the wind direction in degrees (relative to 0 degrees, with 0 degrees being east)
        :param alpha: angle being tested (relative to the wind angle)
        :return: the vector components for the velocity for the angle as a numpy array
        """
        angle = self.abs_wind + alpha

        # Calculate magnitude using the polar efficiency function
        mag = (1 - np.cos(alpha)) * (1 + 0.3 * np.cos(alpha)) / (1 - 0.5 * np.cos(alpha))

        # Return vector components as a numpy array
        res = np.array([mag * np.cos(angle), mag * np.sin(angle)])
        return res

    def _get_optima(self, path):
        """
        Finds the maximum velocity and angle that can be achieved for every possible new heading for the boat to
        determine the most optimal headings on the port and starboard sides of the boat.

        :param wind_angle: angle representing the wind.
        :param path: vector representing the path taken to reach from boat's position to the target position
        :return: A tuple containing the following information:
           windangle_max_r - max boat heading on right side
            windangle_max_l - max boat heading on left side
            vt_max_r - max velocity on right side
            vt_max_l - max velocity on left side
        """

        wind_angle = self.abs_wind * np.pi/180

        # Initializing the right side and left side optimum wind angles
        windangle_max_r = wind_angle
        windangle_max_l = wind_angle

        # Initializing the right side and left side optimum max velocities
        vt_max_r = 0
        vt_max_l = 0

        # Checks all alphas between 0 and 180 and calculates the theoretical speeds
        # to find an optimal heading for a starboard tack
        for alpha in np.linspace(0, np.pi, 40):

            # Uses the polar efficiency function to calculcate the hypothetical velocity of the boat at that angle
            vb_hyp = self._get_polar_efficiency(alpha)

            # Projecting the max speed in the direction of the path
            vt_test = np.dot(vb_hyp, path)/np.sqrt(path[0] ** 2 + path[1] ** 2)

            # Check if this velocity is the new max
            if vt_test > vt_max_r:
                vt_max_r = vt_test
                windangle_max_r = (wind_angle + alpha) % (2*np.pi)

        # Analogous to the starboard tack, but for a port tack
        for alpha in np.linspace(np.pi, 2*np.pi, 40):
            vb_hyp = self._get_polar_efficiency(alpha)
            vt_test = np.dot(vb_hyp, path)/np.sqrt(path[0] ** 2 + path[1] ** 2)
            if vt_test > vt_max_l:
                vt_max_l = vt_test
                windangle_max_l = (wind_angle + alpha) % (2*np.pi)

        return vt_max_r, vt_max_l, windangle_max_r, windangle_max_l

    def _get_best_dir(self, vt_max_r, vt_max_l, windangle_max_r, windangle_max_l, path):
        """
        Checks whether a port tack or starboard tack would be more efficient.

        :param vt_max_r: the maximum velocity on the right hand side
        :param vt_max_l: the maximum velocity on the left hand side
        :param windangle_max_r: angle (in degrees) corresponding to the max velocity on the right hand side
        :param windangle_max_l: angle (in degrees) corresponding to the max velocity on the left hand side
        :param p_c: beating parameter (controls the length of the tacks)
        :param path: the path the boat is following as (a numpy array)
        :param boat_heading: the boat's heading, relative to 0 (with 0 as east)
        :return: the new boat direction and speed
        """
        # Calculates the hysteresis factor
        n = 1 + (self.p_c / (abs(np.sqrt(path[0] ** 2 + path[1] ** 2))))

        heading = self.curr_heading * np.pi/180

        # Choose the direction that is closest to the current boat heading
        r_diff = np.pi - abs(abs(windangle_max_r - heading) - np.pi)
        l_diff = np.pi - abs(abs(windangle_max_l - heading) - np.pi)

        # Choose the closer side, balancing with faster velocity
        if r_diff < l_diff:  # r is closer
            if vt_max_r * n < vt_max_l:  # l is significantly faster
                new_boat_heading = windangle_max_l
            else:
                new_boat_heading = windangle_max_r  # r is fast enough
        else:                # l is closer
            if vt_max_l * n < vt_max_r:  # r is significantly faster
                new_boat_heading = windangle_max_r
            else:
                new_boat_heading = windangle_max_l  # l is fast enough

        return new_boat_heading * 180/np.pi

    def calc_heading(self):
        """
        This method directs the boat by using the boat's position and the target position to calculate the path
        and comparing the boat's heading to the wind heading to determine the most efficient way for the boat to get
        to its target position.

        :param b_p - the boat's current position as a Python list
        :param b_h - the boat's current heading as an angle in degrees
        :param w - the wind angle in degrees, relative to 0 (with 0 as east)
        :return new_dir - the boat's new direction as a numpy array
        """

        # Calculates the path of the boat
        path = self.tar_pos - self.curr_pos
        print("Path", path)

        # Runs the optimum function and compares the optima returned to
        # determine the best direction for the boat to go
        vt_max_r, vt_max_l, windangle_max_r, windangle_max_l = self._get_optima(path)
        self.new_heading = self._get_best_dir(vt_max_r, vt_max_l, windangle_max_r, windangle_max_l, path)
        # print(self.new_heading)


if __name__ == "__main__":
    # Initialize planner object
    planner = ShortCoursePlanner()
    planner.run()
    exit()
