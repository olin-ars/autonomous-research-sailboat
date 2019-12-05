#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32


class Boat:
    def __init__(self):

        self.wind_dir = 0
        self.tar_pos = np.array([None, None])
        self.tar_heading = None
        self.target_reached = True  # won't be true until first target is received
        self.controlled = False

        self.curr_heading = 0
        self.curr_pos = np.array([0, 0], dtype='float32')

        self.xs = []
        self.ys = []

        self.disp_wind()

        rospy.init_node('boat_sim', anonymous=True)

        rospy.Subscriber("abs_wind_dir", Float32, self.update_abs_wind, queue_size=1)
        rospy.Subscriber("target_position", Point32, self.update_tar_pos, queue_size=1)
        rospy.Subscriber("target_heading", Float32, self.update_tar_heading, queue_size=1)
        rospy.Subscriber("rudder_position", Float32, self.update_rudder, queue_size=1)

        self.pub_heading = rospy.Publisher("current_heading", Float32, queue_size=1)
        self.pub_position = rospy.Publisher("current_position", Point32, queue_size=1)

    def run(self):
        r = rospy.Rate(10)
        clock = 0
        while not rospy.is_shutdown():
            while not self.target_reached and clock < 8:
                r.sleep()
                clock += 1
                self.update()
            if self.target_reached:
                print("Target reached!")
            else: # This one gives up easily
                print("Time limit reached.")
                clock = 0
                self.target_reached = True
            while self.target_reached:
                pass

    def update(self):
        if self.curr_heading is not None:
            self.curr_pos += 2 * np.array([np.cos(self.curr_heading * np.pi/180), np.sin(self.curr_heading * np.pi/180)])
        print("NEW Position: %.2f, %.2f\tHeading: %.2f" % (self.curr_pos[0], self.curr_pos[1], self.curr_heading))

        self.xs.append(self.curr_pos[0])
        self.ys.append(self.curr_pos[1])
        ch_msg = Float32(self.curr_heading)
        cp_msg = Point32(x=self.curr_pos[0], y=self.curr_pos[1], z=0)
        self.pub_heading.publish(ch_msg)
        self.pub_position.publish(cp_msg)
        print("CURRENT HEADING PUBLISHED")

        path = self.tar_pos - self.curr_pos  # calculate the path
        if np.sqrt(path[0] ** 2 + path[1] ** 2) < 5:  # re-calculate whether target has been reached.
            self.target_reached = True

    def update_abs_wind(self, msg):
        self.wind_dir = msg.data

    def update_tar_pos(self, msg):
        x, y, _ = msg.x, msg.y, msg.z
        tp = np.array([x, y])
        if (self.tar_pos != tp).any():  # if target changes, reset the target_reached flag
            self.tar_pos = tp
            self.target_reached = False
            print("TARGET: %.2f, %.2f\t CURRENT: %.2f, %.2f" % (x, y, self.curr_pos[0], self.curr_pos[1]))

    def update_tar_heading(self, msg):
        # if no data is coming from rudder controller, just jump to desired heading
        # otherwise, tar_heading is irrelevant to this node since the rudder controller is driving
        if not self.controlled:
            self.tar_heading = msg.data
            self.curr_heading = self.tar_heading

    def update_rudder(self, msg):
        print("Rudder message received!")
        turn_effectiveness = 1 # instantaneous boat turn per degree of rudder turn
        self.controlled = True # show that boat direction is being controlled
        if self.curr_heading is None: # in case heading was not previously set, init at 0
            self.curr_heading = 0
        self.curr_heading -= msg.data * turn_effectiveness # adjust heading with rudder 

    def disp_wind(self):
        print("WIND: %.2f" % self.wind_dir)


if __name__ == '__main__':
    b = Boat()
    b.run()
    exit()
