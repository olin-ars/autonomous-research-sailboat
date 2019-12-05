#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Point32


class Boat:
    def __init__(self):

        self.wind_dir = 0
        self.tar_pos = np.array([None, None])
        self.tar_heading = None
        self.target_reached = -1  # -1 = no target, 0 = not reached, 1 = reached
        self.controlled = False

        self.curr_heading = 0
        self.curr_pos = np.array([0, 0], dtype='float32')

        self.xs = []
        self.ys = []

        print("WIND: %.0f" % self.wind_dir)

        rospy.init_node('boat_sim', anonymous=True)

        self.sub_aw = rospy.Subscriber("abs_wind_dir", Float32, self.update_abs_wind, queue_size=1)
        self.sub_tp = rospy.Subscriber("target_position", Point32, self.update_tar_pos, queue_size=1)
        self.sub_th = rospy.Subscriber("target_heading", Float32, self.update_tar_heading, queue_size=1)
        self.sub_ts = rospy.Subscriber("target_status", Int8, self.update_tar_status, queue_size=1)
        self.sub_rp = rospy.Subscriber("rudder_position", Float32, self.update_rudder, queue_size=1)

        self.pub_heading = rospy.Publisher("current_heading", Float32, queue_size=1)
        self.pub_position = rospy.Publisher("current_position", Point32, queue_size=1)

    def run(self):
        print('Boat sim node initialized.')

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            while self.target_reached == 0:
                self.update()
                r.sleep()
            if self.target_reached == 1:
                print("Target reached!")
            if self.target_reached == -1:
                print("Timeout exceeded or target unset.")

            while self.target_reached != 0: # wait for a new goal
                pass

    def update(self):
        if self.curr_heading is not None and self.curr_pos is not None:
            self.curr_pos += 2 * np.array([np.cos(self.curr_heading * np.pi/180), np.sin(self.curr_heading * np.pi/180)])
            print("NEW Position: %.2f, %.2f\tHeading: %.2f" % (self.curr_pos[0], self.curr_pos[1], self.curr_heading))

            # was used for logging path
            self.xs.append(self.curr_pos[0])
            self.ys.append(self.curr_pos[1])
            
            ch_msg = Float32(self.curr_heading)
            cp_msg = Point32(x=self.curr_pos[0], y=self.curr_pos[1], z=0)
            self.pub_heading.publish(ch_msg)
            self.pub_position.publish(cp_msg)

    def update_abs_wind(self, msg):
        self.wind_dir = msg.data

    def update_tar_pos(self, msg):
        x, y, _ = msg.x, msg.y, msg.z
        tp = np.array([x, y])

    def update_tar_heading(self, msg):
        # if no data is coming from rudder controller, just jump to desired heading
        # otherwise, tar_heading is irrelevant to this node since the rudder controller is driving
        if not self.controlled:
            self.tar_heading = msg.data
            self.curr_heading = self.tar_heading

    def update_rudder(self, msg):
        turn_effectiveness = 1 # instantaneous boat turn per degree of rudder turn
        self.controlled = True # show that boat direction is being controlled
        if self.curr_heading is not None:
            self.curr_heading += msg.data * turn_effectiveness # adjust heading with rudder 

    def update_tar_status(self, msg):
        self.target_reached = msg.data


if __name__ == '__main__':
    b = Boat()
    b.run()
    exit()
