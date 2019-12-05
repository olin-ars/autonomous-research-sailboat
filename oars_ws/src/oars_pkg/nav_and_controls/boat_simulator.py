#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32


class Boat:
    def __init__(self):

        self.log = open("boat_sim_log.txt", "a")
        self.wind_dir = 0
        self.tar_pos = np.array([None, None])
        self.tar_heading = None
        self.target_reached = -1  # -1 = no target, 0 = not reached, 1 = reached
        self.controlled = False

        self.curr_heading = 0
        self.curr_pos = np.array([0, 0], dtype='float32')

        self.xs = []
        self.ys = []

        self.disp_wind()

        rospy.init_node('boat_sim', anonymous=True)

        self.sub_aw = rospy.Subscriber("abs_wind_dir", Float32, self.update_abs_wind, queue_size=1)
        self.sub_tp = rospy.Subscriber("target_position", Point32, self.update_tar_pos, queue_size=1)
        self.sub_th = rospy.Subscriber("target_heading", Float32, self.update_tar_heading, queue_size=1)
        self.sub_ts = rospy.Subscriber("target_status", Int8, self.update_tar_status, queue_size=1)
        self.sub_rp = rospy.Subscriber("rudder_position", Float32, self.update_rudder, queue_size=1)

        self.pub_heading = rospy.Publisher("current_heading", Float32, queue_size=1)
        self.pub_position = rospy.Publisher("current_position", Point32, queue_size=1)

    def run(self):
        r = rospy.Rate(10)

        # if self.log is not None:
        self.log.write("_" * 80)

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

        if self.log is not None:
            self.log.close()

    def update(self):
        if self.curr_heading is not None and self.curr_pos is not None:
            self.curr_pos += 2 * np.array([np.cos(self.curr_heading * np.pi/180), np.sin(self.curr_heading * np.pi/180)])
            self.log_info("NEW Position: %.2f, %.2f\tHeading: %.2f" % (self.curr_pos[0], self.curr_pos[1], self.curr_heading))

            self.xs.append(self.curr_pos[0])
            self.ys.append(self.curr_pos[1])
            ch_msg = Float32(self.curr_heading)
            cp_msg = Point32(x=self.curr_pos[0], y=self.curr_pos[1], z=0)
            self.pub_heading.publish(ch_msg)
            self.pub_position.publish(cp_msg)
            self.log_info("CURRENT HEADING PUBLISHED")

            # HANDLED BY PATH PLANNER
            # path = self.tar_pos - self.curr_pos  # calculate the path
            # if np.sqrt(path[0] ** 2 + path[1] ** 2) < 5:  # re-calculate whether target has been reached.
            #     self.target_reached = 1

    def update_abs_wind(self, msg):
        self.wind_dir = msg.data

    def update_tar_pos(self, msg):
        x, y, _ = msg.x, msg.y, msg.z
        tp = np.array([x, y])
        # HANDLED BY PATH PLANNER
        # if (self.tar_pos != tp).any():  # if target changes, reset the target_reached flag
        #     self.tar_pos = tp
        #     self.target_reached = False
        #     self.log_info("TARGET: %.2f, %.2f\t CURRENT: %.2f, %.2f" % (x, y, self.curr_pos[0], self.curr_pos[1]))

    def update_tar_heading(self, msg):
        # if no data is coming from rudder controller, just jump to desired heading
        # otherwise, tar_heading is irrelevant to this node since the rudder controller is driving
        if not self.controlled:
            self.tar_heading = msg.data
            self.curr_heading = self.tar_heading

    def update_rudder(self, msg):
        self.log_info("Rudder message received!")
        turn_effectiveness = 1 # instantaneous boat turn per degree of rudder turn
        self.controlled = True # show that boat direction is being controlled
        if self.curr_heading is None: # in case heading was not previously set, init at 0
            self.curr_heading = 0
        self.curr_heading -= msg.data * turn_effectiveness # adjust heading with rudder 

    def update_tar_status(self, msg):
        self.target_reached = msg.data

    def disp_wind(self):
        print("WIND: %.2f" % self.wind_dir)

    def log_info(self, msg):
        print(msg)
        if self.log is not None:
            self.log.write(msg)


if __name__ == '__main__':
    b = Boat()
    b.run()
    exit()
