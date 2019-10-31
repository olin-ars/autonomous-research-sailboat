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

        self.curr_heading = -1
        self.curr_pos = np.array([0, 0], dtype='float32')

        self.xs = []
        self.ys = []

        self.disp_wind()

        rospy.init_node('boat_sim', anonymous=True)

        rospy.Subscriber("target_position", Point32, self.update_tar_pos, queue_size=1)
        rospy.Subscriber("target_heading", Float32, self.update_tar_heading, queue_size=1)

        self.pub_heading = rospy.Publisher("current_heading", Float32, queue_size=1)
        self.pub_position = rospy.Publisher("current_position", Point32, queue_size=1)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            while not self.target_reached:
                r.sleep()
                self.update()
            print("Target reached!")
            plt.plot(self.xs, self.ys)
            plt.savefig("fig1.png")
            while self.target_reached:
                pass

    def update(self):
        if self.curr_heading > -1:
            self.curr_pos += 2 * np.array([np.cos(self.curr_heading * np.pi/180), np.sin(self.curr_heading * np.pi/180)])
        print("NEW Position: %.2f, %.2f\tHeading: %.2f" % (self.curr_pos[0], self.curr_pos[1], self.curr_heading))

        self.xs.append(self.curr_pos[0])
        self.ys.append(self.curr_pos[1])
        ch_msg = Float32(self.curr_heading)
        cp_msg = Point32(x=self.curr_pos[0], y=self.curr_pos[1], z=0)
        self.pub_heading.publish(ch_msg)
        self.pub_position.publish(cp_msg)

        path = self.tar_pos - self.curr_pos  # calculate the path
        print("Path", path)
        if np.sqrt(path[0] ** 2 + path[1] ** 2) < 5:  # re-calculate whether target has been reached.
            self.target_reached = True

    def update_tar_pos(self, msg):
        x, y, _ = msg.x, msg.y, msg.z
        tp = np.array([x, y])
        if (self.tar_pos != tp).any():  # if target changes, reset the target_reached flag
            self.tar_pos = tp
            self.target_reached = False
            print("TARGET: %.2f, %.2f\t CURRENT: %.2f, %.2f" % (x, y, self.curr_pos[0], self.curr_pos[1]))

    def update_tar_heading(self, msg):
        self.tar_heading = msg.data
        # a = self.curr_heading - self.tar_heading
        # b = self.curr_heading + 360 - self.tar_heading
        # if abs(a) < abs(b):
        #     change = a #/ 2
        # else:
        #     change = b #/ 2
        self.curr_heading = self.tar_heading  # += change

    def disp_wind(self):
        print("WIND: %.2f, %.2f" % (np.cos(self.wind_dir * np.pi / 180), np.sin(self.wind_dir * np.pi / 180)))


if __name__ == '__main__':
    b = Boat()
    b.run()
