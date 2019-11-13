#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32

class Headings:
    def __init__(self, usingRos = True):
        self.usingRos = True if rospy is not None and usingRos else False
        # Initialize each message. These won't publish if unset. 
        # These are subscriber vars
        self.ch_msg = None
        self.th_msg = None
        self.rw_msg = None
        # self.wv_msg = None

        # Heading vars
        self.sail_or = None
        self.rudder_or = None
        self.aligned = None

        if self.usingRos:
            rospy.init_node('headings', anonymous = True)
            print('Init heading node.')
            # Create a subscriber to the command topic for each value
            self.sub_current_heading = rospy.Subscriber('cmd_ch', Float32, self.recv_ch, queue_size = 1)
            self.sub_target_heading = rospy.Subscriber('cmd_th', Float32, self.recv_th, queue_size = 1)
            self.sub_rel_wind_dir = rospy.Subscriber('cmd_rw', Float32, self.recv_rw, queue_size = 1)
            # self.sub_wind_velocity = rospy.Subscriber('cmd_wv', Float32, self.recv_wv, queue_size = 1)

            # Create a publisher to the corresponding topic for each value
            self.pub_sail_heading = rospy.Publisher('/sail_heading', Float32, queue_size = 5)
            self.pub_rudder_heading = rospy.Publisher('/rudder_heading', Float32, queue_size = 5)

            r = rospy.Rate(20)
            while not rospy.is_shutdown():
                # publish whichever values have been set
                self.publishCommands()
                r.sleep()

    # whenever a command is received, update the message being published by the repeater
    def recv_ch(self, msg):
        self.ch_msg = msg

    def recv_th(self, msg):
        self.th_msg = msg
        if (abs(self.ch_msg - self.th_msg) > 5):
            self.aligned = False
        else:
            self.aligned = True

    def recv_rw(self, msg):
        self.rw_msg = msg
        if (self.th_msg > 180):
            if (self.rw_msg > 300 or self.rw_msg < 55):
                self.sail_or = self.rw_msg + 5
            elif (self.rw_msg < 235 or self.rw_msg > 120):
                self.sail_or = self.rw_msg - 5
        else:
            if (self.rw_msg > 300 or self.rw_msg < 55):
                self.sail_or = self.rw_msg - 5
            elif (self.rw_msg < 235 or self.rw_msg > 120):
                self.sail_or = self.rw_msg + 5
        if self.aligned:
            self.rudder_or = self.sail_or
        else:
            if (self.th_msg - self.ch_msg) > 0:
                self.rudder_or = 180
            else:
                self.rudder_or = 0

    def recv_wv(self, msg):
        self.wv_msg = msg

    def publishCommands(self):
        if self.usingRos:
            # Publish whichever messages have been set
            if self.sail_or != None:
                self.pub_sail_heading.publish(self.sail_or)
            if self.rudder_or != None:
                self.pub_rudder_heading.publish(self.rudder_or)

if __name__ == '__main__':
    Headings()
