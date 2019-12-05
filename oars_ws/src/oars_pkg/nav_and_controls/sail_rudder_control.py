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
        self.aw_msg = None
        # self.wv_msg = None

        # Heading vars
        self.sail_or = None
        self.rudder_or = None

        if self.usingRos:
            rospy.init_node('headings', anonymous = True)
            print('Init heading node.')
            # Create a subscriber to the command topic for each value
            self.sub_current_heading = rospy.Subscriber('cmd_ch', Float32, self.recv_ch, queue_size = 1)
            self.sub_target_heading = rospy.Subscriber('cmd_th', Float32, self.recv_th, queue_size = 1)
            self.sub_rel_wind_dir = rospy.Subscriber('cmd_rw', Float32, self.recv_rw, queue_size = 1)
            self.sub_abs_wind_dir = rospy.Subscriber('cmd_aw', Float32, self.recv_aw, queue_size = 1)
            # self.sub_wind_velocity = rospy.Subscriber('cmd_wv', Float32, self.recv_wv, queue_size = 1)

            # Create a publisher to the corresponding topic for each value
            self.pub_sail_heading = rospy.Publisher('sail_position', Float32, queue_size = 5)
            self.pub_rudder_heading = rospy.Publisher('rudder_position', Float32, queue_size = 5)

            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                # publish whichever values have been set
                self.publishCommands()
                r.sleep()
                
    def calc_rudder_angle(self, ref_angle, target_angle):
        rudder_angle_scale = .25 # degrees of rudder turn per desired turn
        differences = [target_angle - ref_angle,
                       target_angle + 360 - ref_angle,
                       target_angle - ref_angle - 360]
        difference = min(differences, key=abs)
        return difference * rudder_angle_scale
    
    def calc_sail_angle(self):
        # figure out relative wind angle
        if self.rw_msg is not None:
            rel_wind = self.rw_msg
        elif self.aw_msg is not None and self.ch_msg is not None:
            rel_wind = (self.aw_msg - self.ch_msg) % 360
        else:
            return

        if rel_wind > 180: # want a range from 0-180, direction doesn't matter
            rel_wind = 360 - 180

        if rel_wind < 45: # in "irons" - you're fucked, but might as well have the sail in
            self.sail_or = 0
        elif rel_wind > 135: # heading downwind - sail all the way out
            self.sail_or = 90
        else:
            self.sail_or = rel_wind - 45 # ideal sail position

    # whenever a command is received, update the message being published by the repeater
    def recv_ch(self, msg):
        self.ch_msg = msg

    def recv_th(self, msg):
        self.th_msg = msg

    def recv_rw(self, msg):
        self.rw_msg = msg

    def recv_aw(self, msg):
        self.aw_msg = msg

    def recv_wv(self, msg):
        self.wv_msg = msg

    def publishCommands(self):
        if self.usingRos:
            # calculate sail position
            self.calc_sail_angle()
            # calculate rudder position if possible
            if self.ch_msg is not None and self.th_msg is not None:
                self.rudder_or = self.angle_diff(self.ch_msg, self.th_msg)
            # Publish whichever messages have been set
            if self.sail_or != None:
                self.pub_sail_heading.publish(self.sail_or)
            if self.rudder_or != None:
                self.pub_rudder_heading.publish(self.rudder_or)

if __name__ == '__main__':
    Headings()
