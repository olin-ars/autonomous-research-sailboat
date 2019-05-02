#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32

class Repeater:
    def __init__(self, usingRos = True):
        self.usingRos = True if rospy is not None and usingRos else False

        self.cp_msg = None
        self.tp_msg = None
        self.ch_msg = None
        self.th_msg = None
        self.aw_msg = None
        self.rw_msg = None
        self.wv_msg = None

        if self.usingRos:
            rospy.init_node('command', anonymous = True)
            print('repeater node initialized.')

            self.sub_current_position = rospy.Subscriber('cmd_cp', Point32, self.recv_cp, queue_size = 1)
            self.sub_target_position = rospy.Subscriber('cmd_tp', Point32, self.recv_tp, queue_size = 1)
            self.sub_current_heading = rospy.Subscriber('cmd_ch', Float32, self.recv_ch, queue_size = 1)
            self.sub_target_heading = rospy.Subscriber('cmd_th', Float32, self.recv_th, queue_size = 1)
            self.sub_abs_wind_dir = rospy.Subscriber('cmd_aw', Float32, self.recv_aw, queue_size = 1)
            self.sub_rel_wind_dir = rospy.Subscriber('cmd_rw', Float32, self.recv_rw, queue_size = 1)
            self.sub_wind_velocity = rospy.Subscriber('cmd_wv', Float32, self.recv_wv, queue_size = 1)

            self.pub_current_position = rospy.Publisher('current_position', Point32, queue_size = 1)
            self.pub_target_position = rospy.Publisher('target_position', Point32, queue_size = 1)
            self.pub_current_heading = rospy.Publisher('current_heading', Float32, queue_size = 1)
            self.pub_target_heading = rospy.Publisher('target_heading', Float32, queue_size = 1)
            self.pub_abs_wind_dir = rospy.Publisher('abs_wind_dir', Float32, queue_size = 1)
            self.pub_rel_wind_dir = rospy.Publisher('abs_wind_dir', Float32, queue_size = 1)
            self.pub_wind_velocity = rospy.Publisher('wind_velocity', Float32, queue_size = 1)

            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                self.publishCommands()
                r.sleep()

    def recv_cp(self, msg):
        self.cp_msg = msg

    def recv_tp(self, msg):
        self.tp_msg = msg

    def recv_ch(self, msg):
        self.ch_msg = msg

    def recv_th(self, msg):
        self.th_msg = msg

    def recv_aw(self, msg):
        self.aw_msg = msg

    def recv_rw(self, msg):
        self.rw_msg = msg

    def recv_wv(self, msg):
        self.wv_msg = msg

    def publishCommands(self):
        n = 0
        if self.usingRos:
            if self.cp_msg != None:
                self.pub_current_position.publish(self.cp_msg); n+=1
            if self.tp_msg != None:
                self.pub_target_position.publish(self.tp_msg); n+=1
            if self.ch_msg != None:
                self.pub_current_heading.publish(self.ch_msg); n+=1
            if self.th_msg != None:
                self.pub_target_heading.publish(self.th_msg); n+=1
            if self.aw_msg != None:
                self.pub_abs_wind_dir.publish(self.aw_msg); n+=1
            if self.rw_msg != None:
                self.pub_rel_wind_dir.publish(self.rw_msg); n+=1
            if self.wv_msg != None:
                self.pub_wind_velocity.publish(self.wv_msg); n+=1
            print("%d commands published." % n)

if __name__ == '__main__':
    Repeater()