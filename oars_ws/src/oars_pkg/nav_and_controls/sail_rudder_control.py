#!/usr/bin/env python
'''
THIS DOESN'T WORK RIGHT NOW
'''
import rospy
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Point32

class Headings:
    def __init__(self, usingRos = True):
        self.usingRos = True if rospy is not None and usingRos else False
        # Initialize each message. These won't publish if unset. 
        # These are subscriber vars
        self.curr_heading = None
        self.tar_heading = None
        self.rel_wind = None
        self.abs_wind = None
        # self.wv_msg = None

        self.target_reached = -1  # -1 = no target, 0 = not reached, 1 = reached

        # Heading vars
        self.sail_or = None
        self.rudder_or = None

        if self.usingRos:
            rospy.init_node('headings', anonymous = True)

            # Create a subscriber to the command topic for each value
            self.sub_ch = rospy.Subscriber('current_heading', Float32, self.recv_ch, queue_size = 1)
            self.sub_th = rospy.Subscriber('target_heading', Float32, self.recv_th, queue_size = 1)
            self.sub_rw = rospy.Subscriber('rel_wind_dir', Float32, self.recv_rw, queue_size = 1)
            self.sub_aw = rospy.Subscriber('abs_wind_dir', Float32, self.recv_aw, queue_size = 1)
            self.sub_ts = rospy.Subscriber("target_status", Int8, self.update_tar_status, queue_size=1)
            # self.sub_wv = rospy.Subscriber('cmd_wv', Float32, self.recv_wv, queue_size = 1)

            # Create a publisher to the corresponding topic for each value
            self.pub_sail_heading = rospy.Publisher('sail_position', Float32, queue_size = 1)
            self.pub_rudder_heading = rospy.Publisher('rudder_position', Float32, queue_size = 1)

    def run(self):
        print('Sail/rudder control node initialized.')

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            while self.target_reached == 0: # run if there's an unreached target
                # publish whichever values have been set
                self.publishCommands()
                r.sleep()
            while self.target_reached != 0: # wait for a new goal
                pass

    def calc_rudder_angle(self, ref_angle, target_angle):
        rudder_angle_scale = .25 # degrees of rudder turn per desired turn
        differences = [target_angle - ref_angle,
                       target_angle + 360 - ref_angle,
                       target_angle - ref_angle - 360]
        difference = min(differences, key=abs)
        return difference * rudder_angle_scale
    
    def calc_sail_angle(self):
        # figure out relative wind angle
        if self.rel_wind is not None:
            rel_wind = self.rel_wind
        elif self.abs_wind is not None and self.curr_heading is not None:
            rel_wind = (self.abs_wind - self.curr_heading) % 360
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
        self.curr_heading = msg.data

    def recv_th(self, msg):
        self.tar_heading = msg.data

    def recv_rw(self, msg):
        self.rel_wind = msg.data

    def recv_aw(self, msg):
        self.abs_wind = msg.data

    def recv_wv(self, msg):
        self.wv_msg = msg.data

    def update_tar_status(self, msg):
        self.target_reached = msg.data

    def publishCommands(self):
        # calculate sail position
        self.calc_sail_angle()
        # calculate rudder position if possible
        if self.curr_heading is not None and self.tar_heading is not None:
            self.rudder_or = self.calc_rudder_angle(self.curr_heading, self.tar_heading)
        # Publish whichever messages have been set
        if self.sail_or is not None:
            print("SAIL: %.1f" % self.sail_or)
            self.pub_sail_heading.publish(self.sail_or)
        if self.rudder_or is not None:
            print("RUDDER: %.1f" % self.rudder_or)
            self.pub_rudder_heading.publish(self.rudder_or)

if __name__ == '__main__':
    h = Headings()
    h.run()
    exit()
