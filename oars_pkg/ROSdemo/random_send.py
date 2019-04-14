#!/usr/bin/env python

''' Created as an example ROS node which only sends data. '''

import rospy
from std_msgs.msg import Float32, String
from oars_pkg.msg import ObjFrame

from random import choice, random


class ObstacleReporter:
    def __init__(self, usingRos = True):
        self.usingRos = True if rospy is not None and usingRos else False

        if self.usingRos:
            ''' Register the node. This is one node for all publishers and subscribers in this class. '''
            rospy.init_node('obstacle_reporter', anonymous = True)
            print('reporter node initialized.')
            ''' Initializes a publisher. You can create multiple of these, and a given class can have publishers and/or subscribers. '''
            self.pub = rospy.Publisher('/boat/obstacles', ObjFrame, queue_size = 1)

            print('reporter node initialized.')

            '''makeDummyObstacle is the method which carries out the main job of this class. It also triggers data publishing. '''
            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                self.makeDummyObstacle()
                r.sleep()

    def createObstacle(self, N, L, R, T, B):
        ''' Take data (probably several parameters), do computations on them if necessary, format them into the right data types for a message type, and then package them into that message.

        You might have multiple of these methods if you're publishing multiple messages to multiple topics. '''

        # You'll use something like these formats, depending on the data/message structure.
        name = String(N)
        left = Float32(L)
        right = Float32(R)
        top = Float32(T)
        bottom = Float32(B)

        msg = ObjFrame(name = name, left = left, right = right, top = top, bottom = bottom)

        ''' The names of components and the contents expected for their values can be found in the documentation for the message type. http://wiki.ros.org/std_msgs explains the basic standard message types.'''
        return msg

    def makeDummyObstacle(self):
        ''' This function is the sole purpose and objective of this class. Normally it would be the bulk of your program. '''

        objects = ["Car", "Boat", "Tree", "Cloud"]
        name = choice(objects)
        left = random() * 180
        right = random() * 180
        top = random() * 180
        bottom = random() * 180

        if self.usingRos:
            ''' When your program has done all the computations you pay it to do, it packages up messages and publishes them through the appropriate publishers.'''
            msg = self.createObstacle(name, left, right, top, bottom)
            self.pub.publish(msg)
            print("obstacle published.")

if __name__ == '__main__':
    ObstacleReporter()
