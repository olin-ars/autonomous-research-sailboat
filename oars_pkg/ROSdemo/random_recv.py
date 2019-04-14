#!/usr/bin/env python

''' Created as an example ROS node which only receives data. '''

import rospy
from std_msgs.msg import Float32, String
from oars_pkg.msg import ObjFrame


class ObstacleAvoider:
    def __init__(self, usingRos = True):
        self.usingRos = True if rospy is not None and usingRos else False
        self.obstacles = []

        if self.usingRos:
            ''' Register the node. This is one node for all publishers and subscribers in this class. '''
            rospy.init_node('obstacle_receiver', anonymous = True)

            ''' Initializes a subcriber. You can create multiple of these, and a given class can have publishers and/or subscribers.
            The storeNewObstacle is the name (not a function call) of the function to be called when a message is received. The message will be passed in as the input.
            '''
            rospy.Subscriber('/boat/obstacles', ObjFrame, self.storeNewObstacle, queue_size = 1)

            print('receiver node initialized.')

            # avoidObstacles is the method which carries out the main job of this class, using received data if available.
            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                self.avoidObstacles()
                r.sleep()

    def storeNewObstacle(self, msg):
        ''' Called when a ROS message is received with an ObjFrame message. You would have one method like this for each subscriber. '''

        print('Received obstacle message.')
        name = msg.name
        left = msg.left
        right = msg.right
        top = msg.top
        bottom = msg.bottom
        self.obstacles.append((name, left, right, top, bottom))
        # self.obstacles will save received data in this object so you can access it in avoidObstacles().

    def avoidObstacles(self):
        ''' This function is the sole purpose and objective of this class. Normally, it would be the bulk of your program. It takes data it has stored from received messages and does a thing. It runs repeatedly, but you can tell it not to run if there's not enough data. '''

        print("There are %d obstacles." % len(self.obstacles))
        print("Pretend I'm doing some computation, like helping you avoid the obstacles, because that's my job!")


if __name__ == '__main__':
    ObstacleAvoider()
