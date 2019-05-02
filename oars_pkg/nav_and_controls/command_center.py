#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32

class CommandCenter:
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
            print('command node initialized.')

            self.pub_current_position = rospy.Publisher('cmd_cp', Point32, queue_size = 1)
            self.pub_target_position = rospy.Publisher('cmd_tp', Point32, queue_size = 1)
            self.pub_current_heading = rospy.Publisher('cmd_ch', Float32, queue_size = 1)
            self.pub_target_heading = rospy.Publisher('cmd_th', Float32, queue_size = 1)
            self.pub_abs_wind_dir = rospy.Publisher('cmd_aw', Float32, queue_size = 1)
            self.pub_rel_wind_dir = rospy.Publisher('cmd_rw', Float32, queue_size = 1)
            self.pub_wind_velocity = rospy.Publisher('cmd_wv', Float32, queue_size = 1)

            while not rospy.is_shutdown():
                self.getCommands()

    def getCommands(self):
        line = raw_input('> ')
        commands = line.split(' ')
        for command in commands:
            instr = command.split('=')
            if instr[0] in ['current_position', 'curr_pos', 'cp']:
                values = instr[1].split(',')
                if len(values) != 2:
                    print("Wrong number of arguments for current_position: expected 2.\n")
                    continue
                x = float(values[0])
                y = float(values[1])
                print("current_position set to (%f, %f)." % (x, y))
                self.cp_msg = Point32(x, y, 0)

            elif instr[0] in ['target_position', 'target_pos', 'tp']:
                values = instr[1].split(',')
                if len(values) != 2:
                    print("Wrong number of arguments for target_position: expected 2.\n")
                    continue
                x = float(values[0])
                y = float(values[1])
                print("target_position set to (%f, %f)." % (x, y))
                self.tp_msg = Point32(x, y, 0)

            elif instr[0] in ['current_heading', 'curr_heading', 'ch']:
                print("current_heading set to %s." % instr[1])
                self.ch_msg = Float32(float(instr[1]))

            elif instr[0] in ['target_heading', 'th']:
                print("target_heading set to %s." % instr[1])
                self.th_msg = Float32(float(instr[1]))

            elif instr[0] in ['abs_wind_dir', 'abs_wind', 'aw']:
                print("abs_wind_dir set to %s." % instr[1])
                self.aw_msg = Float32(float(instr[1]))

            elif instr[0] in ['rel_wind_dir', 'rel_wind', 'rw']:
                print("rel_wind_dir set to %s." % instr[1])
                self.rw_msg = Float32(float(instr[1]))

            elif instr[0] in ['wind_velocity', 'wind_vel', 'v']:
                print("wind_velocity set to %s." % instr[1])
                self.wv_msg = Float32(float(instr[1]))
            else:
                print("Unrecognized command: " + instr[0])
        self.publishCommands()

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
    CommandCenter()
