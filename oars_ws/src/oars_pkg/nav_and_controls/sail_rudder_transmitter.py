'''
Listens to sail and rudder positioning messages, maps range of received values
to range of good PWM values, and writes a message with both to serial output.

PWM values should be calibrated before use - `sail_rudder_calibration.py` may be
helpful for this.

Intended to be used for communication via XBee modules, but should work with any
appropriately configured serial connection.

Author: Jane Sieving
'''

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
import serial

# Leaving these values pretty unrestricted until this is calibrated.
SAIL_MAX = 360  # 100
SAIL_MIN = -1   # 20
RUD_MAX = 360   # 120
RUD_MIN = -1    # 60


class CommandCenter:
    def __init__(self, usingRos=True):
        self.usingRos = True if rospy is not None and usingRos else False
        # Initialize each message. These won't publish if unset.
        self.cp_msg = None
        self.tp_msg = None

        if self.usingRos:
            rospy.init_node('command', anonymous=True)
            print('command node initialized.')
            # Create a publisher to the command topic for each value
            self.pub_current_position = rospy.Publisher('cmd_cp', Point32, queue_size=1)
            self.pub_target_position = rospy.Publisher('cmd_tp', Point32, queue_size=1)


            while not rospy.is_shutdown():
                # wait for commands to send at the command line
                self.getCommands()

    def getCommands(self):
        # example: > cp=0,0 tp=200,200 ch=90
        # get input
        line = raw_input('> ')
        # split into individual commands, and handle each one
        commands = line.split(' ')
        for command in commands:
            # instr[0] is the value name to set, instr[1] is its value
            instr = command.split('=')

        # publish whichever values have been set
        self.publishCommands()

    def publishCommands(self):
        n = 0
        if self.usingRos:
            # Publish whichever messages have been set
            if self.cp_msg is not None:
                self.pub_current_position.publish(self.cp_msg)
                n += 1
            if self.tp_msg is not None:
                self.pub_target_position.publish(self.tp_msg)
                n += 1
            print("%d commands published." % n)



if __name__ == "__main__":

    ser = serial.Serial('/dev/ttyUSB0')
    print(ser.name)


    rudder = 0 # set these between respective min and max values
    sail = 0



        ser.write(b'(%i,%i)' % (sail, rudder))

    ser.close()

