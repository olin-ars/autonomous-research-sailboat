"""
This code contains the AdafruitIMU class. This code should be run on a Raspberry Pi, because that is what the Adafruit
IMU connects to best. Ideally, when the Raspberry Pi is powered up, it will automatically run this script so that the
main computer can receive the information on the ROS node.

@Author(s): Noah
"""

import os
import rospy
from geometry_msgs.msg import Vector3
from Adafruit_BNO055.BNO055 import BNO055


class AdafruitIMUNode:
    """
    An object for interfacing with the Adafruit BNO055 9-DOF absolute orientation board
    (https://www.adafruit.com/product/2472)
    """

    def __init__(self, port):
        """
        Initialize the connection with the board
        :param port: the serial port to use when connecting to the BNO055
        """
        rospy.init_node('adafruit_imu', anonymous=True)
        rospy.loginfo('Starting Adafruit IMU node...')
        self.sensor = BNO055(serial_port=port, rst=1)

        # Initialize the BNO055
        if not self.sensor.begin():
            err_msg = 'Failed to connect to BNO055 on port {}. Is the IMU connected on that port?'.format(port)
            rospy.logerr(err_msg)
            raise RuntimeError(err_msg)

        # Print system status and self test result
        status, self_test, error = self.sensor.get_system_status()
        rospy.loginfo('System status: {0}'.format(status))
        rospy.loginfo('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))

        # Print out an error if system is in error mode
        if status == 0x01:
            err_msg = 'System error: {0}\nSee datasheet section 4.3.59 for the meaning.'.format(error)
            rospy.logerr(err_msg)
            raise RuntimeError(err_msg)

        self.pub = rospy.Publisher('/boat/imu', Vector3, queue_size=0)

        rospy.loginfo('Adafruit BNO055 IMU successfully initialized. Publishing on /boat/imu.')
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.publish_orientation()
            rate.sleep()

    def publish_orientation(self):
        """
        Reads and publishes the orientation to the ros node /boat/imu. The data structure is a ROS Vector3 message:
            float64 x - heading (degrees)
            float64 y - roll (degrees)
            float64 z - pitch (degrees)
        :return: None
        """
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        heading, roll, pitch = self.sensor.read_euler()
        self.pub.publish(Vector3(heading, roll, pitch))


if __name__ == '__main__':
    # Raspberry Pi configuration with serial UART and RST connected to GPIO 18 (use as default).
    serial_port = os.environ.get('ADAFRUIT_IMU_PORT', '/dev/ttyAMA0')
    AdafruitIMUNode(serial_port)