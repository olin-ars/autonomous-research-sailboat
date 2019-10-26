#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This code uses the Ublox usb gps
Partially based on http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber

@Author(s): Jordan Crawford-O'Banner; small documentation revisions made by Duncan Mazza
"""

import rospy
from geometry_msgs.msg import Point32
import serial
import pynmea2
from numpy import float32
import doctest
import time


class Ublox(object):
    """

    """

    def __init__(self):
        """
        init function just gets an intial value so that is ros not surprised later
        """
        self.signal = False  # boolean for whether the gps signal is read correctly

        # Is the place that tehegps data is read from
        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=0.5)
        # reads a line from the serial terminal
        data = self.ser.readline()

        # GNGGA is the code for the data that holds the longitude and latitude data
        if data[0:6] == '$GNGGA':
            # pynmea2 is able to take in the line of data and put out just the longitude and latitude numbers
            msg = pynmea2.parse(data)

            # saves the latitude and longitude data
            self.latval = msg.lat
            self.longval = msg.long

    def publish_position(self):
        """
        Uses a Point32 ROS message type to send the gps position:
            float32 x - x is longitude
            float32 y - y is latitude
            float32 z - z is always set to zero

        Data from the device:
        Example: "$GNGLL,4217.59379,N,07115.87287,W,153652.00,A,A*62"
        Structure: "$GNGLL,ddmm.mmmm,char,ddmm.mmmm,char,hhmmss.ss,char,char*hexadecimal"
                   "$GLL_protocal_header,Latitude,North/South,Longitude,East/West,UCT_time,A(data_valid)/V(data_invalid)
                   ,Positioning_mode*checksum"

        Page 57 from here https://bit.ly/2IwAcKL explains in greater detail how the information is structured.

        :return: None
        """
        pub = rospy.Publisher('current_position', Point32, queue_size=0)
        rospy.init_node('current_position', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            # repeats the process in the init function over and over again
            while True:
                try:
                    data = self.ser.readline()
                    # print(data)
                except:  # not sure yet what exceptions could be raised
                    self.signal = False
                    continue
                if data[0:6] == '$GNGLL':  # filter out the other message types
                    if data[6:11] == ',,,,,':
                        # gps signal can't be obtained
                        self.signal = False
                        continue
                    else:
                        self.signal = True
                        msg = pynmea2.parse(data)  # separates data into latitude and longitude values

                        # if the first number of the latitude or longitude is 0 then the number is negative
                        if msg.lat[0] == "0":
                            msg.lat = "-" + msg.lat[1:]
                            self.latval = float32(msg.lat)
                        else:
                            self.latval = float32(msg.lat)
                        if msg.lon[0] == "0":
                            msg.lon = "-" + msg.lat[1:]
                            self.longval = float32(msg.lon)
                        else:
                            self.longval = float32(msg.lon)

                        # print(self.longval, self.latval)

                        time_log = time.time()
                        rospy.loginfo(Point32(self.longval, self.latval, time_log))
                        pub.publish(Point32(self.longval, self.latval, time_log))

                else:
                    continue
                rate.sleep()


if __name__ == '__main__':
    doctest.testmod()
    potemnkin = Ublox()
    try:
        potemnkin.publish_position()
    except rospy.ROSInterruptException:
        pass
