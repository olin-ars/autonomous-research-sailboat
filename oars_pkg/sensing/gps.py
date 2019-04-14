"""
This contains the GPS object, which is used by the Map class to parse the information from the GPS sensor.

@Author(s): Duncan Mazza
"""

import time


class GPS:
    """
    Parses the information from the GPS, which is used by the Map class.
    """

    def __init__(self):
        # TODO: Initialize connection to the GPS sensor

        self.current_pos = (42.293559, -71.263967)
        self.last_logged_localtime = time.localtime()  # contains the local time of the last time the gps was updated
        self.last_logged_time_sec = time.time()  # contains the time (sec) of the last time the gps was updated
        self.logged_pos = []  # list of read gps coordinates
        self.logged_times = []  # list of tuples of time.localtime() and time.time() at each gps update
        self.speed = 0

    def read_gps(self):
        """
        Every time this method is called, the information from the gps is parsed and self.current_pos set to these
        coordinates
        :return: self.current_pos
        """
        # TODO
        self.logged_pos.append(self.current_pos)
        self.last_logged_localtime = time.localtime()
        self.last_logged_time_sec = time.time()
        self.logged_times.append((self.last_logged_localtime, self.last_logged_time_sec))
        return self.current_pos

    def read_speed(self):
        """
        Updates the speed of of the boat based on the two most recently recorded gps positions and times
        :return: self.speed
        """
        # TODO
        return self.speed
