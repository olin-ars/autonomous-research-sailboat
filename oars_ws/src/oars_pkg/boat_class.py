"""
This file contains the Boat() class that will be used to interpret information from the sensors and determine the
intended heading. At this point, it is unclear whether this class will encapsulate controls or if that will be handled
elsewhere.
"""

from mapping.map_class import Map

class Boat():

    def __init__(self, init_heading, init_pos, destination, map):
        """
        :param init_heading:
        :param init_pos: gps coordinates as a tuple of floats
        :param destination: gps coordinates as a tuple of floats
        :param map: a map object
        """
        self.current_heading = init_heading
        self.current_pos = init_pos
        self.destination = destination
        self.map = Map(init_pos)



