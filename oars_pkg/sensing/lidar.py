"""
This contains the Lidar object, which is used by the Map class to parse information from the Lidar.
"""


class Lidar:
    """
    Parses the information from the Lidar, which is used by the Map class
    """

    def __init__(self, angle_bin_size=360):
        # TODO: Initialize connection to the Lidar
        # This parameter will control the fidelity of the data reported by this class. For example, this means that the
        # measured distance at each 360 / angle_bin_size degrees should be reported
        self.angle_bin_size = angle_bin_size
        self.lidar_data = []  # a list of distances at every 360 / angle_bin_size degrees

    def read_lidar(self):
        """
        Parse the data from the lidar to gain an understanding of the world around the boat. This data should represent
        the information gathered from one full sweep of the lidar's laser, and should be binned according to
        self.angle_bin_size (as described in __init__)
        :return: self.lidar_data (list of floating points of the measured distance [in meters] at every 360 /
        angle_bin_size degrees)
        """
        # TODO
        return
