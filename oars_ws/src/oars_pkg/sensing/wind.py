"""
This contains the WindVane object, which is used by the Map class to parse the information from the wind vane. This
also contains the Anemometer object that will parse information from the anemometer.

@Author(s): Duncan Mazza
"""


class WindVane:
    """
    Parses the information from the wind vane, which is used by the Map class.
    """

    def __init__(self, wind_speed_threshold=0.1):
        # TODO: Initialize connection to the wind vane

        # Initialize the anemometer object
        self.anemometer = Anemometer()

        # Initialize wind_direction attribute; wind direction is measured in degrees relative to the boat's polar
        # coordinate system, with 0 degrees pointing towards the bow of the boat.
        self.wind_direction = 0

        # Used by the should_ignore() method
        self.wind_speed_threshold = wind_speed_threshold
        # TODO: The wind_speed_threshold initial value should be calibrated through testing after the wind vane is
        #  acquired or made

    def read_wind_vane(self):
        """
        Every time this method is called, the information from the wind vane is parsed and turned into wind direction.
        :return: self.wind_direction (degrees relative to the boat's 0 degree axis)
        """
        # TODO
        return self.wind_direction

    def should_ignore(self):
        """
        If the wind is blowing too slowly, then the wind vane will provide meaningless data as it will depend more on
        the movement of the boat than the wind direction. Whether the data should be ignored will depend on the wind
        speed.
        :return: boolean
        """
        # TODO: Test whether this logic should be more complex or not
        self.anemometer.read_wind()
        if self.anemometer.wind_speed_m_s < self.wind_speed_threshold:
            return True
        else:
            return False


class Anemometer:
    """
    Parses the information from the anemometer
    """

    def __init__(self):
        # TODO: Initialize connection to the anemometer

        # Wind speed is usually measured in knots, but it will likely be most useful to us to have wind speed in m/s.
        # For now, both will be updated, and will always
        self.wind_speed_knot = 0
        self.wind_speed_m_s = 0

    def read_wind(self):
        """
        Parse the data from the anemometer to obtain a wind speed. Updates self.wind_speed_knot and wind_speed_m_s
        :return: wind_speed_m_s (floating point representing the wind speed in knots)
        """
        # TODO
        self.wind_speed_knot = 0.51444444 * self.wind_speed_m_s  # update self.wind_speed_knot
        return self.wind_speed_m_s
