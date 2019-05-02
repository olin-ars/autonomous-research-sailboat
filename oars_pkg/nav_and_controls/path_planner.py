"""
This program implements the short course routing algorithm described in Section 5.1 of Roland Stelzer's Autonomous
Sailboat Navigation paper. This algorithm uses a polar efficiency function to determine the efficiency of all possible
angles that a boat can turn, and then compares the efficiencies to determine the optimal boat heading.
Read the paper here: https://www.dora.dmu.ac.uk/bitstream/handle/2086/7364/thesis-optimized-300dpi.pdf
@Authors: Duncan Mazza and Shreya Chowdhary
"""

import numpy as np
from time import sleep
from turtle import Turtle


def update_boat(boat, dir):
    boat.setheading(dir*180/np.pi)
    boat.forward(2)
    sleep(0.01)
    boat.position()


def draw_wind_boat(wind_dir, direction):
    # Draws the wind vector and positions/orients it correctly
    wind = Turtle()
    wind.pu()
    wind.setpos(250 * np.cos(wind_dir * np.pi / 180), 250 * np.sin(wind_dir * np.pi / 180))

    # Draw the boat vector and orient it
    boat = Turtle()
    boat.setheading(direction)

    return boat, wind


def target_draw_circle(target_pos, target_drawer):
    target_drawer.pu()
    target_drawer.goto(target_pos)
    target_drawer.pd()
    target_drawer.begin_fill()
    target_drawer.circle(10)
    target_drawer.end_fill()
    target_drawer.ht()


class ShortCoursePlanner:

    def __init__(self, p_c=40):
        self.p_c = p_c  # The beating parameter (which controls the length of a tack)

    def _get_polar_efficiency(self, wind_angle, alpha):
        """
        Calculates the hypothetical speed for a given angle alpha, using the polar efficiency function.

        :param wind_angle: the wind direction in degrees (relative to 0 degrees, with 0 degrees being east)
        :param alpha: angle being tested (relative to the wind angle)
        :return: the vector components for the velocity for the angle as a numpy array
        """
        angle = wind_angle + alpha

        # Calculate magnitude using the polar effeciency function
        mag = (1 - np.cos(alpha)) * (1 + 0.3 * np.cos(alpha)) / (1 - 0.5 * np.cos(alpha))

        # Return vector components as a numpy array
        res = np.array([mag * np.cos(angle), mag * np.sin(angle)])
        return res

    def _get_optima(self, wind_angle, path):
        """
        Finds the maximum velocity and angle that can be achieved for every possible new heading for the boat to
        determine the most optimal headings on the port and starboard sides of the boat.

        :param wind_angle: angle representing the wind.
        :param path: vector representing the path taken to reach from boat's position to the target position
        :return: A tuple containing the following information:
           windangle_max_r - max boat heading on right side
            windangle_max_l - max boat heading on left side
            vt_max_r - max velocity on right side
            vt_max_l - max velocity on left side
        """

        wind_angle *= np.pi/180

        # Initializing the right side and left side optimum wind angles
        windangle_max_r = wind_angle
        windangle_max_l = wind_angle

        # Initializing the right side and left side optimum max velocities
        vt_max_r = 0
        vt_max_l = 0

        # Checks all alphas between 0 and 180 and calculates the theoretical speeds
        # to find an optimal heading for a starboard tack
        for alpha in np.linspace(0, np.pi, 40):

            # Uses the polar effeciency function to calculcate the hypothetical velocity of the boat at that angle
            vb_hyp = self._get_polar_efficiency(wind_angle, alpha)

            # Projecting the max speed in the direction of the path
            vt_test = np.dot(vb_hyp, path)/np.sqrt(path[0] ** 2 + path[1] ** 2)

            # velocities[alpha] = vt_test

            # Check if this velocity is the new max
            if vt_test > vt_max_r:
                vt_max_r = vt_test
                windangle_max_r = (wind_angle + alpha) % (2*np.pi)

        # Analogous to the starboard tack, but for a port tack
        for alpha in np.linspace(np.pi, 2*np.pi, 40):
            vb_hyp = self._get_polar_efficiency(wind_angle, alpha)
            vt_test = np.dot(vb_hyp, path)/np.sqrt(path[0] ** 2 + path[1] ** 2)
            # velocities[alpha] = vt_test
            if vt_test > vt_max_l:
                vt_max_l = vt_test
                windangle_max_l = (wind_angle + alpha) % (2*np.pi)

        return vt_max_r, vt_max_l, windangle_max_r, windangle_max_l

    def _get_best_dir(self, vt_max_r, vt_max_l, windangle_max_r, windangle_max_l, path, boat_heading):
        """
        Checks whether a port tack or starboard tack would be more efficient.

        :param vt_max_r: the maximum velocity on the right hand side
        :param vt_max_l: the maximum velocity on the left hand side
        :param windangle_max_r: angle (in degrees) corresponding to the max velocity on the right hand side
        :param windangle_max_l: angle (in degrees) corresponding to the max velocity on the left hand side
        :param p_c: beating parameter (controls the length of the tacks)
        :param path: the path the boat is following as (a numpy array)
        :param boat_heading: the boat's heading, relative to 0 (with 0 as east)
        :return: the new boat direction and speed
        """
        # Calculates the hysteresis factor
        n = 1 + (self.p_c / (abs(np.sqrt(path[0] ** 2 + path[1] ** 2))))

        # Choose the direction that is closest to the current boat heading
        r_diff = np.pi/2 - abs(abs(windangle_max_r - boat_heading) - np.pi/2)
        l_diff = np.pi/2 - abs(abs(windangle_max_l - boat_heading) - np.pi/2)

        # Choose the closer side, balancing with faster velocity
        if r_diff < l_diff: # r is closer
            if vt_max_r * n < vt_max_l: # l is significantly faster
                new_boat_heading = windangle_max_l
            else:
                new_boat_heading = windangle_max_r # r is fast enough
        else:               # l is closer
            if vt_max_l * n < vt_max_r: # r is significantly faster
                new_boat_heading = windangle_max_r
            else:
                new_boat_heading = windangle_max_l # l is fast enough

        return new_boat_heading

    def run(self, target, w, b_h, b_p):
        """
        This method directs the boat by using the boat's position and the target position to calculate the path
        and comparing the boat's heading to the wind heading to determine the most efficient way for the boat to get
        to its target position.

        :param b_p - the boat's current position as a Python list
        :param b_h - the boat's current heading as an angle in degrees
        :param w - the wind angle in degrees, relative to 0 (with 0 as east)
        :return new_dir - the boat's new direction as a numpy array
        """

        # Calculates the path of the boat
        path = np.array(target) - np.array(b_p)

        # Runs the optimum function and compares the optima returned to determine the best direction for
        # boat to go
        vt_max_r, vt_max_l, windangle_max_r, windangle_max_l = self._get_optima(w, path)
        new_dir = self._get_best_dir( vt_max_r, vt_max_l, windangle_max_r, windangle_max_l, path, b_h * np.pi/180)

        return new_dir

if __name__ == "__main__":
    # Initialize planner object
    planner = ShortCoursePlanner()
    # Asks user to input a wind angle
    wind_dir = int(input("wind angle: "))
    # Initializes the boat's current direction, target path
    boat_curr_dir = 0
    # Initializes the list of waypoints (target positions)
    size = 200
    target_list = [(0, size), (size, size), (0, 0), (size, 0), (size, -size), (0, -size), (0, 0)]

    # Draw the boat; initialize boat Turtle object
    boat, wind = draw_wind_boat(wind_dir, boat_curr_dir)

    # Initialize the target drawer turtle object
    target_drawer = Turtle()
    target_drawer.pencolor("blue")
    target_drawer.fillcolor("blue")


    for target_pos in target_list:
        target_draw_circle(target_pos, target_drawer)

        print("Main: Current Target: ", target_pos)
        while boat.distance(target_pos) > 5:  # update the boat position
            boat_new_dir = planner.run(target_pos, wind_dir, boat.heading(), boat.position())
            print("Main: Boat new dir: ", boat_new_dir)
            update_boat(boat, boat_new_dir)
