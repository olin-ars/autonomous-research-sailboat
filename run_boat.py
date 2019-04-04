"""
Run the boat with this file
"""

import os.system
from oars_pkg.boat_class import Boat
from oars_pkg.mapping.map_class import Map


def main():
    # Run launch files to set up the ROS nodes. Make sure you have ros set up on your system.
    os.system("roslaunch launch/")

    boat = Boat()


if __name__ == "__main__":
    main()
