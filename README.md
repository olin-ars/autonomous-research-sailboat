# OARS Software Team Repository
## A proposed new structure for the OARS software repository

# Setting up a Dev Environment

In order to get a new development machine up and running, simply do the following
on a computer running Ubuntu 16.04:

  0) Open a terminal and optionally create a folder for OARS in your home directory and `cd` into it.
  1) Run `mkdir -p /catkin_ws/src && cd /catkin_ws/src`
  2) Clone the repo by running `git clone https://github.com/olin-robotic-sailing/oars_2019.git`
  3) Run `cd oars_2019 && bash setup` to run the setup script
  4) Run the command it says to run afterwards

The setup script will walk you through installing ROS and setting it up with Python
in a way that will not conflict with other Python configurations (e.g. Anaconda).
It will also give you the option to install the PyCharm integrated development
environment, which is great for debugging, and configure it for use with ROS.

Just enter your password when it asks for it and press Enter when it asks for the
PyCharm installation path (this will accept the default).


** will be updated in the future with more information.
