# OARS Software Team Repository
A proposed new structure for the OARS software repository -
Olin Aquatic Robotic Systems @ Olin College.

# Setting up a Dev Environment

In order to get a new development machine up and running, simply do the following
on a computer running Ubuntu 16.04:

  0) Open a terminal and **optionally** create a folder for OARS in your home directory and `cd` into it.
  1) Run `mkdir -p oars_ws/src && cd oars_ws/src`
  2) Clone the repo by running `git clone https://github.com/olin-robotic-sailing/oars_2019.git`
  3) Run `cd oars_2019 && bash ./setup` to run the setup script

The setup script will walk you through installing ROS and setting it up with Python
in a way that will not conflict with other Python configurations (e.g. Anaconda).
It will also give you the option to install the PyCharm integrated development
environment, which is great for debugging, and configure it for use with ROS.

Just enter your password when it asks for it and press Enter when it asks for the
PyCharm installation path (this will accept the default).

After you have done this, you should try the ROS demo in `oars_pkg/ROSdemo` to make sure your environment and package are configured correctly and to learn how to run a ROS node.

# What is all this stuff?

If this is your first time here, here are the main attractions in our repo. This is all very new and very much a work in progress, so there might not be much to see in some folders.

* `oars_pkg/`  ROS nodes and code to run the boat. Your work probably goes here.
  - `ROSdemo/`  information and code for playing with and learning about ROS
  - `launch/`  ROS launch scripts for starting multiple nodes
  - `msg/`  custom message types for ROS
  - `nav_and_controls/`  nodes relating to navigation and boat controls
  - `sensing/`  nodes relating to sensors
  - `boat_class.py`  the boat object which relies on sensing, navigation and controls
* `run_boat.py`  template for a main program to start and run a boat. Use if useful.
* `setup`  setup script for creating your own dev environment - see the README
* `setup_root`  version of `setup` to run as root (without sudo)

** will be updated in the future with more information.
