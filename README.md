# OARS Software Team Repository
## A proposed new structure for the OARS software repository

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


** will be updated in the future with more information.
