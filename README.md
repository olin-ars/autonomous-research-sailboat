# OARS Software Team Repository
A proposed new structure for the OARS software repository -
Olin Aquatic Robotic Systems @ Olin College.

# Setting up a Dev Environment
## If you don't have ROS installed (Standard Approach)
In order to get a new development machine up and running, simply do the following
on a computer running Ubuntu 16.04:

  0) Open a terminal and **optionally** create a folder for OARS in your home directory and `cd` into it.
  1) Run `mkdir -p oars_ws/src && cd oars_ws/src`
  2) Clone the repo by running `git clone https://github.com/olin-robotic-sailing/autonomous-research-sailboat.git`
  3) Run `cd oars_2019 && bash ./setup` to run the setup script

The setup script will walk you through installing ROS and setting it up with Python
in a way that will not conflict with other Python configurations (e.g. Anaconda).
It will also give you the option to install the PyCharm integrated development
environment, which is great for debugging, and configure it for use with ROS.

Just enter your password when it asks for it and press Enter when it asks for the
PyCharm installation path (this will accept the default).

## Docker Approach (If you're comfortable with basic terminal commands)
### For Ubuntu systems
Install Docker following the steps on https://docs.docker.com/install/linux/docker-ce/ubuntu/.

You're now ready to use Docker! Docker's a pretty poweful tool used by many corporations, with numerous capabilities and features. It's both amazing and pretty overwhelming when getting started, so to help you get used to it, we've provided you some convenience commands. To get those loaded in, as well as the repo downloaded, do the following steps:
 - Clone the repo: `git clone https://github.com/olin-robotic-sailing/autonomous-research-sailboat.git`
 - `cd` into the folder to load the convenience start-up commands onto your machine:
   - `cd autonomous-research-sailboat/docker_setup/`
 - Run the setup script! `bash d_setup_linux.sh`
 - Remove the repo (you don't need it anymore, since you'll be working on it from within the Docker container): `cd ../.. && rm -rf autonomous-research-sailboat`

And you're done! To learn more about Docker, why we're suggesting it, and what commands we've given you/what they do, go [here](docker_setup/USAGE.md).

The Docker image used by OARS - `olinoars/ros:2019-2020` - is built using the `Dockerfile` and `ros_entrypoint.sh` files in the `docker_setup/` folder.

## If you already have ROS and a nice editor
 - Clone the repo: `git clone https://github.com/olin-robotic-sailing/autonomous-research-sailboat.git`
 - `cd` into the `oars_ws` folder. This is a ROS workspace! Run `catkin_make` to build the project, and get started!

## Once ROS and the repo are installed
After you have done this, you should try the ROS demo in `oars_pkg/ROSdemo` to make sure your environment and package are configured correctly and to learn how to run a ROS node.

# What is all this stuff?

If this is your first time here, here are the main attractions in our repo. This is all very new and very much a work in progress, so there might not be much to see in some folders.
 - `oars_ws`  ROS workspace solely for OARS. You can `cd` inside this and immediately run `catkin_make`.
   - `build`  This is where all the compiled ROS node executables will be stored by default. Don't do work here. This folder will only appear after running `catkin_make`.
   - `devel`  This is the source code for the ROS building stuff. Don't do work here. This folder will only appear after running `catkin_make`.
   - `src`  The source code folder of the workspace. Do your ROS projects in here.
     - `oars_pkg/`  ROS nodes and code to run the boat. Your work probably goes here. *
       - `ROSdemo/`  information and code for playing with and learning about ROS
       - `launch/`  ROS launch scripts for starting multiple nodes
       - `msg/`  custom message types for ROS
       - `nav_and_controls/`  nodes relating to navigation and boat controls
       - `sensing/`  nodes relating to sensors
       - `boat_class.py`  the boat object which relies on sensing, navigation and controls
 - `run_boat.py`  template for a main program to start and run a boat. Use if useful. *
 - `setup`  setup script for creating your own dev environment - see the README *
 - `setup_root`  version of `setup` to run as root (without sudo) *
 - * will be updated in the future with more information.
