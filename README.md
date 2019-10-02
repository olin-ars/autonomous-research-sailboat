# OARS Software Team Repository
The ROS workspace for commanding and controlling OARS' autonomous research vessel.
> Olin Aquatic Robotic Systems @ Olin College.

## Setting up a Dev Environment
We use Docker Containers to maintain a sane and consistent environment. Please install Docker following the steps on https://hub.docker.com/search/?type=edition&offering=community for your operating system. If you're operating system is not listed, try Googling a solution.

You're now ready to use Docker! Docker's a pretty poweful tool used by many corporations, with numerous capabilities and features. It's both amazing and pretty overwhelming when getting started, so to help you get used to it, we've provided you some convenience commands. To get those loaded in, as well as the repo downloaded, do the following steps:
 - Clone the repo: `git clone https://github.com/olin-robotic-sailing/autonomous-research-sailboat.git`
 - `cd` into the folder to load the convenience start-up commands onto your machine: `cd autonomous-research-sailboat/docker`
 - Run the setup script! `bash d_setup_linux.sh`
 - Remove the repo (you don't need it anymore, since you'll be working on it from within the Docker container): `cd ../.. && rm -rf autonomous-research-sailboat`

And you're done! To learn more about Docker, why we're suggesting it, and what commands we've given you/what they do, go [here](docker/USAGE.md).

The Docker image used by OARS - `olinoars/ros:2019-2020` - is built using the `Dockerfile` and `ros_entrypoint.sh` files in the `docker/` folder.

### If you already have ROS and a nice editor
 - Clone the repo: `git clone https://github.com/olin-robotic-sailing/autonomous-research-sailboat.git`
 - `cd` into the `oars_ws` folder. This is a ROS workspace! Run `catkin_make` to build the project, and get started!

## Once ROS and the repo are installed
After you have done this, you should try the ROS demo in `oars_pkg/ROSdemo` to make sure your environment and package are configured correctly and to learn how to run a ROS node.

## What is all this stuff?
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
 - * will be updated in the future with more information.

## License
This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
