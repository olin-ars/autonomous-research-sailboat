# Navigation and Controls

Scripts having to do with planning and executing boat maneuvers go here.

`path_planner.py` repeatedly computes the optimal heading for a boat to reach a waypoint, taking wind direction into account. **TODO:** This currently runs a Turtle simulation. It should be copied and converted to a ROS node, ideally listening to the topics that `repeater.py` publishes to.

`repeater.py` listens to user-published updates of dummy environment information (wind direction, boat orientation, etc.) and publishes it repeatedly.

`command_center.py` allows the user to provide dummy environment information to `repeater.py`. This can help scripts to run for testing, in the absence of some sensors.
