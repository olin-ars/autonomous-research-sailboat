# oars_pkg: OARS' ROS package

`/ROSdemo` is a demo/brief tutorial to check that ROS works after setup, and to go through some of the steps of creating a new node.

`/msg` is a folder for custom message types for ROS.

`/nav_and_controls` is a folder for code having to do with planning and executing boat maneuvers.

`/sensing` is a folder for code to collect and interpret sensor data and publish it to ROS.

`boat_class.py` is a template file, which could give structure to code dealing with the boat's coordinated functioning. If you're working on that, you should figure out what structure you need, *and then* look at this file to see if it is useful to you.

## Running all this mess

`cd` into `oars_ws` and `catkin_make`.

`source devel/setup.bash`. You may have to do this in both panes/tabs.

Assuming you're in docker, and have tmux, this is probably a relatively good way to run things:

Create 2 tabs/panes (`tmux`, then Ctrl+b Shift+5)

In one tab/pane: (Ctrl+b left arrow key)

`roslaunch oars_pkg boat.launch`

In the other tab/pane: (Ctrl+b right arrow key)

`rosrun oars_pkg command_center.py`

Now, in the first tab/pane, you can type commands to control the simulation. As of last commit, the nodes run by `boat.launch` will take care of everything except for the target position and wind direction. So, `tp=100,100 aw=0` will do the trick. At any time, you can provide commands to update conditions (like setting a new target.)

## More on the command center

Any nodes that provide information will publish, but where there is no source of information (like a starting position/heading or a target position) the "command center" node will publish whatever data you enter. Then, a "repeater" node will continue to publish it. This will create weird behavior if you publish data that is also coming from one of the nodes, so try not to do that.

Multiple simultaneous commands should be space separated, formatted like `foo=10 bar=20 bla=0` or `foo=10 bar=20,5` for tuple values like positions. The command names are as follows:

- Current position: 'current_position', 'current_pos', 'curr_pos', 'cp' 
- Target position: 'target_position', 'target_pos', 'tar_pos', 'tp'
- Current heading: 'current_heading', 'curr_heading', 'ch'
- Target heading: 'target_heading', 'tar_heading', 'th'
- Absolute wind direction: 'abs_wind_dir', 'absolute_wind', 'abs_wind', 'aw'
- Relative wind direction: 'rel_wind_dir', 'relative_wind', 'rel_wind', 'rw'
- Wind velocity: 'wind_velocity', 'wind_vel', 'wv', 'v'

Values of N/n/X/x will unset the given variable. For example:

`> foo=x bar=20` will clear `foo` and stop it publishing. A single X or N should be used for tuple values.


