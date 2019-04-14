# ROSdemo instructions
After cloning this repo and running the setup script, you may want to run this demo to check that ROS works and go through some of the steps of creating a new node.

## Node & message files
The nodes used in this demo, `random_send.py` and `random_recv.py`, should be found in this directory (`ROSdemo`). They depend on the custom `ObjFrame` message type, which is defined in `oars_pkg/msg/ObjFrame.msg`.

## Setting up the package
In the `oars_pkg` directory, open `CMakeLists.txt`. Note: this is not the same as the `CMakeLists.txt` in the `src` directory. Find the line that says `add_message_files(` (it is probably commented out). Uncomment that block (down to the closing parenthesis), and replace `Message1.msg` and `Message2.msg` with `ObjFrame.msg`, leaving the word `FILES` at the top. **Any time you create a new message, you will need to add it here.**

Next, go to the line that says `generate_messages` and uncomment that whole block.

## Rebuilding the package
`cd` into `catkin_ws` and run `catkin_make`. If it gives you any errors, look back at `CMakeLists.txt` and make sure you didn't make any typos or syntax errors, and that you properly saved the file. Once that finishes, restart the terminal to allow the changes to take effect. You will also need to do this when you create new messages.

## Starting ROS
Remember to run `useoars` in each new terminal window to activate the virtual environment and quickly move into your OARS directory. To start ROS, run `roscore`. Then, open a new terminal tab (`Ctrl+Shift+T`) and run `rosrun oars_pkg random_send.py`. The node should start and print that it is publishing messages.

## Listening and `echo`
Open a new terminal tab and run `rosrun oars_pkg random_recv.py` to run the listener node. If you open another terminal and run `rostopic echo /boat/obstacles`, you can see the data being published by `random_send.py` in real time, without a listener node.

## A little more detail
`/boat/obstacles` is the topic name, or the name of the topic which `random_send.py` publishes to. `random_recv.py` does not publish to any topics, but it subcribes to `/boat/obstacles`. A single node can publish and subscribe to several topics.

In general, the workflow for making changes in ROS is:
- Add/edit source files in `oars_pkg` (nodes, messages, etc.)
- Update `oars_pkg/CMakeLists.txts` to look for new files or dependencies
- `cd` into `oars_ws` and run `catkin_make`

The common commands for starting ROS nodes are:
- `roscore`: start ROS
- `rosrun oars_pkg <node_file.py>`: start a node
- `rostopic echo </topic/name>`: listen to a certain topic

A lot of the steps of installing ROS, creating a catkin package, making it and setting up your environment are covered by the `setup` script in this repo. However, if something is not working or you would like to learn more about catkin workspaces, you can read some of the tutorials here: http://wiki.ros.org/catkin/Tutorials
