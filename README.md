# Pick-and-Place-Using-A-Mobile-Base-Manipulator
## Instructions to build
Unzip 'mobile_manipulator.zip' to get 'mobile_manipulator'

This is a ROS2 package that can be built when placed into a ROS2 colcon workspace using 'colcon build'. The Odometry package must also be in the same workspace but is not provided here. 

## Instructions to run
After successfuly building and sourcing install/local_setup.bash, type in

'ros2 launch mobile_manipulator gazebo.launch.py'

to launch the world with pickup objects and the mobile base manipulator.

In order to run the pick and place node, enter 

'ros2 run mobile_manipulator pick_place_node.py'

into the terminal.
