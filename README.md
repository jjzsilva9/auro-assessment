# AURO

This implementation depends on only packages already available in the AURO VM.

Running the code is as simple as being in the directory, and running

colcon build --symlink-install && source install/local_setup.bash

then

ros2 launch solution solution_nav2_launch.py num_robots:=x

where x is the desired amount of robots (from 1-3).

The scenarios the implementation should be exercised on are as follows:

- 1 robot
- 2 robots
- 3 robots
(maybe sensor noise or no obstacles)