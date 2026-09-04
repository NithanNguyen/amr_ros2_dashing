# SIMULATION

1. Build the workspace:
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

colcon build --packages-select elder_robot



1. Source the workspace:
source install/setup.bash

1. Launch the robot in Gazebo:
ros2 launch elder_robot sim.launch.py

ros2 launch elder_robot sim_navigation.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard

