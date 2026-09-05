# SIMULATION

1. Build the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

colcon build --packages-select elder_robot
```

2. Launch the robot in Gazebo:

## SLAM 
```bash
ros2 launch elder_robot sim_slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map
ros2 run nav2_map_server map_saver -f ~/ros2_ws/src/elder_robot/maps/my_map
```

## Navigation
```bash
ros2 launch elder_robot sim_navigation.launch.py
```

## Monitor
```bash
cd ~/ros2_ws/src/elder_robot/scripts
chmod +x ...
# SLAM
python3 monitor_sim_slam.py --nodes async_slam_toolbox_node robot_state_publisher
# Navigation
python3 monitor_sim_nav.py --nodes amcl dwb_controller bt_navigator navfn_planner ekf_filter_node local_costmap global_costmap robot_state_publisher imu_reader
# Power
python3 monitor_power_ros2.py
```
