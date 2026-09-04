# SIMULATION

1. Build the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

colcon build --packages-select elder_robot
```


1. Source the workspace:
```bash
source install/setup.bash
```

2. Launch the robot in Gazebo:
```bash
# Terminal 1:
ros2 launch elder_robot sim_navigation.launch.py

# Terminal 2:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
