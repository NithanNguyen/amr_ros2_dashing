cmake_minimum_required(VERSION 3.5)
project(elder_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds config scripts models maps
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/robot_teleop.py
    scripts/imu_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


