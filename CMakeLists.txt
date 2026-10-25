cmake_minimum_required(VERSION 3.0.2)
project(robot_sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/obstacle_avoidance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
