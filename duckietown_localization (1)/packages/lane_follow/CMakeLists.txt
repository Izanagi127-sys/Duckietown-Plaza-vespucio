cmake_minimum_required(VERSION 2.8.3)
project(lane_follow)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  duckietown_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
 
catkin_install_python(PROGRAMS
  src/lane_follow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
