cmake_minimum_required(VERSION 2.8.3)
project(vision_superior_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  duckietown_msgs
  message_runtime
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/vision_superior_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
