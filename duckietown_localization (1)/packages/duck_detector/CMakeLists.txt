cmake_minimum_required(VERSION 2.8.3)
project(duck_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  duckietown_msgs
  message_runtime
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs duckietown_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  src/duck_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


