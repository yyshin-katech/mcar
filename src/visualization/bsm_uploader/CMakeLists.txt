cmake_minimum_required(VERSION 3.0.2)
project(bsm_uploader)

find_package(catkin REQUIRED COMPONENTS
  rospy
  novatel_gps_msgs
  katech_custom_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy novatel_gps_msgs katech_custom_msgs
)

catkin_install_python(PROGRAMS
  scripts/bsm_uploader_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
