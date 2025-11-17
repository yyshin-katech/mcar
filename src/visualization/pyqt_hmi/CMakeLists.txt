cmake_minimum_required(VERSION 3.0.2)
project(pyqt_hmi)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  mmc_msgs
  katech_diagnostic_msgs
  v2x_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main_display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)