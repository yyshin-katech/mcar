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
  scripts/main_display_a1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Python 모듈 설치
install(DIRECTORY scripts/widgets scripts/widgets_a1 scripts/utils
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

# 리소스 (QSS / fonts) 설치
install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)