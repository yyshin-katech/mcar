cmake_minimum_required(VERSION 3.0.2)
project(spat_viewer)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/extract_map_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/serve_http.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
