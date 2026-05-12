cmake_minimum_required(VERSION 3.0.2)
project(web_hmi)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  perception_ros_msg
  mmc_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs perception_ros_msg mmc_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(web_hmi_threejs_tracks_cpp
  src/web_hmi_threejs_tracks_node.cpp
)
add_dependencies(web_hmi_threejs_tracks_cpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(web_hmi_threejs_tracks_cpp
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

install(TARGETS web_hmi_threejs_tracks_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/web_hmi_bridge.py
  scripts/web_hmi_threejs_bridge.py
  scripts/_demo_publisher.py
  scripts/web_server.py
  scripts/open_browser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Static web assets (HTML / JSX / CSS / vendored libs / fonts)
install(DIRECTORY web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
