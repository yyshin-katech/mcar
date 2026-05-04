cmake_minimum_required(VERSION 3.0.2)
project(web_hmi)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

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
