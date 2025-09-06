cmake_minimum_required(VERSION 3.0.2)
project(katech_diagnostic_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 cpt7_gps_diagnostic_msg.msg
 k_adcu_diagnostic_msg.msg
 lidar_diagnostic_msg.msg
 radar_diagnostic_msg.msg
 v2x_diagnostic_msg.msg
 hmi_diagnostic_msg.msg
 vcu_diagnostic_msg.msg
 cam_diagnostic_msg.msg
 ipc_diagnostic_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 geometry_msgs
 nav_msgs
 sensor_msgs
 std_msgs
 visualization_msgs
)

catkin_package(CATKIN_DEPENDS
 geometry_msgs
 nav_msgs
 sensor_msgs
 std_msgs
 visualization_msgs
  message_runtime
)