cmake_minimum_required(VERSION 3.0.2)
project(mmc_msgs)

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
 association_msg.msg
 chassis_msg.msg
 lane_array_msg.msg
 lane_msg.msg
 matrix_msg.msg
 object_array_msg.msg
 object_msg.msg
 localization2D_msg.msg
 vehicle_cmd_msg.msg
 lane_occupancy.msg
 to_control_team_from_local_msg.msg
 from_control_team.msg
 V2V.msg
 path_history.msg
 ego_vel_acc.msg
 localization2D_sd_msg.msg
 ntrip_status.msg
 intersection_msg.msg
 intersection_array_msg.msg
 movement_msg.msg
 ioniq_motion_lane.msg
 ioniq_motion_lane_array.msg
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
