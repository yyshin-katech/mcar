cmake_minimum_required(VERSION 3.0.2)
project(v2x_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 intersection_msg.msg
 intersection_array_msg.msg
 movement_msg.msg
 v2x_go_ahead_msg.msg
 v2x_pedes_assist_msg.msg
 v2x_tim_content_msg.msg
 v2x_tim_region_msg.msg
 v2x_tim_dataframe_msg.msg
 v2x_tim_regional_msg.msg
 v2x_tim_can_go_msg.msg
 v2x_tim_total_msg.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 sensor_msgs
 std_msgs
)

catkin_package(CATKIN_DEPENDS
 sensor_msgs
 std_msgs
message_runtime
)
