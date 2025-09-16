cmake_minimum_required(VERSION 3.0.2)
project(katech_custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 ped_crosswalk_check_msg.msg
 ped_crosswalk_check_array_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES std_msgs
)

catkin_package(CATKIN_DEPENDS
 std_msgs
  message_runtime
)