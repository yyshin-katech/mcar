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
