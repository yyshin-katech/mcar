cmake_minimum_required(VERSION 3.0.2)
project(j3224_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 DDateTime.msg
 DetectedObjectData.msg
 FullPositionVector.msg
 ObjectCommonData.msg
 PositionalAccuracy.msg
 sdsm.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES std_msgs
)

catkin_package(CATKIN_DEPENDS
 std_msgs
  message_runtime
)