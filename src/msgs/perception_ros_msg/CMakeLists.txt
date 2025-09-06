cmake_minimum_required(VERSION 3.0.2)
project(perception_ros_msg)

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
 AxisStatusPose.msg
 CoreInfo.msg
 Curve.msg
 EndPoints.msg
 FreeSpaceInfos.msg
 Indices.msg
 Lane.msg
 Lanes.msg
 LidarFrameMsg.msg
 Matrix3f.msg
 object_array_msg.msg
 object_msg.msg
 Object.msg
 Objects.msg
 Point2f.msg
 Point3d.msg
 Point3f.msg
 Point4f.msg
 Pose.msg
 PoseMap.msg
 RoadEdge.msg
 RoadEdges.msg
 RsPerceptionMsg.msg
 SupplementInfo.msg
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
