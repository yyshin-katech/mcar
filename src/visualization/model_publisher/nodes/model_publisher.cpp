#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mode_publisher") ;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string base_frame;
  private_nh.param<std::string>("base_frame", base_frame, "/ego_frame");

  std::string topic_name;
  private_nh.param<std::string>("topic_name", topic_name, "vehicle_model");
  
  std::string model_path;
  private_nh.param<std::string>("model_path", model_path, "package://model_publisher/model/genesis.dae");

  double offset_x;
  private_nh.param<double>("offset_x", offset_x, 1.3);

  double offset_y;
  private_nh.param<double>("offset_y", offset_y, 0);

  double offset_z;
  private_nh.param<double>("offset_z", offset_z, 0);

  double offset_roll;
  private_nh.param<double>("offset_roll", offset_roll, 90);

  double offset_yaw;
  private_nh.param<double>("offset_yaw", offset_yaw, 90);

  double offset_pitch;
  private_nh.param<double>("offset_pitch", offset_pitch, 0);

  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(topic_name, 10, true);

  // visualization_msgs::Marker marker;
  // marker.header.frame_id = base_frame;
  // marker.header.stamp = ros::Time::now();
  // marker.ns = topic_name;
  // marker.id = 0;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  // marker.mesh_resource = model_path;
  // // marker.mesh_resource = "package://model_publisher/model/genesis.dae";
  // marker.mesh_use_embedded_materials = true;
  // marker.pose.position.x = offset_x;
  // marker.pose.position.y = offset_y;
  // marker.pose.position.z = offset_z;
  // marker.pose.position.y = 0;
  // marker.pose.position.z = 0;
  // double roll = offset_roll * (M_PI / 180.0);
  // double yaw = offset_yaw * (M_PI / 180.0);
  // double pitch = offset_pitch * (M_PI / 180.0);
  // marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  // marker.color.r = 0.0;
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;
  // marker.color.a = 0.0;
  // marker.scale.x = 1.0;
  // marker.scale.y = 1.0;
  // marker.scale.z = 1.0;
  // marker.frame_locked = false;

  // pub.publish(marker);


  ros::Rate rate(100);
  while(ros::ok()){
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = topic_name;
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = model_path;
    // marker.mesh_resource = "package://model_publisher/model/genesis.dae";
    marker.mesh_use_embedded_materials = true;
    marker.pose.position.x = offset_x;
    marker.pose.position.y = offset_y;
    marker.pose.position.z = offset_z;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    double roll = offset_roll * (M_PI / 180.0);
    double yaw = offset_yaw * (M_PI / 180.0);
    double pitch = offset_pitch * (M_PI / 180.0);
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.frame_locked = true;

    pub.publish(marker);

    rate.sleep();
    
  }

  
  ros::spin();

  return 0;
}
