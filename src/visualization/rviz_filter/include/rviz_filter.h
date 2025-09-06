#ifndef RVIZ_FILTER_
#define RVIZ_FILTER_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include <proj.h>
#include <perception_ros_msg/object_array_msg.h>
#include <perception_ros_msg/object_msg.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <perception_ros_msg/Point3f.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <j3224_msgs/sdsm.h>
#include <mmc_msgs/localization2D_msg.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>
#include <map>

class RVIZ_FILTER{

    public:
        RVIZ_FILTER();
        ~RVIZ_FILTER();

        ros::Subscriber sub1;
        ros::Subscriber sdsm_sub;
        ros::Subscriber local_sub;
        ros::Publisher marker_pub;

        void getOrientationFromDirection(visualization_msgs::Marker &merker, const double &orientation);
        void perception_info_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
        void percept_callback(const perception_ros_msg::object_array_msg::ConstPtr& msg);
        void sdsm_callback(const j3224_msgs::sdsm::ConstPtr& msg);
        void local_callback(const mmc_msgs::to_control_team_from_local_msg::ConstPtr& msg);
        bool wgs84_to_epsg5179(double lat_deg, double lon_deg, double& x_out, double& y_out);
        void loop(void);

        static void end(int sig);
    
    private:
        double host_east_ = 0.0;
        double host_north_ = 0.0;
        bool host_initialized_ = false;
};
#endif