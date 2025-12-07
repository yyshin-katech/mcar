#include <ros/ros.h>
#include <novatel_gps_msgs/Inspva.h>
#include <sensor_msgs/NavSatFix.h>
#include <ublox_msgs/NavPVT.h>
#include <mmc_msgs/localization2D_msg.h>
#include <proj.h>

class GpsToPose2D
{
public:
  GpsToPose2D()
  {

    // sub_ = node_.subscribe("/sensors/gps/inspva", 10, &GpsToPose2D::inspvaCallback, this);
    sub_ = node_.subscribe("/ublox/fix", 10, &GpsToPose2D::fixCallback, this);
    sub2_ = node_.subscribe("/ublox/navpvt", 10, &GpsToPose2D::pavpvtCallback, this);
    pub_ = node_.advertise<mmc_msgs::localization2D_msg>("/localization/pose_2d_gps", 1);

    C_proj = proj_context_create();
    P_proj = proj_create_crs_to_crs(C_proj, "EPSG:4326", "EPSG:5179", NULL);
  }

  ~GpsToPose2D()
  {
    proj_destroy(P_proj);
    proj_context_destroy(C_proj);
  }


  void latLonToEPSG5179(double lat, double lon, double &x, double &y)
  {
    PJ *norm_proj = proj_normalize_for_visualization(C_proj, P_proj);
    PJ_COORD a = proj_coord(lon, lat, 0, 0);
    PJ_COORD b = proj_trans(norm_proj, PJ_FWD, a);

    x = b.enu.e;
    y = b.enu.n;
  }

  void pavpvtCallback(const ublox_msgs::NavPVT& msg)
  {
    pose_msg.yaw = 1.57 - (msg.heading*1e-5) * M_PI / 180.0;  
    // pose_msg.yaw = -msg.heading * M_PI / 180.0;

    // while (pose_msg.yaw > M_PI) pose_msg.yaw -= 2.0 * M_PI;
    // while (pose_msg.yaw < -M_PI) pose_msg.yaw += 2.0 * M_PI;
  }

  void fixCallback(const sensor_msgs::NavSatFix& msg)
  {
    double east, north;
    latLonToEPSG5179(msg.latitude, msg.longitude, east, north);

    pose_msg.time = ros::Time::now();  
    pose_msg.EPSG = 5179;              
    pose_msg.east = east;
    pose_msg.north = north;

    pub_.publish(pose_msg);
  }

  mmc_msgs::localization2D_msg pose_msg;
  
private:
  ros::NodeHandle node_;
  ros::Subscriber sub_, sub2_;
  ros::Publisher pub_;


  PJ_CONTEXT *C_proj;
  PJ *P_proj;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_to_pose_2d_node");
  GpsToPose2D gps_to_pose_2d;
  ros::spin();
  return 0;
}
