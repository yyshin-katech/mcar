#include <ros/ros.h>
#include <novatel_gps_msgs/Inspva.h>
#include <mmc_msgs/localization2D_msg.h>
#include <proj.h>

class GpsToPose2D
{
public:
  GpsToPose2D()
  {

    sub_ = node_.subscribe("/sensors/gps/inspva", 10, &GpsToPose2D::inspvaCallback, this);
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


  void inspvaCallback(const novatel_gps_msgs::InspvaConstPtr& msg)
  {
    double east, north;
    latLonToEPSG5179(msg->latitude, msg->longitude, east, north);


    mmc_msgs::localization2D_msg pose_msg;
    pose_msg.time = ros::Time::now();  
    pose_msg.EPSG = 5179;              
    pose_msg.east = east;
    pose_msg.north = north;
    pose_msg.yaw = 1.57 - msg->azimuth * M_PI / 180.0;  


    pub_.publish(pose_msg);
  }

private:
  ros::NodeHandle node_;
  ros::Subscriber sub_;
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
