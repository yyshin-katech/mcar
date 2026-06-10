#include <ros/ros.h>
// #include <novatel_gps_msgs/Inspva.h>
#include <ublox_msgs/NavPVT.h>
#include <mmc_msgs/localization2D_msg.h>
#include <mmc_msgs/gps_time_msg.h>
#include <proj.h>
#include <algorithm>

class GpsToPose2D
{
public:
  GpsToPose2D()
  {

    // sub_ = node_.subscribe("/sensors/gps/inspva", 10, &GpsToPose2D::inspvaCallback, this);
    sub_ = node_.subscribe("/ublox/navpvt", 10, &GpsToPose2D::navpvtCallback, this);
    pub_ = node_.advertise<mmc_msgs::localization2D_msg>("/localization/pose_2d_gps", 1);
    time_pub_ = node_.advertise<mmc_msgs::gps_time_msg>("/localization/gps_time", 1);


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

  void navpvtCallback(const ublox_msgs::NavPVTConstPtr& msg)
  {
    double east, north;
    double latitude = msg->lat * 1e-7;
    double longitude = msg->lon * 1e-7;

    latLonToEPSG5179(latitude, longitude, east, north);

    mmc_msgs::localization2D_msg pose_msg;
    pose_msg.time = ros::Time::now();  
    pose_msg.EPSG = 5179;              
    pose_msg.east = east;
    pose_msg.north = north;
    pose_msg.altitude = msg->hMSL * 1e-3;  // mm -> m (MSL, 해발고도)

    double heading = msg->heading * 1e-5;
    pose_msg.yaw = 1.57 - heading * M_PI / 180.0;

    pub_.publish(pose_msg);

    // NavPVT UTC 시각 분해 → /localization/gps_time (CAN GPSTimestamp 송신/HMI 표시용)
    // (기존 local_CAN_writer 의 시각 추출 로직 이전. CAN 인코딩 year-2000 은 소비 측에서)
    mmc_msgs::gps_time_msg time_msg;
    time_msg.stamp = ros::Time::now();
    int second = msg->sec;
    int millisecond = msg->nano / 1000000;  // ns → ms
    if (millisecond < 0) {                   // nano 음수(초 미만) 보정
      millisecond += 1000;
      second -= 1;
    }
    millisecond = std::max(0, std::min(999, millisecond));
    time_msg.year = msg->year;
    time_msg.month = msg->month;
    time_msg.day = msg->day;
    time_msg.hour = msg->hour;
    time_msg.minute = msg->min;
    time_msg.second = second;
    time_msg.millisecond = millisecond;
    time_msg.valid = (msg->valid & ublox_msgs::NavPVT::VALID_TIME) != 0;
    time_pub_.publish(time_msg);
  }

  // void inspvaCallback(const novatel_gps_msgs::InspvaConstPtr& msg)
  // {
  //   double east, north;
  //   latLonToEPSG5179(msg->latitude, msg->longitude, east, north);


  //   mmc_msgs::localization2D_msg pose_msg;
  //   pose_msg.time = ros::Time::now();  
  //   pose_msg.EPSG = 5179;              
  //   pose_msg.east = east;
  //   pose_msg.north = north;
  //   pose_msg.yaw = 1.57 - msg->azimuth * M_PI / 180.0;  


  //   pub_.publish(pose_msg);
  // }

private:
  ros::NodeHandle node_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher time_pub_;


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
