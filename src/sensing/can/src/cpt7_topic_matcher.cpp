#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/NovatelCorrectedImuData.h>
#include <novatel_gps_msgs/cpt_short_info.h>
#include <boost/optional.hpp> 

class GpsImuProcessor
{
public:
    GpsImuProcessor()
    {
        // Publisher
        pub_ = nh_.advertise<novatel_gps_msgs::cpt_short_info>("/cpt7_pose_topic", 10);

        // Subscribers
        inspva_sub_ = nh_.subscribe("/sensors/gps/inspva", 10, &GpsImuProcessor::inspvaCallback, this);
        corrimudata_sub_ = nh_.subscribe("/sensors/gps/corrimudata", 10, &GpsImuProcessor::corrimudataCallback, this);
    }

private:
    void inspvaCallback(const novatel_gps_msgs::Inspva::ConstPtr& msg)
    {
        inspva_data_ = *msg;
        publishData();
    }

    void corrimudataCallback(const novatel_gps_msgs::NovatelCorrectedImuData::ConstPtr& msg)
    {
        corrimudata_ = *msg;
        publishData();
    }

    void publishData()
    {
        if (inspva_data_ && corrimudata_)
        {
            novatel_gps_msgs::cpt_short_info output_msg;

            output_msg.header.stamp = ros::Time::now();
            output_msg.header.frame_id = "base_link";

            output_msg.longitude = inspva_data_->longitude;
            output_msg.latitude = inspva_data_->latitude;

            output_msg.pitch = inspva_data_->pitch;
            output_msg.roll = inspva_data_->roll;
            output_msg.azimuth = inspva_data_->azimuth;

            output_msg.pitch_rate = corrimudata_->pitch_rate;
            output_msg.roll_rate = corrimudata_->roll_rate;
            output_msg.yaw_rate = corrimudata_->yaw_rate;

            output_msg.longitudinal_acceleration = corrimudata_->longitudinal_acceleration;
            output_msg.lateral_acceleration = corrimudata_->lateral_acceleration;
            output_msg.vertical_acceleration = corrimudata_->vertical_acceleration;

            pub_.publish(output_msg);
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber inspva_sub_;
    ros::Subscriber corrimudata_sub_;

    boost::optional<novatel_gps_msgs::Inspva> inspva_data_; 
    boost::optional<novatel_gps_msgs::NovatelCorrectedImuData> corrimudata_; 
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cpt7_topic_matcher");
    GpsImuProcessor processor;
    ros::spin();
    return 0;
}
