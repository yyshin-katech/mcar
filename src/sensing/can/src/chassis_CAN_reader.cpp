#include <can_pub.h>
#include <katech_diagnostic_msgs/k_adcu_diagnostic_msg.h>
#include <katech_custom_msgs/ioniq5_ad_can_msg.h>

ros::Publisher adcu_diag_pub;
uint8_t brain_life_count_cur = 0;
uint8_t brain_life_count_old = 0;
katech_diagnostic_msgs::k_adcu_diagnostic_msg adcu_diag_msg;

void adcu_diag_timer_callback(const ros::TimerEvent&)
{
    if(brain_life_count_cur == brain_life_count_old)
    {
        adcu_diag_msg.ADCU_SWC_StatCode = 1;
    }
    else
    {
        brain_life_count_old = brain_life_count_cur;
        adcu_diag_msg.ADCU_SWC_StatCode = 0;
        adcu_diag_msg.ADCU_AliveCount++;
    }
    adcu_diag_msg.time = ros::Time::now();
    adcu_diag_pub.publish(adcu_diag_msg);
}

void ioniq5_ad_can_callback(const katech_custom_msgs::ioniq5_ad_can_msg::ConstPtr& msg)
{
    brain_life_count_cur = msg->brain_life_count;
}

int main(int argc, char **argv){

  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "chassis_CAN_reader");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////

  strcpy(filename, (relative_path + "/dbc/CANdb_IONIQ5_AD_CAN_v3.dbc").c_str());
  // strcpy(filename, (relative_path + "/dbc/ESR DV3_64Tgt-AS.dbc").c_str());
  int channel_num = 0;
  bool init_access_flag = false; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  pub1 = node.advertise<katech_custom_msgs::ioniq5_ad_can_msg>("/sensors/ioniq5_ad_can", 1);

  // ADCU diagnostic: subscribe own topic and check BrainState life_count
  adcu_diag_pub = node.advertise<katech_diagnostic_msgs::k_adcu_diagnostic_msg>("/diagnostic/adcu", 1);
  ros::Subscriber adcu_sub = node.subscribe("/sensors/ioniq5_ad_can", 5, ioniq5_ad_can_callback);
  ros::Timer adcu_timer = node.createTimer(ros::Duration(0.1), adcu_diag_timer_callback);

  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  string can_status_str;


  if(can_status == canOK){

    CHASSIS_CAN_READER();

  }

  ros::waitForShutdown();
  canBusOff(hCAN);
  canClose(hCAN);
}
