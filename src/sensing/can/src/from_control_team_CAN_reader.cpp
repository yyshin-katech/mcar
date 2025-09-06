#include <can_pub.h>

int main(int argc, char **argv){
  
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "from_control_team_CAN_reader");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/sensor_fusion_result_revised.dbc").c_str());
  int channel_num = 2;
  bool init_access_flag = false; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  pub1 = node.advertise<mmc_msgs::from_control_team>("/from_control_team", 1);
  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);
  
  if(can_status == canOK){
    FROM_CONTROL_TEAM_CAN_READER();
  }
  
  ros::waitForShutdown();
  canBusOff(hCAN);
  canClose(hCAN);
}