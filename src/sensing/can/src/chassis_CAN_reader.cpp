#include <can_pub.h>

int main(int argc, char **argv){
  
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "chassis_CAN_reader");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  
  strcpy(filename, (relative_path + "/dbc/CANdb_IONIQev_PCAN1.dbc").c_str());
  // strcpy(filename, (relative_path + "/dbc/ESR DV3_64Tgt-AS.dbc").c_str());
  int channel_num = 0;
  bool init_access_flag = false; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  pub0 = node.advertise<katech_diagnostic_msgs::k_adcu_diagnostic_msg>("/diagnostic/adcu", 1);
  pub1 = node.advertise<mmc_msgs::chassis_msg>("/sensors/chassis", 1);
  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);
  
  string can_status_str;


  if(can_status == canOK){
    
    CHASSIS_CAN_READER();
    
  }
  
  ros::waitForShutdown();
  canBusOff(hCAN);
  canClose(hCAN);
}