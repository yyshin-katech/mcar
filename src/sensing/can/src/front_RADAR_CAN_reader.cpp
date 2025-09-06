#include <can_pub.h>

int main(int argc, char **argv){
  
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "front_RADAR_CAN_reader");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/ESR DV3_64Tgt-AS.dbc").c_str());

  int channel_num = 0;
  bool init_access_flag = false; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  radar_pub = node.advertise<mmc_msgs::object_array_msg>("/sensors/front_radar", 1);
  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  if(can_status == canOK){
    FRONT_RADAR_CAN_READER();
  }
  
  ros::waitForShutdown();   
  canBusOff(hCAN);
  canClose(hCAN);
}