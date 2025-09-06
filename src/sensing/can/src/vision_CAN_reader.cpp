#include <can_pub.h>

int main(int argc, char **argv){
  
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "vision_CAN_reader");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/180518_Camera_P_CAN_R01.dbc").c_str());
  int channel_num = 0;
  bool init_access_flag = false; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  pub1 = node.advertise<mmc_msgs::lane_array_msg>("/sensors/vision_lane", 1);
  pub2 = node.advertise<mmc_msgs::object_array_msg>("/sensors/vision_pos", 1);
  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);
  
  // overlay text /////////////////////////////////////////////////////////////////
  pub3 = node.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/vision_CAN_status", 10, true);
  jsk_rviz_plugins::OverlayText msg;
  string can_status_str;

  if(can_status == canOK){
    can_status_str = "OK";

  }else{
    can_status_str = "ERR";
  }

  msg.text.append("- CAN Vision ... " + can_status_str + "\n");

  std_msgs::ColorRGBA state_color;
  
  int32_t width = 250;
  int32_t height = width*2;

  msg.action = msg.ADD;
  msg.font = "DejaVu Sans Mono";
  msg.text_size = 12;
  msg.width = width;
  msg.height = height;
  msg.left = 10;
  msg.top = 480;

  state_color.r = 1;
  state_color.g = 1;
  state_color.b = 1;
  state_color.a = 1;
  msg.fg_color = state_color;

  state_color.r = 0;
  state_color.g = 0;
  state_color.b = 0;
  state_color.a = 0.0;
  msg.bg_color = state_color;
  pub3.publish(msg);

  if(can_status == canOK){
    VISION_CAN_READER();
  }
  
  ros::waitForShutdown();   
  canBusOff(hCAN);
  canClose(hCAN);
}