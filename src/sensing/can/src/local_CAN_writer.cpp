#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <string.h>
#include <cstring>
#include <vector>
#include <tuple>

#include <mmc_msgs/object_array_msg.h>
#include <mmc_msgs/object_msg.h>
#include <mmc_msgs/lane_array_msg.h>
#include <mmc_msgs/lane_msg.h>
#include <mmc_msgs/chassis_msg.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

#define POSITION_DEC 10000

using namespace std;

class LOCAL_CAN_WRITER{
  public:
    ros::Subscriber sub1;
    ros::Publisher pub5;

    unsigned int kvaDb_flags = 0;
    unsigned int dlc = 8;

    double time1 = -1;
    float new_time = 0;
    int alive_count = 0;    

    bool local_calling = false;

    vector<tuple<char*, vector<char*>>> msg_list;

    LOCAL_CAN_WRITER();
    void CALLBACK_LOCAL(const mmc_msgs::to_control_team_from_local_msg& data);
    short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
    canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
    void LOOP();
};

LOCAL_CAN_WRITER::LOCAL_CAN_WRITER(){
  // List of CAN messages that you want to send to a CAN bus /////////////////////////////////////////////////

  msg_list.push_back(make_tuple((char*)"LOCAL_MAP_INFO",  vector<char*> {(char*)"is_stop_line",\
                                                                         (char*)"look_at_signalGroupID",\
                                                                         (char*)"look_at_IntersectionID",\
                                                                         (char*)"NEXT_LINK_ID",\
                                                                         (char*)"LINK_ID",\
                                                                         (char*)"distance_to_lane_end",\
                                                                         (char*)"have_to_LangeChange_left",\
                                                                         (char*)"have_to_LangeChange_right",\
                                                                         (char*)"left_LaneChange_avail",\
                                                                         (char*)"right_LaneChange_avail",\
                                                                         (char*)"Speed_Limit",\
                                                                         (char*)"guard_zone"}));

  msg_list.push_back(make_tuple((char*)"LANE_INFO_A",  vector<char*> {(char*)"TIME",\
                                                                 (char*)"ALLIVE_CNT",\
                                                                 (char*)"LANE_ID"}));

  msg_list.push_back(make_tuple((char*)"LANE_INFO_B",  vector<char*> {(char*)"M_ENTER",\
                                                                      (char*)"M_EXIT"}));

  msg_list.push_back(make_tuple((char*)"CAR_EGO_A",  vector<char*> {(char*)"X",\
                                                                   (char*)"Y"}));

  msg_list.push_back(make_tuple((char*)"CAR_EGO_B",  vector<char*> {(char*)"YAW",\
                                                                    (char*)"WAYPOINT"}));


  msg_list.push_back(make_tuple((char*)"CAR_EGO_SD",  vector<char*> {(char*)"EGO_S",\
                                                                     (char*)"EGO_D"}));

  msg_list.push_back(make_tuple((char*)"On_ODD_Stat",  vector<char*> {(char*)"On_ODD",\
                                                                     (char*)"Road_State",\
                                                                     (char*)"distance_out_of_ODD"}));

  msg_list.push_back(make_tuple((char*)"CAR_EGO_A_Ex",  vector<char*> {(char*)"X_High",\
                                                                     (char*)"Y_High"}));

}
 
void LOCAL_CAN_WRITER::CALLBACK_LOCAL(const mmc_msgs::to_control_team_from_local_msg& msg ){
  local_calling = true;

  alive_count += 1;
  if (alive_count > 255){
    alive_count = 0;
  }

  if (time1 == -1){
    time1 = msg.time.sec%10000 + msg.time.nsec/1000000000.0;
  }

  new_time = (msg.time.sec%10000 + msg.time.nsec/1000000000.0) - time1;
  
  unsigned char can_data[dlc];
  char* target_msg;
  char* target_msg2;
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  vector<double> temp_data2;
  unsigned int id_write, flag = 0;
  int re_value = 0;

  float east, north;
  short east_high, north_high;

  for(short i=0; i<6; i++){

    switch(i){

      case(0):
        target_msg = (char*)"LOCAL_MAP_INFO";
        temp_data = {(double)msg.is_stop_line,(double)msg.look_at_signalGroupID,(double)msg.look_at_IntersectionID,(double)msg.NEXT_LINK_ID,
        (double)msg.LINK_ID, msg.distance_to_lane_end,(double)msg.have_to_LangeChange_left,
        (double)msg.have_to_LangeChange_right,(double)msg.left_LaneChange_avail,
        (double)msg.right_LaneChange_avail,(double)msg.Speed_Limit};
      break;

      case(1):
        target_msg = (char*)"LANE_INFO_A";
        temp_data = {new_time, (double)alive_count, (double)msg.lane_id};
      break;

      case(2):
        // if((msg.host_north >=2012730.0) && (msg.host_north <2012731.0)) 
        {
          target_msg = (char*)"CAR_EGO_A";
          east_high = (short)(msg.host_east / POSITION_DEC);
          east = (float)(msg.host_east - east_high * POSITION_DEC);
          north_high = (short)(msg.host_north / POSITION_DEC);
          north = (float)(msg.host_north - north_high * POSITION_DEC);

          // temp_data = {(float)msg.host_east,  (float)msg.host_north};
          temp_data = {east, north};

          target_msg2 = (char*)"CAR_EGO_A_Ex";
          temp_data2 = {east_high, north_high};
        }
        
      break;

      case(3):
        target_msg = (char*)"CAR_EGO_B";
        temp_data = {(float)msg.host_yaw, (double)msg.waypoint_index};  
      break;

      case(4):
        target_msg = (char*)"CAR_EGO_SD";
        temp_data = {msg.station, msg.lateral_offset};
      break;

      case(5):
        target_msg = (char*)"On_ODD_Stat";
        temp_data = {(double)msg.On_ODD, (double)msg.Road_State, msg.distance_out_of_ODD};
      break;

    }

    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[j]);
    }
    // if(id_write == 1827)
    {
      re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
      memset(can_data, 0, sizeof(can_data));
    }

    if(id_write == 1827)
    {
      msg_idx = FIND_MSG_IDX(target_msg2, &msg_list);
      kvaDbGetMsgByName(dh, target_msg2, &mh);
      kvaDbGetMsgId(mh, &id_write, &flag);

      for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
        kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
        kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data2[j]);
      }

      re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
      memset(can_data, 0, sizeof(can_data));
      
    }


  }
}

short LOCAL_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
  short idx = -1;

  for(short i=0; i!=msg_list->size(); i++){

    if(strcmp(target_msg, get<0>(msg_list->at(i))) == 0){
      idx = i;
      break;
    }
  }

  if(idx != -1){
    return idx;

  }else{
    cout<<"No matched message! : "<<target_msg<<endl;
    return -1;
  }
}

canStatus LOCAL_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
  canInitializeLibrary();

  int open_flag;

  if(init_access_flag == true){
      open_flag = canOPEN_REQUIRE_INIT_ACCESS;

    }else{
      open_flag = canOPEN_NO_INIT_ACCESS;
    }

    cout<<"Opening the channel "<<channel_num<<"..."<<endl;
    hCAN = canOpenChannel(channel_num, open_flag);
  
  if(hCAN==canOK){
    cout<<"The CAN channel "<<channel_num<<" has been opened successfully..."<<endl;
  }
  
  can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
  can_status = canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
  can_status = canBusOn(hCAN);

  if(can_status == canOK){
    cout<<"The CAN bus is on..."<<endl;
  }

  kvaDb_status = kvaDbOpen(&dh);
  kvaDb_status = kvaDbReadFile(dh, filename);
  kvaDb_status = kvaDbGetFlags(dh, &kvaDb_flags);

  if(kvaDb_status == kvaDbOK){
    cout<<"Database file has been loaded successfully..."<<endl;
  }

  return canOK;
}

void LOCAL_CAN_WRITER::LOOP(){
  // ros::Rate rate(20);

  while(ros::ok()){
    // rate.sleep();
  }
}

int main(int argc, char **argv){
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "Local_CAN_Writer");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  strcpy(filename, (relative_path + "/dbc/CANdb_IONIQev_PCAN1.dbc").c_str());
  int channel_num = 0;
  bool init_access_flag = false;

  LOCAL_CAN_WRITER LCW;

  can_status = LCW.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  ros::Subscriber sub1 = node.subscribe("/localization/to_control_team", 1, &LOCAL_CAN_WRITER::CALLBACK_LOCAL, &LCW);

  LCW.pub5 = node.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/local_CAN_status", 10, true);
  jsk_rviz_plugins::OverlayText msg;
  string can_status_str;

  if(can_status == canOK){
    can_status_str = "OK";

  }else{
    can_status_str = "ERR";
  }

  msg.text.append("- CAN Local .... " + can_status_str + "\n");

  std_msgs::ColorRGBA state_color;
  
  int32_t width = 250;
  int32_t height = width*2;

  msg.action = msg.ADD;
  msg.font = "DejaVu Sans Mono";
  msg.text_size = 12;
  msg.width = width;
  msg.height = height;
  msg.left = 10;
  msg.top = 540;

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
  LCW.pub5.publish(msg);
  
  ros::waitForShutdown();   
  canBusOff(hCAN);
  canClose(hCAN);
}
