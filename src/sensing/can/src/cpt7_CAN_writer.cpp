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

#include <std_msgs/Header.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/NovatelCorrectedImuData.h>
#include <novatel_gps_msgs/cpt_short_info.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

using namespace std;

class CPT7_CAN_WRITER{
  public:
    ros::Subscriber sub1;
    ros::Publisher pub5;

    unsigned int kvaDb_flags = 0;
    unsigned int dlc = 8;


    bool local_calling = false;

    vector<tuple<char*, vector<char*>>> msg_list;

    CPT7_CAN_WRITER();
    void CALLBACK_LOCAL(const novatel_gps_msgs::cpt_short_info& data);
    short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
    canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
    void LOOP();
};

CPT7_CAN_WRITER::CPT7_CAN_WRITER(){
  // List of CAN messages that you want to send to a CAN bus /////////////////////////////////////////////////

  msg_list.push_back(make_tuple((char*)"CPT7_Position_Lon",  vector<char*> {(char*)"cpt7_longitude"}));

  msg_list.push_back(make_tuple((char*)"CPT7_Position_Lat",  vector<char*> {(char*)"cpt7_latitude"}));

  msg_list.push_back(make_tuple((char*)"CPT7_Position_Azimuth",  vector<char*> {(char*)"cpt7_azimuth"}));

  msg_list.push_back(make_tuple((char*)"CPT7_Position_Yawrate",  vector<char*> {(char*)"cpt7_yawrate"}));

  msg_list.push_back(make_tuple((char*)"CPT7_Longitude_Acc",  vector<char*> {(char*)"cpt7_longAcc"}));

  msg_list.push_back(make_tuple((char*)"CPT7_Lateral_Acc",  vector<char*> {(char*)"cpt7_latAcc"}));

  msg_list.push_back(make_tuple((char*)"CPT7_Vertical_Acc",  vector<char*> {(char*)"cpt7_vertAcc"}));

  msg_list.push_back(make_tuple((char*)"CPT7_Position_RP",  vector<char*> {(char*)"cpt7_roll", \
                                                                           (char*)"cpt7_pitch"}));

  msg_list.push_back(make_tuple((char*)"CPT7_Position_RPRate",  vector<char*> {(char*)"cpt7_rollrate", \
                                                                               (char*)"cpt7_pitchrate"}));
}
 
void CPT7_CAN_WRITER::CALLBACK_LOCAL(const novatel_gps_msgs::cpt_short_info& msg ){
  local_calling = true;

//   alive_count += 1;
//   if (alive_count > 255){
//     alive_count = 0;
//   }

//   if (time1 == -1){
//     time1 = msg.time.sec%10000 + msg.time.nsec/1000000000.0;
//   }

//   new_time = (msg.time.sec%10000 + msg.time.nsec/1000000000.0) - time1;
  
  unsigned char can_data[dlc];
  char* target_msg;
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  unsigned int id_write, flag = 0;

  for(short i=0; i<9; i++){

    switch(i){

      case(0):
        target_msg = (char*)"CPT7_Position_Lon";
        temp_data = {msg.longitude};
      break;

      case(1):
        target_msg = (char*)"CPT7_Position_Lat";
        temp_data = {msg.latitude};
      break;

      case(2):
        target_msg = (char*)"CPT7_Position_Azimuth";
        temp_data = {msg.azimuth};
      break;

      case(3):
        target_msg = (char*)"CPT7_Position_Yawrate";
        temp_data = {msg.yaw_rate};  
      break;

      case(4):
        target_msg = (char*)"CPT7_Longitude_Acc";
        temp_data = {msg.longitudinal_acceleration};
      break;

      case(5):
        target_msg = (char*)"CPT7_Lateral_Acc";
        temp_data = {msg.lateral_acceleration};
      break;

      case(6):
        target_msg = (char*)"CPT7_Vertical_Acc";
        temp_data = {msg.vertical_acceleration};
      break;

      case(7):
        target_msg = (char*)"CPT7_Position_RP";
        temp_data = {msg.roll, msg.pitch};
      break;

      case(8):
        target_msg = (char*)"CPT7_Position_RPRate";
        temp_data = {msg.roll_rate, msg.pitch_rate};
      break;
    }

    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[j]);
    }

    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));

  }
}

short CPT7_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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

canStatus CPT7_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
  canInitializeLibrary();

  int open_flag;

  if(init_access_flag == true){
      open_flag = canOPEN_REQUIRE_INIT_ACCESS;

    }else{
      open_flag = canOPEN_NO_INIT_ACCESS;
    }

  // hCAN = canOpenChannel(channel_num, open_flag);

  // while(hCAN!=canOK){
    cout<<"Opening the channel "<<channel_num<<"..."<<endl;
    hCAN = canOpenChannel(channel_num, open_flag);
  // }
  
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

void CPT7_CAN_WRITER::LOOP(){
  ros::Rate rate(20);

  while(ros::ok()){
    rate.sleep();
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

  CPT7_CAN_WRITER LCW;

  can_status = LCW.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  ros::Subscriber sub1 = node.subscribe("/cpt7_pose_topic", 1, &CPT7_CAN_WRITER::CALLBACK_LOCAL, &LCW);
  
  ros::waitForShutdown();   
  canBusOff(hCAN);
  canClose(hCAN);
}

