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

#include <v2x_msgs/intersection_msg.h>
#include <v2x_msgs/intersection_array_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

using namespace std;

class SPAT_CAN_WRITER{
  public:
    ros::Subscriber sub1;

    unsigned int kvaDb_flags = 0;
    unsigned int dlc = 8;

    double time1 = -1;
    float new_time = 0;
    int alive_count = 0;

    bool spat_calling = false;

    vector<tuple<char*, vector<char*>>> msg_list;

    SPAT_CAN_WRITER();
    void CALLBACK_SPAT(const v2x_msgs::intersection_array_msg& data);
    short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
    canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
    void LOOP();
};

SPAT_CAN_WRITER::SPAT_CAN_WRITER(){
  // List of CAN messages that you want to send to a CAN bus /////////////////////////////////////////////////

  msg_list.push_back(make_tuple((char*)"V2X_SPaT_1",  vector<char*> {(char*)"movementName_1",\
                                                                    (char*)"minEndTime_1",\
                                                                    (char*)"eventState_1",\
                                                                    (char*)"signalGroup_1",\
                                                                    (char*)"Intersection_ID_1"}));
}

void SPAT_CAN_WRITER::CALLBACK_SPAT(const v2x_msgs::intersection_array_msg& msg){
  spat_calling = true;
  alive_count = (alive_count + 1) % 256;

  if (time1 == -1)
    time1 = msg.time.sec%10000 + msg.time.nsec/1000000000.0;
  new_time = (msg.time.sec%10000 + msg.time.nsec/1000000000.0) - time1;

  unsigned char can_data[dlc];
  memset(can_data, 0, sizeof(can_data));

  char* target_msg = (char*)"V2X_SPaT_1";
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  unsigned int id_write, flag = 0;
  vector<double> temp_data;

  // siheung_v2x가 이미 현재 링크 기반으로 필터링하여 발행
  // data[0]에 매칭된 신호 또는 전체 0 메시지가 들어옴
  if (!msg.data.empty() && msg.data[0].IntersectionID != 0)
  {
    auto& d = msg.data[0];
    temp_data = {0.0,
                 (double)d.Movements.TimeChangeDetails,
                 (double)d.Movements.MovementPhaseStatus,
                 (double)d.Movements.SignalGroupID,
                 (double)d.IntersectionID};

    ROS_INFO("[SPaT CAN] IntID=%d SigGrp=%d Phase=%d minEnd=%.1fs",
             d.IntersectionID, d.Movements.SignalGroupID,
             d.Movements.MovementPhaseStatus,
             d.Movements.TimeChangeDetails / 10.0);
  }
  else
  {
    // 신호 없는 링크: 전부 0
    temp_data = {0.0, 0.0, 0.0, 0.0, 0.0};
  }

  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[j]);
  }
  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
}

short SPAT_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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

canStatus SPAT_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
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

void SPAT_CAN_WRITER::LOOP(){
  // ros::Rate rate(20);

  while(ros::ok()){
    // rate.sleep();
  }
}

int main(int argc, char **argv){
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "SPaT_CAN_Writer");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  strcpy(filename, (relative_path + "/dbc/CANdb_IONIQev_PCAN1.dbc").c_str());
  int channel_num = 0;
  bool init_access_flag = false;

  SPAT_CAN_WRITER SPaTCW;

  can_status = SPaTCW.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  ros::Subscriber sub1 = node.subscribe("/siheung_spat", 1, &SPAT_CAN_WRITER::CALLBACK_SPAT, &SPaTCW);

  ros::waitForShutdown();   
  canBusOff(hCAN);
  canClose(hCAN);
}
