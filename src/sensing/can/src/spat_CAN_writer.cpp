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
#include <mmc_msgs/to_control_team_from_local_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

using namespace std;

#define MAX_SPAT_MSG 5

class SPAT_CAN_WRITER{
  public:
    ros::Subscriber sub1, sub2;

    unsigned int kvaDb_flags = 0;
    unsigned int dlc = 4;

    double time1 = -1;
    float new_time = 0;
    int alive_count = 0;    

    bool spat_calling = false;
    
    unsigned short g_intersection_id = 0;
    unsigned char g_signalGroup_id = 0;

    vector<tuple<char*, vector<char*>>> msg_list;

    SPAT_CAN_WRITER();
    void CALLBACK_LOCAL_TEAM(const mmc_msgs::to_control_team_from_local_msg& msg);
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

void SPAT_CAN_WRITER::CALLBACK_LOCAL_TEAM(const mmc_msgs::to_control_team_from_local_msg& msg){

  g_intersection_id = msg.look_at_IntersectionID;
  g_signalGroup_id = msg.look_at_signalGroupID;
}

void SPAT_CAN_WRITER::CALLBACK_SPAT(const v2x_msgs::intersection_array_msg& msg ){
  spat_calling = true;

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
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  unsigned int id_write, flag = 0;
  int re_value = 0;
  
  // ROS_INFO("SPaT msg CALLBACK@!!!!!!!!");
  int temp_time =0;
  unsigned char temp_phase = 0;
  for(int i = 0; i<MAX_SPAT_MSG; i++)
  {
    // g_intersection_id = 500;
    // g_signalGroup_id = 2;
    if(g_intersection_id != 0)
    {
      if(msg.data[i].IntersectionID == g_intersection_id)
      {
        if(msg.data[i].Movements.SignalGroupID == g_signalGroup_id)
        {
          target_msg = (char*)"V2X_SPaT_1";

          temp_time = msg.data[i].Movements.TimeChangeDetails;
          temp_phase = msg.data[i].Movements.MovementPhaseStatus;
          // 어린이 보호구역 진입 신호등
          if ((g_intersection_id == 200) && (g_signalGroup_id == 9))
          {
            if (msg.data[i].Movements.MovementPhaseStatus == 3) //red
            {
              if (msg.data[i].Movements.TimeChangeDetails > (65-12))
              {
                temp_time = 65 - msg.data[i].Movements.TimeChangeDetails;
                temp_phase = 3;
              }
              else
              {
                temp_time = 20;
                temp_phase = 6;
              }
            }
          } 
          else
          {
            temp_time = msg.data[i].Movements.TimeChangeDetails;
            temp_phase = msg.data[i].Movements.MovementPhaseStatus;
          }

          temp_data = {(char)0,
          (int)temp_time,
          (unsigned char)temp_phase,
          (double)msg.data[i].Movements.SignalGroupID,
          (double)msg.data[i].IntersectionID};

          msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
          kvaDbGetMsgByName(dh, target_msg, &mh);
          kvaDbGetMsgId(mh, &id_write, &flag);

          for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[j]);
          }
          re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
          memset(can_data, 0, sizeof(can_data));

          ROS_INFO("Intersection ID : %d", msg.data[i].IntersectionID);
          ROS_INFO("signalGroup : %d", msg.data[i].Movements.SignalGroupID);
          ROS_INFO("eventState : %d", msg.data[i].Movements.MovementPhaseStatus);
          ROS_INFO("minEndTime : %d", msg.data[i].Movements.TimeChangeDetails);
        }
      }
    }
    else
    {
      target_msg = (char*)"V2X_SPaT_1";
      temp_data = {(char)0,
      (int)0,
      (unsigned char)0,
      (double)0,
      (double)0};

      msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
      kvaDbGetMsgByName(dh, target_msg, &mh);
      kvaDbGetMsgId(mh, &id_write, &flag);

      for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
        kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
        kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[j]);
      }
      re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
      memset(can_data, 0, sizeof(can_data));
    }
  }
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

  ros::Subscriber sub1 = node.subscribe("/katri_v2x_node/katri_spat", 1, &SPAT_CAN_WRITER::CALLBACK_SPAT, &SPaTCW);
  // ros::Subscriber sub1 = node.subscribe("/ktri_obu_interface_node/katri_spat", 1, &SPAT_CAN_WRITER::CALLBACK_SPAT, &SPaTCW);
  ros::Subscriber sub2 = node.subscribe("/localization/to_control_team", 1, &SPAT_CAN_WRITER::CALLBACK_LOCAL_TEAM, &SPaTCW);

  ros::waitForShutdown();   
  canBusOff(hCAN);
  canClose(hCAN);
}
