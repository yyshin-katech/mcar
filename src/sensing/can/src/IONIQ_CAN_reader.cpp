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

#include <mmc_msgs/V2V.h>

#include <mmc_msgs/motor_rpm_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <kvaDbLib.h>
#include <canlib.h>

using namespace std;

#define PI 3.141592

ros::Timer timer_;
ros::Publisher pub1;

canHandle hCAN;
canStatus can_status;

double tire_radius = 0.326;
double gear_ratio = 7.412;

long temp_id;
unsigned long timestamp;
unsigned int id_write, flag, dlc = 64, canread_flag = 0;
unsigned char can_data[64]; // CAN FD

KvaDbStatus kvaDb_status;
KvaDbHnd dh = 0;
KvaDbMessageHnd mh = 0;
KvaDbSignalHnd sh = 0;
unsigned int kvaDb_flags = 0;

unsigned short freq_for_channel_0 = 4000; // Hz
unsigned short timeout_channel_0 = 100;   // ms

unsigned short id;
char buff[50];
double value;

typedef struct{
  double wheel_spd_fl;
  double wheel_spd_fr;
  double wheel_spd_rl;
  double wheel_spd_rr;
} structWHL_SPD;

structWHL_SPD sWheel_SPD;
unsigned char gear_pos;

mmc_msgs::motor_rpm_msg msgRPM;

void timerCallback(const ros::TimerEvent&)
{
  double wheel_rpm = 0;
  double motor_rpm = 0;

  // wheel_speed is in m/s (V_CAN_Release.dbc, scale 0.01)
  wheel_rpm = (sWheel_SPD.wheel_spd_fl * 60.0) / (2.0 * PI * tire_radius);
  motor_rpm = wheel_rpm * gear_ratio;

  msgRPM.N = motor_rpm;
  msgRPM.gear_pos = gear_pos;

  pub1.publish(msgRPM);
}

short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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

canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag)
{
  canInitializeLibrary();
  int open_flag;

  if (init_access_flag == true)
  {
    open_flag = canOPEN_REQUIRE_INIT_ACCESS | canOPEN_CAN_FD;
  }
  else
  {
    open_flag = canOPEN_NO_INIT_ACCESS | canOPEN_CAN_FD;
  }

  hCAN = canOpenChannel(channel_num, open_flag);

  cout<<hCAN<<endl;

  if (hCAN == canOK)
  {
    cout << "The CAN channel " << channel_num << " has been opened successfully..." << endl;
  }

  can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
  can_status = canSetBusParamsFd(hCAN, canFD_BITRATE_1M_80P, 0, 0, 0);
  can_status = canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
  can_status = canBusOn(hCAN);

  if (can_status == canOK)
  {
    cout << "The CAN bus is on..." << endl;
  }

  kvaDb_status = kvaDbOpen(&dh);
  kvaDb_status = kvaDbReadFile(dh, filename);
  kvaDb_status = kvaDbGetFlags(dh, &kvaDb_flags);

  if (kvaDb_status == kvaDbOK)
  {
    cout << "The database file has been loaded successfully..." << endl;
  }

  return canOK;
}

void IONIQ_CAN_READER()
{
  ros::Rate rate(freq_for_channel_0);
  ros::Time time_last_pub = ros::Time::now();

  short MsgCount = 0;

  vector<tuple<char *, vector<char *>>> msg_list;

  msg_list.push_back(make_tuple((char *)"WheelInfo", vector<char *>{(char *)"wheel_speed_fl",
                                                                    (char *)"wheel_speed_fr",
                                                                    (char *)"wheel_speed_rl",
                                                                    (char *)"wheel_speed_rr"}));

  msg_list.push_back(make_tuple((char *)"GearInfo", vector<char *>{(char *)"gear_status"}));

  while (ros::ok())
  { // 4000Hz
    dlc = 64;
    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_0);
    kvaDb_status = kvaDbGetMsgById(dh, temp_id, &mh);

    if (kvaDb_status == kvaDbOK)
    {
      kvaDbGetMsgName(mh, buff, sizeof(buff));
      short msg_idx = 9999;

      // 매칭되는 메시지 찾기
      bool matched_flag = false;

      for(int i=0; i < msg_list.size(); i++){
        if(strcmp(buff, get<0>(msg_list[i])) == 0){
          matched_flag = true;
          msg_idx = i;
          break; // 찾았으면 루프 종료
        }
      }

      if (msg_idx == 0) // WheelInfo (CAN FD, 16 bytes)
      {
        for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
        {
          kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, can_data, dlc);

          switch (i)
          {
          case (0): // wheel_speed_fl
            sWheel_SPD.wheel_spd_fl = (double)value;
            break;
          case (1): // wheel_speed_fr
            sWheel_SPD.wheel_spd_fr = (double)value;
            break;
          case (2): // wheel_speed_rl
            sWheel_SPD.wheel_spd_rl = (double)value;
            break;
          case (3): // wheel_speed_rr
            sWheel_SPD.wheel_spd_rr = (double)value;
            break;
          default:
            break;
          }
        }

      }
      else if (msg_idx == 1) // GearInfo
      {
        for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
        {
          kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, can_data, dlc);

          switch (i)
          {
          case (0): // gear_status (P=1, R=2, N=3, D=4)
            gear_pos = (unsigned char)value;
            break;
          default:
            break;
          }
        }
      }

    }
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  cout << "Initializing ..." << endl;
  ros::init(argc, argv, "IONIQ_CAN_reader");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  timer_ = node.createTimer(ros::Duration(0.1), &timerCallback);
  pub1 = node.advertise<mmc_msgs::motor_rpm_msg>("/sensors/rpm", 1);

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/V_CAN_Release.dbc").c_str());
  int channel_num = 2;
  bool init_access_flag = true; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  if (can_status == canOK)
  {
    IONIQ_CAN_READER();
  }

  ros::waitForShutdown();
  canBusOff(hCAN);
  canClose(hCAN);
}
