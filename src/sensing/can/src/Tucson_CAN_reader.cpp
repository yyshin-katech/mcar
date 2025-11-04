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

#include <sensor_msgs/NavSatFix.h>
#include <mmc_msgs/localization2D_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <kvaDbLib.h>
#include <canlib.h>

using namespace std;

#define PI 3.141592

ros::Publisher pub1;

canHandle hCAN;
canStatus can_status;

double tire_radius = 0.326;
double gear_ratio = 7.412;

long temp_id;
unsigned long timestamp;
unsigned int id_write, flag, dlc = 8, canread_flag = 0;
unsigned char can_data[8]; // CAN standard

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
  unsigned char AliveCnt_LSB;
  unsigned char AliveCnt_MSB;
  unsigned char Checksum_LSB;
  unsigned char Checksum_MSB;
} structWHL_SPD11;

structWHL_SPD11 sWheel_SPD;

canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag)
{
  canInitializeLibrary();
  int open_flag;

  if (init_access_flag == true)
  {
    open_flag = canOPEN_REQUIRE_INIT_ACCESS;
  }
  else
  {
    open_flag = canOPEN_NO_INIT_ACCESS;
  }

  hCAN = canOpenChannel(channel_num, open_flag);

  cout<<hCAN<<endl;

  if (hCAN == canOK)
  {
    cout << "The CAN channel " << channel_num << " has been opened successfully..." << endl;
  }

  can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
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
  ros::Duration time_force_pub = ros::Duration(0.01);
  mmc_msgs::V2V msg;

  short MsgCount = 0;
  double wheel_rpm = 0;
  double motor_rpm = 0;
  
  vector<tuple<char *, vector<char *>>> msg_list;

  msg_list.push_back(make_tuple((char *)"WHL_SPD11", vector<char *>{(char *)"WHL_SPD_FL",
                                                                    (char *)"WHL_SPD_FR",
                                                                    (char *)"WHL_SPD_RL",
                                                                    (char *)"WHL_SPD_RR",
                                                                    (char *)"WHL_SPD_AliveCounter_LSB",
                                                                    (char *)"WHL_SPD_AliveCounter_MSB",
                                                                    (char *)"WHL_SPD_Checksum_LSB"
                                                                    (char *)"WHL_SPD_Checksum_MSB",}));

  while (ros::ok())
  { // 4000Hz
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

      for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
      {
        kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
        kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

        switch (i)
        {
        case (0): // Neutral
          sWheel_SPD.wheel_spd_fl = (double)value;
          break;
        case (1): // Park
          sWheel_SPD.wheel_spd_fr = (double)value;
          break;
        case (2): // Drive(forward)
          sWheel_SPD.wheel_spd_rl = (double)value;
          break;
        case (3): // Reverse
          sWheel_SPD.wheel_spd_rl = (double)value;
          break;
        case (4): // Reverse
          sWheel_SPD.AliveCnt_LSB = (unsigned char)value;
          break;
        case (5): // Reverse
          sWheel_SPD.AliveCnt_MSB = (unsigned char)value;
          break;
        case (6): // Reverse
          sWheel_SPD.Checksum_LSB = (unsigned char)value;
          break;
        case (7): // Reverse
          sWheel_SPD.Checksum_MSB = (unsigned char)value;
          break;
        default:
          break;
        }
      }  
    }

    wheel_rpm = (sWheel_SPD.wheel_spd_fl * 1000) / (60 * 2 * PI * tire_radius);
    motor_rpm = wheel_rpm * gear_ratio;
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

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/2gen-2ch-C_IoniqEV.dbc").c_str());
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
