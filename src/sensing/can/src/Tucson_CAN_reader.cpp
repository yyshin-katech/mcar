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
double east = 0;
double north = 0;
double yaw = 0;
double altitude = 0;
double position_covariance_xx = 0;
double position_covariance_yy = 0;




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

void TUCSON_CAN_READER()
{
  ros::Rate rate(freq_for_channel_0);
  ros::Time time_last_pub = ros::Time::now();
  ros::Duration time_force_pub = ros::Duration(0.01);
  mmc_msgs::V2V msg;

  short MsgCount = 0;

  vector<tuple<char *, vector<char *>>> msg_list;

  msg_list.push_back(make_tuple((char *)"TCU1", vector<char *>{(char *)"gear_selector_display"}));

  msg_list.push_back(make_tuple((char *)"TCS1", vector<char *>{(char *)"ABS_controlIndication"}));

  msg_list.push_back(make_tuple((char *)"EMS2", vector<char *>{(char *)"Indication_ofBrakeSwON_OFF"}));

  msg_list.push_back(make_tuple((char *)"SAS1", vector<char *>{(char *)"Steering_wheel_angle"}));

  msg_list.push_back(make_tuple((char *)"TCU2", vector<char *>{(char *)"Vehicle_SpeedCalculatedByTCU"}));

  msg_list.push_back(make_tuple((char *)"ESP2", vector<char *>{(char *)"LONG_ACCEL",
                                                               (char *)"LAT_ACCEL",
                                                               (char *)"YAW_RATE"}));

  while (ros::ok())
  { // 4000Hz

    if (ros::Time::now() - time_last_pub > time_force_pub)
    {
      if (msg.TransmissionState == 1 && msg.Speed == 0) // 고장차량 정의 : 기어P단 && 속도0
      {
        msg.Vehicle_Event_Flag_eventDisabledVehicle = 1;
      }
      else
      {
        msg.Vehicle_Event_Flag_eventDisabledVehicle = 0;
      }
      
      // V2V.msg 파일 참고해라 
      // V2V 토픽 안에 있는 변수 이름이랑 실제 값이랑 다른 의미를 가지고 있다. V2V.msg 파일을 반드시 확인할 것.
      msg.GPS_Position_Latitude = east;
      msg.GPS_Position_Longitude = north;
      //////////////////////////////////////////////////////////////////////////////////////////
      msg.Heading = yaw;
      msg.GPS_Position_Elevation = altitude;
      msg.Position_Accuracy_SemiMajorAxisAccuracy = position_covariance_xx;
      msg.Position_Accuracy_SemiMinorAxisAccuracy = position_covariance_yy;
      
      msg.MsgCount = MsgCount;
      pub1.publish(msg);
      time_last_pub = ros::Time::now();
      MsgCount += 1;

      if (MsgCount > 127)
      {
        MsgCount = 0;
      }
    }

    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_0);
    kvaDb_status = kvaDbGetMsgById(dh, temp_id, &mh);

    if (kvaDb_status == kvaDbOK)
    {
      kvaDbGetMsgName(mh, buff, sizeof(buff));
      short msg_idx = 9999;

      for (short i = 0; i != msg_list.size(); i++)
      {

        if (strcmp(buff, get<0>(msg_list[i])) == 0)
        {
          msg_idx = i;
        }
      }

      switch (msg_idx)
      {
      case (0): // TCU1

        for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
        {
          kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

          switch ((int)value)
          {
          case (6): // Neutral
            msg.TransmissionState = 0;
            break;
          case (0): // Park
            msg.TransmissionState = 1;
            break;
          case (5): // Drive(forward)
            msg.TransmissionState = 2;
            break;
          case (7): // Reverse
            msg.TransmissionState = 3;
            break;
          default:
            msg.TransmissionState = 7;
            break;
          }
        }
        break;

      case (1): // TCS1

        for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
        {
          kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

          if (value == 1)
          {
            msg.BrakeSystemStatus_Anti_Lock_Brake_Status = "Engaged";
            msg.Vehicle_Event_Flag_eventABSactivated = 1;
          }
          else
          {
            msg.BrakeSystemStatus_Anti_Lock_Brake_Status = "On"; // Available but inactive
            msg.Vehicle_Event_Flag_eventABSactivated = 0;
          }
        }
        break;

      case (2): // EMS2

        for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
        {
          kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

          if (value == 1 || value == 3)
          {
            msg.BrakeSystemStatus_Brake_Applied_Status = 0;
          }
          else if (value == 2)
          {
            msg.BrakeSystemStatus_Brake_Applied_Status = 1;
          }
        }
        break;

      case (3): // SAS1

        for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
        {
          kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

          msg.SteeringWheelAngle = value;
        }
        break;

      case (4): // EMS1

        for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
        {
          kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

          msg.Speed = value / 3.6; // kph to m/s
        }
        break;

      case (5): // ESP2
        for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
        {
          kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
          kvaDbRetrieveSignalValuePhys(sh, &value, &can_data, sizeof(can_data));

          switch (i)
          {
          case (0):
            msg.Acceleration_Set_4Way_Longitude_Acceleration = value;
            break;
          case (1):
            msg.Acceleration_Set_4Way_Latitude_Acceleration = value;
            break;
          case (2):
            msg.Acceleration_Set_4Way_YawRate = value;
            break;
          }
        }
        break;

      default:
        break;
      }
    }
    rate.sleep();
  }
}


void gps_fix_callback(const sensor_msgs::NavSatFix& data){
  altitude = data.altitude;
  position_covariance_xx = data.position_covariance[0];
  position_covariance_yy = data.position_covariance[5];
}


void local_callback(const mmc_msgs::localization2D_msg& data){
  east = data.east;
  north = data.north;
  yaw = data.yaw;
}


int main(int argc, char **argv)
{

  cout << "Initializing ..." << endl;
  ros::init(argc, argv, "Tucson_CAN_reader");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/HMC_1p7_2_TUCSON.dbc").c_str());
  int channel_num = 0;
  bool init_access_flag = true; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  ros::Subscriber sub1 = node.subscribe("/sensors/gps/fix",         1, gps_fix_callback);
  ros::Subscriber sub2 = node.subscribe("/localization/V2V_Tucson", 1, local_callback);

  pub1 = node.advertise<mmc_msgs::V2V>("/V2V", 1);
  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  if (can_status == canOK)
  {
    TUCSON_CAN_READER();
  }

  

  ros::waitForShutdown();
  canBusOff(hCAN);
  canClose(hCAN);
}
