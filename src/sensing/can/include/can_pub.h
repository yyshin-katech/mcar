#ifndef CAN_PUB_H
#define CAN_PUB_H

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
#include <mmc_msgs/from_control_team.h>

#include <katech_diagnostic_msgs/k_adcu_diagnostic_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <visualization_msgs/MarkerArray.h>

#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_rviz_plugins/OverlayMenu.h>




#include <kvaDbLib.h>
#include <canlib.h>

using namespace std;

#define PI 3.141592

////////////////// Global variables ///////////////////
ros::Publisher pub0, pub1, pub2, pub3, pub4, pub5, radar_pub;
ros::Subscriber sub1, sub2, sub3, sub4, sub5;

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
unsigned short freq_for_channel_1 = 4000; // Hz
unsigned short freq_for_channel_2 = 4000; // Hz
unsigned short freq_for_channel_3 = 3000; // Hz
unsigned short timeout_channel_0 = 100; // ms
unsigned short timeout_channel_1 = 100; // ms
unsigned short timeout_channel_2 = 100; // ms
unsigned short timeout_channel_3 = 100; // ms

unsigned short id;
char buff[50];
double value;
///////////////////////////////////////////////////////

canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
void FRONT_RADAR_CAN_READER();
void FRONT_LIDAR_CAN_READER();
void VISION_CAN_READER();
void CHASSIS_CAN_READER();
void PLANNED_PATH_CAN_READER();
void FROM_CONTROL_TEAM_CAN_READER();

#endif