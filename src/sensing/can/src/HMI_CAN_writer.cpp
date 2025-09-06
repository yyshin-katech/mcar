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
#include <mmc_msgs/lane_occupancy.h>
#include <mmc_msgs/localization2D_msg.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>
#include <mmc_msgs/from_control_team.h>
#include <sensor_msgs/NavSatFix.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

using namespace std;

bool vel_in=false;

class HMI_CAN_WRITER{
  public:
    ros::Subscriber sub1, sub2, sub3, sub4, sub5, sub6, sub7;
    ros::Publisher pub5;

    unsigned short freq_for_channel_3 = 100; // Hz

    float default_lane_width = 3.3f;
    
    ros::Time sensors_front_radar_time = ros::Time::now();
    ros::Time sensors_front_lidar_time = ros::Time::now();
    ros::Time sensors_front_vision_time = ros::Time::now();

    // unsigned int id_write, flag = 0;
    unsigned int kvaDb_flags = 0;
    unsigned int dlc = 8;

    unsigned int lane_change_info = 0;
    double speed_limit = 0;

    double station = 0;
    double station_len = 3693.307;
    double station_dest = 2051.65; // 파란 집

    float vel;
    bool vel_in = false;
    
    double dest_longitude_global = 128.401025; // 파란 집
    double dest_latitude_global = 35.6494966667; // 파란 집
    double dest_time = -1;

    int alive_count = 0;
    int alive_front_radar;
    int alive_front_lidar;
    int alive_front_vision;

    vector<int> lane_vacancy = {1, 1, 1};

    struct time_struct{double millisecond, second, minute, hour, day, month, year;};
    struct timeb international_time;
    struct tm *local_time;
    time_struct time_now;

    struct left         {double a, b, c, d; float view_range; int quality;};
    struct right        {double a, b, c, d; float view_range; int quality;};
    struct center_coeff {double a, b, c, d;};
    struct lane         {struct left  left;\
                         struct right right;\
                         struct center_coeff center;\
                         float width = 3.3f;};
    lane lane;
    
    vector<tuple<char*, vector<char*>>> msg_list;

    HMI_CAN_WRITER();

    void CALLBACK_VISION_LANE(const mmc_msgs::lane_array_msg& data);
    void CALLBACK_TRACK      (const mmc_msgs::object_array_msg& data);
    void CALLBACK_GPS      (const sensor_msgs::NavSatFix& data);
    void CALLBACK_HEADING(const mmc_msgs::localization2D_msg& data);
    void CALLBACK_VEL(const mmc_msgs::chassis_msg& data) {vel=data.speed; vel_in=true;}
    void CALLBACK_STATION(const mmc_msgs::to_control_team_from_local_msg& data);

    short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
    void GET_DATETIME();
    float GET_FUNC_VAL(const struct lane& lane, const short& x);
    canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
    void LOOP();
    void CALLBACK_LANE_OCCUPIED(const mmc_msgs::lane_occupancy& data);
    void CALLBACK_CONTROL_TEAM(const mmc_msgs::from_control_team& data);
};


HMI_CAN_WRITER::HMI_CAN_WRITER(){
  // List of CAN messages that you want to send to a CAN bus /////////////////////////////////////////////////
  msg_list.push_back(make_tuple((char*)"LeftLane",  vector<char*> {(char*)"Left_Lane_Curvature_Derivative",\
                                                                   (char*)"Left_Lane_Curvature",\
                                                                   (char*)"Left_Lane_Heading",\
                                                                   (char*)"Left_Lane_Position"}));

  msg_list.push_back(make_tuple((char*)"RightLane",  vector<char*> {(char*)"Right_Lane_Curvature_Derivative",\
                                                                    (char*)"Right_Lane_Curvature",\
                                                                    (char*)"Right_Lane_Heading",\
                                                                    (char*)"Right_Lane_Position"}));

  msg_list.push_back(make_tuple((char*)"Lane_Info",  vector<char*> {(char*)"Cam_lane_road_boundry",\
                                                                    (char*)"Cam_lane_driving",\
                                                                    (char*)"Cam_lane_confluence_branch",\
                                                                    (char*)"Cam_lane_change_info",\
                                                                    (char*)"Cam_right_lane_end_position",\
                                                                    (char*)"Cam_left_lane_end_position",\
                                                                    (char*)"Cam_right_lane_color",\
                                                                    (char*)"Cam_left_lane_color",\
                                                                    (char*)"Cam_right_lane_type",\
                                                                    (char*)"Cam_left_lane_type"}));

  msg_list.push_back(make_tuple((char*)"NextLeftLane",  vector<char*> {(char*)"Next_LLane_Curvature_Derivative",\
                                                                       (char*)"Next_Left_Lane_Curvature",\
                                                                       (char*)"Next_Left_Lane_Heading",\
                                                                       (char*)"Next_Left_Lane_Position"}));

  msg_list.push_back(make_tuple((char*)"NextRightLane",  vector<char*> {(char*)"Next_RLane_Curvature_Derivative",\
                                                                        (char*)"Next_Right_Lane_Curvature",\
                                                                        (char*)"Next_Right_Lane_Heading",\
                                                                        (char*)"Next_Right_Lane_Position"}));

  msg_list.push_back(make_tuple((char*)"NextLane_Info",  vector<char*> {(char*)"Next_lane_road_boundry",\
                                                                        (char*)"Next_lane_driving",\
                                                                        (char*)"Next_lane_confluence_branch",\
                                                                        (char*)"Next_lane_change_info",\
                                                                        (char*)"Next_right_lane_end_position",\
                                                                        (char*)"Next_left_lane_end_position",\
                                                                        (char*)"Next_right_lane_color",\
                                                                        (char*)"Next_left_lane_color",\
                                                                        (char*)"Next_right_lane_type",\
                                                                        (char*)"Next_left_lane_type"}));

  msg_list.push_back(make_tuple((char*)"PositionMessage1",     vector<char*> {(char*)"Longitude",\
                                                                              (char*)"Latitude"}));

  msg_list.push_back(make_tuple((char*)"PositionMessage2",     vector<char*> {(char*)"Heading"}));

  msg_list.push_back(make_tuple((char*)"DestinationMessage1",  vector<char*> {(char*)"Dest_Longitude",\
                                                                              (char*)"Dest_Latitude"}));

  msg_list.push_back(make_tuple((char*)"DestinationMessage2",  vector<char*> {(char*)"Destination_Time",\
                                                                              (char*)"Destination_Dist"}));

  msg_list.push_back(make_tuple((char*)"GUIMessage1",          vector<char*> {(char*)"Speed_Limit",\
                                                                              (char*)"Second",\
                                                                              (char*)"Minute",\
                                                                              (char*)"Hour",\
                                                                              (char*)"Day",\
                                                                              (char*)"Month",\
                                                                              (char*)"Year"}));

//////////////////////////////////////////////////////////////////////////////////////////////////////////

}


void HMI_CAN_WRITER::CALLBACK_VISION_LANE(const mmc_msgs::lane_array_msg& data){
  lane.left.a = data.data[0].a;
  lane.left.b = data.data[0].b;
  lane.left.c = data.data[0].c;
  lane.left.d = data.data[0].d;
  lane.left.view_range = data.data[0].View_Range;
  lane.left.quality = data.data[0].Quality;

  lane.right.a = data.data[1].a;
  lane.right.b = data.data[1].b;
  lane.right.c = data.data[1].c;
  lane.right.d = data.data[1].d;
  lane.right.view_range = data.data[1].View_Range;
  lane.right.quality = data.data[1].Quality;

  if(lane.left.quality > 1 && lane.right.quality > 1){
    lane.width = fabs(lane.left.d - lane.right.d);

  }else if(lane.left.quality <= 1 && lane.right.quality > 1){
    lane.width = default_lane_width;
    lane.left.d = lane.right.d + lane.width;

  }else if(lane.left.quality > 1 && lane.right.quality <= 1){
    lane.width = default_lane_width;
    lane.right.d = lane.left.d - lane.width;

  }else{
    lane.width = default_lane_width;
    lane.left.d = lane.width * 0.5;
    lane.right.d = -lane.width * 0.5;

  }

  lane.center.a = (lane.left.a + lane.right.a) * 0.5;
  lane.center.b = (lane.left.b + lane.right.b) * 0.5;
  lane.center.c = (lane.left.c + lane.right.c) * 0.5;
  lane.center.d = (lane.left.d + lane.right.d) * 0.5;

  // CAN write: LeftLane
  unsigned char can_data[dlc];
  unsigned short int msg_idx;
  char* target_msg;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  unsigned int id_write, flag = 0;

  target_msg = (char*)"LeftLane";  
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  vector<double> LEFT_LANE( get<1>(msg_list[msg_idx]).size() );
  LEFT_LANE = {lane.left.a, lane.left.b, lane.left.c, lane.left.d};

  for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), LEFT_LANE[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));

  // CAN write: RightLane
  target_msg = (char*)"RightLane";
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  vector<double> RIGHT_LANE( get<1>(msg_list[msg_idx]).size() );
  RIGHT_LANE = {lane.right.a, lane.right.b, lane.right.c, lane.right.d};

  for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), RIGHT_LANE[i]);
  }
  
  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));

  // CAN write: Lane_Info
  unsigned char can_data_lane_info[7];
  target_msg = (char*)"Lane_Info";
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  vector<double> LANE_INFO( get<1>(msg_list[msg_idx]).size() );

  int road_boundary = 0;
  int confluence_branch = 0;

  if(lane_vacancy[0]==0 && lane_vacancy[2]==0 || lane_vacancy[0]==1 && lane_vacancy[2]==1){
    road_boundary = 0;

  }else if(lane_vacancy[0]==0){
    road_boundary = 1;

  }else if(lane_vacancy[2]==0){
    road_boundary = 2;
  }
  
  if(station > 3600){
    confluence_branch = 1;

  }else if(900 < station && station < 1050){
    confluence_branch = 2;
  }

  int lane_type[2] = {1, 1}; // [left right]

  for(int i=0; i!=2; i++){

    if(data.data[i].marker_type == 0){
      lane_type[i] = 2;

    }else if(data.data[i].marker_type == 4){
      lane_type[i] = 3;
    }
  }
  
  LANE_INFO = {(double)road_boundary, 0, (double)confluence_branch, (double)lane_change_info,\
               lane.right.view_range, lane.left.view_range, 1, 1, (double)lane_type[1], (double)lane_type[0]};
                  
  for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data_lane_info, sizeof(can_data_lane_info), LANE_INFO[i]);
  }
  
  canWrite(hCAN, id_write, &can_data_lane_info, dlc, canMSG_STD);
  memset(can_data_lane_info, 0, sizeof(can_data_lane_info));

  // cout<<"vacancy = "<<endl;
  // cout<<"left  "<<lane_vacancy[0]<<endl;
  // cout<<"right  "<<lane_vacancy[2]<<endl;

  // CAN write: NextLeftLane / 한자연측의 요청으로 next lane 관련 메시지들은 내보내지 않는다.
  // target_msg = (char*)"NextLeftLane";  
  // msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  // kvaDbGetMsgByName(dh, target_msg, &mh);
  // kvaDbGetMsgId(mh, &id_write, &flag);

  // vector<double> NEXT_LEFT_LANE( get<1>(msg_list[msg_idx]).size() );
  // NEXT_LEFT_LANE = {lane.left.a, lane.left.b, lane.left.c, lane.left.d+lane.width};

  // for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
  //   kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
  //   kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), NEXT_LEFT_LANE[i]);
  // }

  // canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  // memset(can_data, 0, sizeof(can_data));
  

  // CAN write: NextRightLane / 한자연측의 요청으로 next lane 관련 메시지들은 내보내지 않는다.
  // target_msg = (char*)"NextRightLane";
  // msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  // kvaDbGetMsgByName(dh, target_msg, &mh);
  // kvaDbGetMsgId(mh, &id_write, &flag);

  // vector<double> NEXT_RIGHT_LANE( get<1>(msg_list[msg_idx]).size() );
  // NEXT_RIGHT_LANE = {lane.right.a, lane.right.b, lane.right.c, lane.right.d-lane.width};

  // for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
  //   kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
  //   kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), NEXT_RIGHT_LANE[i]);
  // }
  
  // canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  // memset(can_data, 0, sizeof(can_data));
  

  // CAN write: NextLane_Info / 한자연측의 요청으로 next lane 관련 메시지들은 내보내지 않는다.
  // target_msg = (char*)"NextLane_Info";
  // msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  // kvaDbGetMsgByName(dh, target_msg, &mh);
  // kvaDbGetMsgId(mh, &id_write, &flag);

  // int next_lane_type[2] = {0, 0};

  // for(int i=0; i!=2; i++){

  //   if(lane_vacancy[i] == 1){
  //     next_lane_type[i] = lane_type[i];
  //   }
  // }

  // vector<double> NEXT_LANE_INFO( get<1>(msg_list[msg_idx]).size() );

  // if(lane_vacancy[0]==1)
  // NEXT_LANE_INFO = {(double)road_boundary, 0, (double)confluence_branch, (double)lane_change_info,\
  //                   lane.right.view_range, lane.left.view_range, (double)1, (double)1, (double)next_lane_type[1]*lane_vacancy[2], (double)next_lane_type[0]*lane_vacancy[0]};

  // for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
  //   kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
  //   kvaDbStoreSignalValuePhys(sh, &can_data_lane_info, sizeof(can_data_lane_info), NEXT_LANE_INFO[i]);
  // }
  
  // canWrite(hCAN, id_write, &can_data_lane_info, dlc, canMSG_STD);
  // memset(can_data_lane_info, 0, sizeof(can_data_lane_info));
  

}


void HMI_CAN_WRITER::CALLBACK_GPS(const sensor_msgs::NavSatFix& data)
{

  unsigned char can_data[dlc];
  char* target_msg;
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  unsigned int id_write, flag = 0;
 
  target_msg = (char*)"PositionMessage1";
  temp_data = {(double)data.longitude, (double)data.latitude};

  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));
}


void HMI_CAN_WRITER::CALLBACK_HEADING(const mmc_msgs::localization2D_msg &data)
{

  unsigned char can_data[4];
  char *target_msg;
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  unsigned int id_write, flag = 0;

  target_msg = (char *)"PositionMessage2";
  temp_data = {(double)data.yaw*180/3.1415};

  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
  {
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));
}


void HMI_CAN_WRITER::CALLBACK_STATION(const mmc_msgs::to_control_team_from_local_msg& data)
{
  double dest_dist = (station_dest - data.station);
  station = data.station;
  //  time output = -1 when vel = 0, chassis input x 
  
  if (dest_dist < 0)
  {
    dest_dist += station_len;
  }

  if (vel_in && 1<vel)
  {
    dest_time = dest_dist / vel;
  }

  unsigned char can_data[dlc];
  char* target_msg;
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  unsigned int id_write, flag = 0;

  double dest_longitude = dest_longitude_global;
  double dest_latitude = dest_latitude_global;

  // 데모시작지점과 파란집 사이에서만 destination 정보가 나가야 한다. 아닌 경우 다 0으로 보낸다.
  if(2051.65<data.station && data.station<2500 || \
     300<data.station && data.station<500 && data.lane_id==4){ 
    dest_time = 0;
    dest_dist = 0;
    dest_longitude = 0;
    dest_latitude = 0;
  }

  cout<<"#############################"<<endl;
  cout<<"dest_time  = "<<dest_time<<endl;
  cout<<"dest_dist  = "<<dest_dist<<endl;
  cout<<"dest_longi = "<<dest_longitude<<endl;
  cout<<"dest_lati  = "<<dest_latitude<<endl;

  for (short i = 0; i < 2; i++)
  {

    switch (i)
    {

    case (0):
      target_msg = (char *)"DestinationMessage2";
      temp_data = {dest_time, dest_dist};
    break;

    case (1):
      target_msg = (char *)"DestinationMessage1";
      temp_data = {dest_longitude, dest_latitude};
    break;
    }

    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
    {
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[i]);
    }

    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));
  }
}


short HMI_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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

void HMI_CAN_WRITER::GET_DATETIME(){
  ftime(&international_time);
  local_time = localtime(&international_time.time);

  time_now.millisecond = international_time.millitm;
  time_now.second = local_time->tm_sec;
  time_now.minute = local_time->tm_min;
  time_now.hour = local_time->tm_hour;
  time_now.day = local_time->tm_mday;
  time_now.month = local_time->tm_mon + 1;
  time_now.year = local_time->tm_year - 100; // default: +1900
} 

float HMI_CAN_WRITER::GET_FUNC_VAL(const struct lane& lane, const short& x){
  // the y-axis of Mobileye coordinates is the opposite of the y-axis of ISO coordinates
  float func_val = -(lane.center.a * pow(x, 3) + lane.center.b * pow(x, 2) + lane.center.c * x + lane.center.d);
  return func_val;
}


// canStatus HMI_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
//   canInitializeLibrary();

//   int open_flag;

//   if(init_access_flag == true){
//       open_flag = canOPEN_REQUIRE_INIT_ACCESS;
//       // open_flag = 0;

//     }else{
//       open_flag = canOPEN_NO_INIT_ACCESS;
//     }

//   hCAN = canOpenChannel(channel_num, open_flag);

//   // while(hCAN!=canOK){
//   //   cout<<"Opening the channel "<<channel_num<<"..."<<endl;
//   //   hCAN = canOpenChannel(channel_num, open_flag);
//   // }

//   cout<<"hCAN: "<<hCAN<<endl;
  
//   if(hCAN==canOK){
//     cout<<"The CAN channel "<<channel_num<<" has been opened successfully..."<<endl;
//   }
  
//   can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
//   can_status = canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
//   can_status = canBusOn(hCAN);

//   if(can_status == canOK){
//     cout<<"The CAN bus is on..."<<endl;
//   }

//   kvaDb_status = kvaDbOpen(&dh);
//   kvaDb_status = kvaDbReadFile(dh, filename);
//   kvaDb_status = kvaDbGetFlags(dh, &kvaDb_flags);

//   if(kvaDb_status == kvaDbOK){
//     cout<<"Database file has been loaded successfully..."<<endl;
//   }

//   return canOK;
// }


void HMI_CAN_WRITER::LOOP(){
  ros::Rate rate(freq_for_channel_3);
  
  while(ros::ok()){

    unsigned char can_data[7];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    unsigned int id_write, flag = 0;

    // CAN write: time information
    target_msg= (char*)"GUIMessage1";
    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    vector<double> GUIMessage1( get<1>(msg_list[msg_idx]).size() );
    GET_DATETIME();

    GUIMessage1 = {speed_limit,\
                   time_now.second,\
                   time_now.minute,\
                   time_now.hour,\
                   time_now.day,\
                   time_now.month,\
                   time_now.year};

    for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), GUIMessage1[i]);
    }
    
    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));


    rate.sleep();
  }
}


void HMI_CAN_WRITER::CALLBACK_LANE_OCCUPIED(const mmc_msgs::lane_occupancy& data){
  lane_vacancy.clear();

  if(data.left == 0){
    lane_vacancy.push_back(1);

  }else{
    lane_vacancy.push_back(0);
  }

  if(data.center == 0){
    lane_vacancy.push_back(1);

  }else{
    lane_vacancy.push_back(0);
  }

  if(data.right == 0){
    lane_vacancy.push_back(1);

  }else{
    lane_vacancy.push_back(0);
  }
}


void HMI_CAN_WRITER::CALLBACK_CONTROL_TEAM(const mmc_msgs::from_control_team& data){
  lane_change_info = data.lane_change_info;
  speed_limit = data.speed_limit;
}


int main(int argc, char **argv){
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "HMI_CAN_WRITER");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];
  
  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/GUI_CAN_V2.3.dbc").c_str());
  int channel_num = 3;
  // int channel_num = 0;
  bool init_access_flag = false; // 테크노에서 이거 반드시 확인해라!!!!!!!!!!! 
  ///////////////////////////////////////////////////////////////////////////////////////

  HMI_CAN_WRITER HCW;
  
  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  ros::Subscriber sub1 = node.subscribe("/sensors/vision_lane",          1, &HMI_CAN_WRITER::CALLBACK_VISION_LANE,   &HCW);
  ros::Subscriber sub2 = node.subscribe("/point_cloud/lane_occupied",    1, &HMI_CAN_WRITER::CALLBACK_LANE_OCCUPIED, &HCW);
  ros::Subscriber sub3 = node.subscribe("/sensors/gps/fix",              1, &HMI_CAN_WRITER::CALLBACK_GPS, &HCW);
  ros::Subscriber sub4 = node.subscribe("/localization/pose_2d",         1, &HMI_CAN_WRITER::CALLBACK_HEADING, &HCW);
  ros::Subscriber sub5 = node.subscribe("/sensors/chassis",              1, &HMI_CAN_WRITER::CALLBACK_VEL, &HCW);
  ros::Subscriber sub6 = node.subscribe("/localization/to_control_team", 1, &HMI_CAN_WRITER::CALLBACK_STATION, &HCW);
  ros::Subscriber sub7 = node.subscribe("/from_control_team",            1, &HMI_CAN_WRITER::CALLBACK_CONTROL_TEAM, &HCW);
  
  // overlay text /////////////////////////////////////////////////////////////////
  HCW.pub5 = node.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/HMI_CAN_status", 10, true);
  jsk_rviz_plugins::OverlayText msg;
  string can_status_str;

  if(can_status == canOK){
    can_status_str = "OK";

  }else{
    can_status_str = "ERR";
  }

  msg.text.append("- CAN HMI ...... " + can_status_str + "\n");

  std_msgs::ColorRGBA state_color;
  
  int32_t width = 250;
  int32_t height = width*2;

  msg.action = msg.ADD;
  msg.font = "DejaVu Sans Mono";
  msg.text_size = 12;
  msg.width = width;
  msg.height = height;
  msg.left = 10;
  msg.top = 520;

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
  HCW.pub5.publish(msg);

  HCW.LOOP();

  ros::waitForShutdown();   
  canBusOff(hCAN);
  canClose(hCAN);
}