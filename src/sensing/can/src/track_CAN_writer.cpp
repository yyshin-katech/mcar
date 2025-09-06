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

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <kvaDbLib.h>
#include <canlib.h>

#include <can_pub.h>

using namespace std;

// canHandle hCAN;
// canStatus can_status;
// KvaDbStatus kvaDb_status;
// KvaDbHnd dh = 0;

class TRACK_CAN_WRITER{
  public:
    // ros::Subscriber sub1, sub2, sub3, sub4, sub5, sub6, sub7;
    ros::Publisher pub1, pub2, pub3, pub4, pub5;
    
    unsigned short freq_for_channel_2 = 100; // Hz
    short look_ahead_dist = 150;
    short look_behind_dist = -150;

    float default_lane_width = 3.6f;

    float speed_ego = 0.f;
    
    ros::Time sensors_front_radar_time = ros::Time::now();
    ros::Time sensors_front_lidar_time = ros::Time::now();
    ros::Time sensors_front_vision_time = ros::Time::now();
    ros::Time sensors_front_V2V_time = ros::Time::now();
    ros::Time track_time = ros::Time::now();
    ros::Time time1 = ros::Time::now();
    ros::Time time2 = ros::Time::now();
    ros::Duration alive_timeout = ros::Duration(1.0);
    ros::Duration dt;

    ros::Time time_fault_radar = ros::Time::now();
    ros::Time time_alive_radar = ros::Time::now();
    ros::Time time_fault_lidar = ros::Time::now();
    ros::Time time_alive_lidar = ros::Time::now();
    ros::Time time_fault_vision = ros::Time::now();
    ros::Time time_alive_vision = ros::Time::now();
    ros::Time time_fault_V2V = ros::Time::now();
    ros::Time time_alive_V2V = ros::Time::now();

    // unsigned int id_write;
    // unsigned int flag = 0;
    unsigned int kvaDb_flags = 0;
    unsigned int dlc = 8;

    bool track_calling = false;
    bool param_radar_fault_input = false;
    bool param_lidar_fault_input = false;
    bool param_vision_fault_input = false;
    bool param_V2V_fault_input = false;

    int radar_fault_detect = 1;
    int lidar_fault_detect = 1;
    int vision_fault_detect = 1;
    int V2V_fault_detect = 1;

    int alive_count = 0;
    int radar_fault_input;
    int lidar_fault_input;
    int vision_fault_input;
    int V2V_fault_input;
        
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
                         float width = 3.6f;};
    lane lane;
    
    vector<short> x_range_front, x_range_rear;
    vector<float> lane_points_front, lane_points_rear;
    vector<tuple<char*, vector<char*>>> msg_list;
    
    mmc_msgs::object_array_msg *track_stored = new mmc_msgs::object_array_msg;
    mmc_msgs::object_array_msg track;

    //**************************** new ************************************//
    mmc_msgs::object_array_msg *track_box = new mmc_msgs::object_array_msg;
    bool track_fault = false;

    //**************************** new ************************************//


    TRACK_CAN_WRITER();



    //**************************** new ************************************//
    void CALLBACK_BOX(const mmc_msgs::object_array_msg& data);
    int Nearest_idx(const float x, float y);
    //**************************** new ************************************//
    void CALLBACK_FRONT_RADAR(const mmc_msgs::object_array_msg& data);
    void CALLBACK_FRONT_LIDAR(const mmc_msgs::object_array_msg& data);
    void CALLBACK_VISION_POS (const mmc_msgs::object_array_msg& data);
    void CALLBACK_VISION_LANE(const mmc_msgs::lane_array_msg& data);
    void CALLBACK_V2V(const mmc_msgs::object_array_msg& data);
    void CALLBACK_CHASSIS(const mmc_msgs::chassis_msg& data);
    void CALLBACK_TRACK      (const mmc_msgs::object_array_msg& data);
    void TRACK_FRONT_PROCESS       (mmc_msgs::object_array_msg& track, const vector<short>& idx);
    void TRACK_CORNER_LEFT_PROCESS (mmc_msgs::object_array_msg& track, const vector<short>& idx);
    void TRACK_CORNER_RIGHT_PROCESS(mmc_msgs::object_array_msg& track, const vector<short>& idx);
    void TRACK_REAR_PROCESS        (mmc_msgs::object_array_msg& track, const vector<short>& idx);
    short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
    void GET_DATETIME();
    float GET_FUNC_VAL(const struct lane& lane, const short& x);
    // canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
    void LOOP();
};


TRACK_CAN_WRITER::TRACK_CAN_WRITER(){
  // List of CAN messages that you want to send to a CAN bus /////////////////////////////////////////////////
  msg_list.push_back(make_tuple((char*)"TimeInformation",  vector<char*> {(char*)"MilliSecond",\
                                                                          (char*)"Second",\
                                                                          (char*)"Minute",\
                                                                          (char*)"Hour",\
                                                                          (char*)"Day",\
                                                                          (char*)"Month",\
                                                                          (char*)"Year"}));

  msg_list.push_back(make_tuple((char*)"FAULT_INPUT_MESSAGE",  vector<char*> {(char*)"FAULT_INPUT_V2V",\
                                                                              (char*)"FAULT_INPUT_FRONT_VISION",\
                                                                              (char*)"FAULT_INPUT_FRONT_RADAR",\
                                                                              (char*)"FAULT_INPUT_FRONT_LIDAR"}));

  msg_list.push_back(make_tuple((char*)"FAULT_DETECT_MESSAGE",  vector<char*> {(char*)"FAULT_DETECT_V2V",\
                                                                               (char*)"FAULT_DETECT_FRONT_VISION",\
                                                                               (char*)"FAULT_DETECT_FRONT_RADAR",\
                                                                               (char*)"FAULT_DETECT_FRONT_LIDAR"}));

  msg_list.push_back(make_tuple((char*)"RIGHT_LANE_B", vector<char*> {(char*)"LANE_VIEW_RANGE",\
                                                                      (char*)"LANE_QUALITY",\
                                                                      (char*)"LANE_D",\
                                                                      (char*)"LANE_C"}));

  msg_list.push_back(make_tuple((char*)"RIGHT_LANE_A", vector<char*> {(char*)"LANE_B",\
                                                                      (char*)"LANE_A"}));

  msg_list.push_back(make_tuple((char*)"LEFT_LANE_B", vector<char*> {(char*)"LANE_VIEW_RANGE",\
                                                                     (char*)"LANE_QUALITY",\
                                                                     (char*)"LANE_D",\
                                                                     (char*)"LANE_C"}));                                                                            

  msg_list.push_back(make_tuple((char*)"LEFT_LANE_A", vector<char*> {(char*)"LANE_B",\
                                                                     (char*)"LANE_A"}));

  msg_list.push_back(make_tuple((char*)"REAR_03", vector<char*> {(char*)"TRACK_ID",\
                                                                 (char*)"TRACK_STATUS",\
                                                                 (char*)"TRACK_VY",\
                                                                 (char*)"TRACK_VX",\
                                                                 (char*)"TRACK_Y",\
                                                                 (char*)"TRACK_X"}));

  msg_list.push_back(make_tuple((char*)"REAR_03_ADD", vector<char*> {(char*)"TRACK_ID",\
                                                                     (char*)"TRACK_AY",\
                                                                     (char*)"TRACK_AX",\
                                                                     (char*)"TRACK_ORIENTATION",\
                                                                     (char*)"TRACK_SIZE_Y",\
                                                                     (char*)"TRACK_SIZE_X"}));

  msg_list.push_back(make_tuple((char*)"REAR_02", vector<char*> {(char*)"TRACK_ID",\
                                                                 (char*)"TRACK_STATUS",\
                                                                 (char*)"TRACK_VY",\
                                                                 (char*)"TRACK_VX",\
                                                                 (char*)"TRACK_Y",\
                                                                 (char*)"TRACK_X"}));

  msg_list.push_back(make_tuple((char*)"REAR_02_ADD", vector<char*> {(char*)"TRACK_ID",\
                                                                     (char*)"TRACK_AY",\
                                                                     (char*)"TRACK_AX",\
                                                                     (char*)"TRACK_ORIENTATION",\
                                                                     (char*)"TRACK_SIZE_Y",\
                                                                     (char*)"TRACK_SIZE_X"}));

  msg_list.push_back(make_tuple((char*)"REAR_01", vector<char*> {(char*)"TRACK_ID",\
                                                                 (char*)"TRACK_STATUS",\
                                                                 (char*)"TRACK_VY",\
                                                                 (char*)"TRACK_VX",\
                                                                 (char*)"TRACK_Y",\
                                                                 (char*)"TRACK_X"}));

  msg_list.push_back(make_tuple((char*)"REAR_01_ADD", vector<char*> {(char*)"TRACK_ID",\
                                                                     (char*)"TRACK_AY",\
                                                                     (char*)"TRACK_AX",\
                                                                     (char*)"TRACK_ORIENTATION",\
                                                                     (char*)"TRACK_SIZE_Y",\
                                                                     (char*)"TRACK_SIZE_X"}));

  msg_list.push_back(make_tuple((char*)"CORNER_RIGHT", vector<char*> {(char*)"TRACK_ID",\
                                                                      (char*)"TRACK_STATUS",\
                                                                      (char*)"TRACK_VY",\
                                                                      (char*)"TRACK_VX",\
                                                                      (char*)"TRACK_Y",\
                                                                      (char*)"TRACK_X"}));

  msg_list.push_back(make_tuple((char*)"CORNER_RIGHT_ADD", vector<char*> {(char*)"TRACK_ID",\
                                                                          (char*)"TRACK_AY",\
                                                                          (char*)"TRACK_AX",\
                                                                          (char*)"TRACK_ORIENTATION",\
                                                                          (char*)"TRACK_SIZE_Y",\
                                                                          (char*)"TRACK_SIZE_X"}));

  msg_list.push_back(make_tuple((char*)"CORNER_LEFT", vector<char*> {(char*)"TRACK_ID",\
                                                                     (char*)"TRACK_STATUS",\
                                                                     (char*)"TRACK_VY",\
                                                                     (char*)"TRACK_VX",\
                                                                     (char*)"TRACK_Y",\
                                                                     (char*)"TRACK_X"}));

  msg_list.push_back(make_tuple((char*)"CORNER_LEFT_ADD", vector<char*> {(char*)"TRACK_ID",\
                                                                         (char*)"TRACK_AY",\
                                                                         (char*)"TRACK_AX",\
                                                                         (char*)"TRACK_ORIENTATION",\
                                                                         (char*)"TRACK_SIZE_Y",\
                                                                         (char*)"TRACK_SIZE_X"}));

  msg_list.push_back(make_tuple((char*)"FRONT_RIGHT", vector<char*> {(char*)"TRACK_ID",\
                                                                     (char*)"TRACK_STATUS",\
                                                                     (char*)"TRACK_VY",\
                                                                     (char*)"TRACK_VX",\
                                                                     (char*)"TRACK_Y",\
                                                                     (char*)"TRACK_X"}));

  msg_list.push_back(make_tuple((char*)"FRONT_RIGHT_ADD", vector<char*> {(char*)"TRACK_ID",\
                                                                         (char*)"TRACK_AY",\
                                                                         (char*)"TRACK_AX",\
                                                                         (char*)"TRACK_ORIENTATION",\
                                                                         (char*)"TRACK_SIZE_Y",\
                                                                         (char*)"TRACK_SIZE_X"}));

  msg_list.push_back(make_tuple((char*)"FRONT_CENTER", vector<char*> {(char*)"TRACK_ID",\
                                                                      (char*)"TRACK_STATUS",\
                                                                      (char*)"TRACK_VY",\
                                                                      (char*)"TRACK_VX",\
                                                                      (char*)"TRACK_Y",\
                                                                      (char*)"TRACK_X"}));

  msg_list.push_back(make_tuple((char*)"FRONT_CENTER_ADD", vector<char*> {(char*)"TRACK_ID",\
                                                                          (char*)"TRACK_AY",\
                                                                          (char*)"TRACK_AX",\
                                                                          (char*)"TRACK_ORIENTATION",\
                                                                          (char*)"TRACK_SIZE_Y",\
                                                                          (char*)"TRACK_SIZE_X"}));

  msg_list.push_back(make_tuple((char*)"FRONT_LEFT", vector<char*> {(char*)"TRACK_ID",\
                                                                    (char*)"TRACK_STATUS",\
                                                                    (char*)"TRACK_VY",\
                                                                    (char*)"TRACK_VX",\
                                                                    (char*)"TRACK_Y",\
                                                                    (char*)"TRACK_X"}));

  msg_list.push_back(make_tuple((char*)"FRONT_LEFT_ADD", vector<char*> {(char*)"TRACK_ID",\
                                                                        (char*)"TRACK_AY",\
                                                                        (char*)"TRACK_AX",\
                                                                        (char*)"TRACK_ORIENTATION",\
                                                                        (char*)"TRACK_SIZE_Y",\
                                                                        (char*)"TRACK_SIZE_X"}));
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  for(short i=0; i!=look_ahead_dist+1; i++){
    x_range_front.push_back(i);
    lane_points_front.push_back(0);
  }

  // for(short i=-look_behind_dist; i!=0; i++){
  for(short i=0; i!=look_behind_dist; --i){
    x_range_rear.push_back(-i);
    lane_points_rear.push_back(0);
  }

}


//**************************** new ************************************//
void TRACK_CAN_WRITER::CALLBACK_BOX(const mmc_msgs::object_array_msg& data){

  track_box->data.clear();
  *track_box = data;

  track_fault = true;
  
}

int TRACK_CAN_WRITER::Nearest_idx(const float x, float y){

  float dist = 999;
  int idx = -1;

  if (track_fault){
    for (int i=0; i<track_box->data.size();i++){
      
      float box_x = track_box->data[i].x;
      float box_y = track_box->data[i].y;

      float temp_dist = sqrt(pow(box_x-x,2) + 2*pow(box_y-y,2)); // mahal

      // if (temp_dist<dist){
      if (temp_dist<dist && abs(box_x-x)<7 && abs(box_y-y)<1.5){
        idx = i;
      }
    }
  }

  return idx;
}
//**************************** new ************************************//


void TRACK_CAN_WRITER::CALLBACK_FRONT_RADAR(const mmc_msgs::object_array_msg& data){
  sensors_front_radar_time = ros::Time::now();
}


void TRACK_CAN_WRITER::CALLBACK_FRONT_LIDAR(const mmc_msgs::object_array_msg& data){
  sensors_front_lidar_time = ros::Time::now();
}


void TRACK_CAN_WRITER::CALLBACK_VISION_POS(const mmc_msgs::object_array_msg& data){
  sensors_front_vision_time = ros::Time::now();
}

void TRACK_CAN_WRITER::CALLBACK_V2V(const mmc_msgs::object_array_msg& data){
  sensors_front_V2V_time = ros::Time::now();
}


void TRACK_CAN_WRITER::CALLBACK_CHASSIS(const mmc_msgs::chassis_msg& data){
  speed_ego = data.speed;
}

void TRACK_CAN_WRITER::CALLBACK_VISION_LANE(const mmc_msgs::lane_array_msg& data){
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
    lane.width = fabs(lane.left.d) + fabs(lane.right.d);

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

  for(short i=0; i!=x_range_front.size(); i++){
    lane_points_front[i] = GET_FUNC_VAL(lane, x_range_front[i]);
  }

  for(short i=0; i!=x_range_rear.size(); i++){
    lane_points_rear[i] = GET_FUNC_VAL(lane, x_range_rear[i]);
  }

  // cout<<"lane_points_rear"<<endl;
  // for(short i=0; i!=lane_points_rear.size(); i++){
  //   cout<<lane_points_rear[i]<<", ";
  // }
  // cout<<endl;

  // CAN write: lane left
  unsigned char can_data[dlc];
  unsigned short int msg_idx;
  char* target_msg;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  unsigned int id_write;
  unsigned int flag = 0;

  target_msg = (char*)"LEFT_LANE_A";  
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  vector<double> LEFT_LANE_A( get<1>(msg_list[msg_idx]).size() );
  LEFT_LANE_A = {lane.left.b, lane.left.a};

  for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), LEFT_LANE_A[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));

  target_msg = (char*)"LEFT_LANE_B";
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  vector<double> LEFT_LANE_B( get<1>(msg_list[msg_idx]).size() );
  LEFT_LANE_B = {lane.left.view_range, (double)lane.left.quality, lane.left.d, lane.left.c};

  for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), LEFT_LANE_B[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, 0);
  memset(can_data, 0, sizeof(can_data));

  // CAN write: lane right
  target_msg = (char*)"RIGHT_LANE_A";
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  vector<double> RIGHT_LANE_A( get<1>(msg_list[msg_idx]).size() );
  RIGHT_LANE_A = {lane.right.b, lane.right.a};

  for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), RIGHT_LANE_A[i]);
  }
  
  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));

  target_msg = (char*)"RIGHT_LANE_B";
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  vector<double> RIGHT_LANE_B( get<1>(msg_list[msg_idx]).size() );
  RIGHT_LANE_B = {lane.right.view_range, (double)lane.right.quality, lane.right.d, lane.right.c};

  for(short i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), RIGHT_LANE_B[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, 0);
  memset(can_data, 0, sizeof(can_data));
}


void TRACK_CAN_WRITER::CALLBACK_TRACK(const mmc_msgs::object_array_msg& data){
  track_calling = true;

  track_time = data.time;
  track_stored->data.clear();
  *track_stored = data;

  track_calling = false;
}


void TRACK_CAN_WRITER::TRACK_FRONT_PROCESS(mmc_msgs::object_array_msg& track, const vector<short>& idx){
  short x_round;
  vector<short> occupancy_grid_flag(idx.size());
  mmc_msgs::object_msg ref;
  
  for(short i=0; i!=idx.size(); i++){
    ref = track.data[idx[i]];
    x_round = (short)round(ref.x);

    if(x_round > x_range_front.back()){
      x_round = x_range_front.back();
    }

    // cout<<"###### x = "<<ref.x<<",  y = "<<ref.y<<endl;
    // cout<<"thres = "<<lane_points_front[x_round]<<endl;
    // cout<<"width = "<<lane.width<<endl;
    // cout<<"calcul = "<<lane_points_front[x_round] - lane.width * 0.5f<<endl;


    if(ref.y > lane_points_front[x_round] + lane.width * 1.5f){
      occupancy_grid_flag[i] = -1; // out of the grid
      // cout<<"out of grid"<<endl;

    }else if(ref.y > lane_points_front[x_round] + lane.width * 0.5f){
      occupancy_grid_flag[i] = 0; // front left
      // cout<<"left @@@@"<<endl;

    }else if(ref.y > lane_points_front[x_round] - lane.width * 0.5f){
      occupancy_grid_flag[i] = 1; // front center
      // cout<<"center @@@@"<<endl;

    }else if(ref.y > lane_points_front[x_round] - lane.width * 1.5f){
      occupancy_grid_flag[i] = 2; // front right
      // cout<<"right @@@@"<<endl;

    }else{
      occupancy_grid_flag[i] = -1; // out of the grid
      // cout<<"out of grid"<<endl;
    }
  }

  bool front_center_track_exists = false;

  for(short i=0; i!=occupancy_grid_flag.size(); i++){

    if(occupancy_grid_flag[i] == 1){
      front_center_track_exists = true;
    }
  }

  for(int i=0; i!=occupancy_grid_flag.size(); i++){
    cout<<occupancy_grid_flag[i]<<", ";
  }
  cout<<endl;

  // 일단 기능을 없애는게 맞는 거 같은데 지우지는 말자
  // if(front_center_track_exists == false){

  //   for(short i=0; i!=occupancy_grid_flag.size(); i++){
  //     ref = track.data[idx[i]];

  //     // considering the lane measurement uncertainty, regard a outlier which is far(further 80m) from the ego vehicle and also is in y-axis boundary(lane_width*1.5) to a front track
  //     if((occupancy_grid_flag[i] == -1 || ref.x >= 80.0) && (fabs(ref.y) <= default_lane_width * 1.5)){
  //       occupancy_grid_flag[i] = 1;
  //     }
  //   }
  // }

  vector<short> idx_nearest = {1000, 1000, 1000}; // order: left, center, right

  // sorting nearest tracks with position x
  for(short i=0; i!=occupancy_grid_flag.size(); i++){
    ref = track.data[idx[i]];

    if(occupancy_grid_flag[i] >= 0 && ref.x > 0.0){
      
      if(idx_nearest[occupancy_grid_flag[i]] == 1000){
        idx_nearest[occupancy_grid_flag[i]] = idx[i];

      }else if(ref.x < track.data[ idx_nearest[ occupancy_grid_flag[i] ] ].x){
        idx_nearest[occupancy_grid_flag[i]] = idx[i];
      }
    }
  }

  // publishing topics
  mmc_msgs::object_array_msg msg;
  mmc_msgs::object_msg temp_msg;
  msg.time = ros::Time::now();
  
  for(int i=0; i!=idx_nearest.size(); i++){

    if(idx_nearest[i] != 1000 && i==1){ // i==1 의미는 front center만 보겟다는 뜻
      ref = track.data[ idx_nearest[i] ];
      temp_msg.id = ref.id;
      temp_msg.x  = ref.x;
      temp_msg.y  = ref.y;
      temp_msg.vx = ref.vx;
      temp_msg.vy = ref.vy;
      temp_msg.ax = ref.ax;
      temp_msg.ay = ref.ay;

      // //**************************** new ************************************//
      // int idx = Nearest_idx(ref.x, ref.y);
      // if (idx == -1){
      //   temp_msg.size_x = 0.0f;
      //   temp_msg.size_y = 0.0f;
      //   temp_msg.orientation = 0.0f;
      //   track.data[ idx_nearest[i] ].size_x = 0.0f;
      //   track.data[ idx_nearest[i] ].size_y = 0.0f;
      //   track.data[ idx_nearest[i] ].orientation = 0.0f;

      // }else{
      //   temp_msg.size_x = track_box->data[idx].size_x;
      //   temp_msg.size_y = track_box->data[idx].size_y;
      //   temp_msg.orientation = track_box->data[idx].orientation;
      //   track.data[ idx_nearest[i] ].size_x = track_box->data[idx].size_x;
      //   track.data[ idx_nearest[i] ].size_y = track_box->data[idx].size_y;
      //   track.data[ idx_nearest[i] ].orientation = track_box->data[idx].orientation;
      // }

      // // cout<<track.data[ idx_nearest[i] ].x<<endl;
      // // cout<<track.data[ idx_nearest[i] ].size_x<<endl;
      // // cout<<track.data[ idx_nearest[i] ].size_y<<endl;

      // //**************************** new ************************************//

      msg.data.push_back(temp_msg);

    }else if(idx_nearest[i] == 1000 && i==1){
      temp_msg.id = 0;
      temp_msg.x  = 0;
      temp_msg.y  = 0;
      temp_msg.vx = 0;
      temp_msg.vy = 0;
      temp_msg.ax = 0;
      temp_msg.ay = 0;

      msg.data.push_back(temp_msg);
    }



    
  }

  pub1.publish(msg);

  // // CAN write: track front
  // unsigned char can_data[dlc];
  // char* target_msg;
  // char* target_msg_add;
  // unsigned short msg_idx;
  // KvaDbMessageHnd mh = 0;
  // KvaDbSignalHnd sh = 0;
  // vector<double> temp_data;
  // vector<double> temp_data_add;
  // unsigned int id_write;
  // unsigned int flag = 0;
  
  for(short i=0; i!=idx_nearest.size(); i++){

    // CAN write: track front
    unsigned char can_data[dlc];
    char* target_msg;
    char* target_msg_add;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data;
    vector<double> temp_data_add;
    unsigned int id_write;
    unsigned int flag = 0;


    switch(i){

      case(0):
        target_msg = (char*)"FRONT_LEFT";
        target_msg_add = (char*)"FRONT_LEFT_ADD";
      break;

      case(1):
        target_msg = (char*)"FRONT_CENTER";
        target_msg_add = (char*)"FRONT_CENTER_ADD";
      break;

      case(2):
        target_msg = (char*)"FRONT_RIGHT";
        target_msg_add = (char*)"FRONT_RIGHT_ADD";
      break;
    }

    if(idx_nearest[i] != 1000){
      ref = track.data[ idx_nearest[i] ];

      if(fabs(ref.y) < 40.0){
        // cout<<"target_msg = "<<target_msg<<endl;
        temp_data = {(double)ref.id, (double)1, ref.vy, ref.vx, ref.y, ref.x-3.85f-2.f}; // -3.85 : to_base_link - box_center(2m)
        temp_data_add = {(double)ref.id, ref.ay, ref.ax, ref.orientation, ref.size_y, ref.size_x};
        // cout<<"x = "<<ref.x<<endl;
        // cout<<"y = "<<ref.y<<endl;

      }else{
        temp_data = {0, 0, 0, 0, 0, 0};
        temp_data_add = {0, 0, 0, 0, 0, 0};

      }

    }else{
      temp_data = {0, 0, 0, 0, 0, 0};
      temp_data_add = {0, 0, 0, 0, 0, 0};

    }

    


    // original message(x,y,vx,vy,id,status)
    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[i]);
    }

    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));

    // additional message(size_x,size_y,orientation,ax,ay,id)
    msg_idx = FIND_MSG_IDX(target_msg_add, &msg_list);
    kvaDbGetMsgByName(dh, target_msg_add, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_add[i]);
    }

    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));
  }
}


void TRACK_CAN_WRITER::TRACK_CORNER_LEFT_PROCESS(mmc_msgs::object_array_msg& track, const vector<short>& idx){
  short idx_nearest = 1000;
  mmc_msgs::object_msg ref;
  
  // choosing nearest tracks with position x
  for(short i=0; i!=idx.size(); i++){
    ref = track.data[ idx.at(i) ];
      
    if(idx_nearest == 1000){
      idx_nearest = idx[i];

    }else if(ref.x < track.data[idx_nearest].x){
      idx_nearest = idx[i];
    }
  }

  // // publishing topics
  // mmc_msgs::object_array_msg msg;
  // mmc_msgs::object_msg temp_msg;
  // msg.time = ros::Time::now();
  
  // if(idx_nearest != 1000){
  //   ref = track.data[idx_nearest];
  //   temp_msg.id = ref.id;
  //   temp_msg.x  = ref.x;
  //   temp_msg.y  = ref.y;
  //   temp_msg.vx = ref.vx;
  //   temp_msg.vy = ref.vy;
  //   temp_msg.ax = ref.ax;
  //   temp_msg.ay = ref.ay;

  //   // //**************************** new ************************************//
  //   // int idx = Nearest_idx(ref.x, ref.y);
  //   // if (idx == -1){
  //   //   temp_msg.size_x = 4.0f;
  //   //   temp_msg.size_y = 2.0f;
  //   //   temp_msg.orientation = 0.0f;
  //   //   track.data[ idx_nearest ].size_x = 4.0f;
  //   //   track.data[ idx_nearest ].size_y = 2.0f;
  //   //   track.data[ idx_nearest ].orientation = 0.0f;

  //   // }else{
  //   //   temp_msg.size_x = track_box->data[idx].size_x;
  //   //   temp_msg.size_y = track_box->data[idx].size_y;
  //   //   temp_msg.orientation = track_box->data[idx].orientation;
  //   //   track.data[ idx_nearest ].size_x = track_box->data[idx].size_x;
  //   //   track.data[ idx_nearest ].size_y = track_box->data[idx].size_y;
  //   //   track.data[ idx_nearest ].orientation = track_box->data[idx].orientation;
  //   // }
  //   // //**************************** new ************************************//

  //   msg.data.push_back(temp_msg);
  // }

  // pub2.publish(msg);

  // CAN write: track corner left
  unsigned char can_data[dlc];
  char* target_msg;
  char* target_msg_add;
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  vector<double> temp_data_add;
  unsigned int id_write;
  unsigned int flag = 0;

  target_msg = (char*)"CORNER_LEFT";
  target_msg_add = (char*)"CORNER_LEFT_ADD";
  
  if(idx_nearest != 1000){
    ref = track.data[idx_nearest];
    temp_data = {(double)ref.id, (double)1, ref.vy, ref.vx, ref.y, ref.x};
    temp_data_add = {(double)ref.id, ref.ay, ref.ax, ref.orientation, ref.size_y, ref.size_x};
    
  }else{
    temp_data = {0, 0, 0, 0, 0, 0};
    temp_data_add = {0, 0, 0, 0, 0, 0};
  }

  // original message(x,y,vx,vy,id,status)
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));

  // additional message(size_x,size_y,orientation,ax,ay,id)
  msg_idx = FIND_MSG_IDX(target_msg_add, &msg_list);
  kvaDbGetMsgByName(dh, target_msg_add, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_add[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));
}


void TRACK_CAN_WRITER::TRACK_CORNER_RIGHT_PROCESS(mmc_msgs::object_array_msg& track, const vector<short>& idx){
  short idx_nearest = 1000;
  mmc_msgs::object_msg ref;
  
  // choosing nearest tracks with position x
  for(short i=0; i!=idx.size(); i++){
    ref = track.data[idx[i]];
      
    if(idx_nearest == 1000){
      idx_nearest = idx[i];

    }else if(ref.x < track.data[idx_nearest].x){
      idx_nearest = idx[i];
    }
  }

  // // publishing topics
  // mmc_msgs::object_array_msg msg;
  // mmc_msgs::object_msg temp_msg;
  // msg.time = ros::Time::now();
  
  // if(idx_nearest != 1000){
  //   ref = track.data[idx_nearest];
  //   temp_msg.id = ref.id;
  //   temp_msg.x  = ref.x;
  //   temp_msg.y  = ref.y;
  //   temp_msg.vx = ref.vx;
  //   temp_msg.vy = ref.vy;
  //   temp_msg.ax = ref.ax;
  //   temp_msg.ay = ref.ay;

  //   //**************************** new ************************************//
  //   int idx = Nearest_idx(ref.x, ref.y);
  //   if (idx == -1){
  //     temp_msg.size_x = 4.0f;
  //     temp_msg.size_y = 2.0f;
  //     temp_msg.orientation = 0.0f;
  //     track.data[ idx_nearest ].size_x = 4.0f;
  //     track.data[ idx_nearest ].size_y = 2.0f;
  //     track.data[ idx_nearest ].orientation = 0.0f;

  //   }else{
  //     temp_msg.size_x = track_box->data[idx].size_x;
  //     temp_msg.size_y = track_box->data[idx].size_y;
  //     temp_msg.orientation = track_box->data[idx].orientation;
  //     track.data[ idx_nearest ].size_x = track_box->data[idx].size_x;
  //     track.data[ idx_nearest ].size_y = track_box->data[idx].size_y;
  //     track.data[ idx_nearest ].orientation = track_box->data[idx].orientation;
  //   }
  //   //**************************** new ************************************//

  //   msg.data.push_back(temp_msg);
  // }

  // pub3.publish(msg);

  // CAN write: track corner right
  unsigned char can_data[dlc];
  char* target_msg;
  char* target_msg_add;
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  vector<double> temp_data_add;
  unsigned int id_write;
  unsigned int flag = 0;

  target_msg = (char*)"CORNER_RIGHT";
  target_msg_add = (char*)"CORNER_RIGHT_ADD";
  
  if(idx_nearest != 1000){
    ref = track.data[idx_nearest];
    temp_data = {(double)ref.id, (double)1, ref.vy, ref.vx, ref.y, ref.x-2}; // box_center(2m)
    temp_data_add = {(double)ref.id, ref.ay, ref.ax, ref.orientation, ref.size_y, ref.size_x};
    
  }else{
    temp_data = {0, 0, 0, 0, 0, 0};
    temp_data_add = {0, 0, 0, 0, 0, 0};
  }

  // original message(x,y,vx,vy,id,status)
  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));

  // additional message(size_x,size_y,orientation,ax,ay,id)
  msg_idx = FIND_MSG_IDX(target_msg_add, &msg_list);
  kvaDbGetMsgByName(dh, target_msg_add, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_add[i]);
  }

  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
  memset(can_data, 0, sizeof(can_data));
}


void TRACK_CAN_WRITER::TRACK_REAR_PROCESS(mmc_msgs::object_array_msg& track, const vector<short>& idx){
  short x_round;
  vector<short> occupancy_grid_flag(idx.size());
  mmc_msgs::object_msg ref;
  
  // cout<<"lane_points_rear"<<endl;
  // for(short i=0; i!=lane_points_rear.size(); i++){
  //   cout<<lane_points_rear[i]<<", ";
  // }
  // // cout<<endl;
  // cout<<"lane width : "<<lane.width<<endl;
  // cout<<"occupancy_grid_flag"<<endl;
  for(short i=0; i!=idx.size(); i++){
    ref = track.data[idx[i]];
    x_round = (short)round(abs(ref.x));
    
    if(x_round < x_range_rear[0]){
      x_round = x_range_rear[0];
    }

    if(ref.y > lane_points_rear[x_round] + lane.width * 1.5f){
      occupancy_grid_flag[i] = -1; // out of the grid

    }else if(ref.y > lane_points_rear[x_round] + lane.width * 0.5f){
      occupancy_grid_flag[i] = 0; // rear left

    }else if(ref.y > lane_points_rear[x_round] - lane.width * 0.5f){
      occupancy_grid_flag[i] = 1; // rear center

    }else if(ref.y > lane_points_rear[x_round] - lane.width * 1.5f){
      occupancy_grid_flag[i] = 2; // rear right

    }else{
      occupancy_grid_flag[i] = -1; // out of the grid

    }

    // cout<<occupancy_grid_flag[i]<<", x = "<<ref.x<<", y = "<<ref.y<<", thres = "<<lane_points_rear[x_round]<<endl;
  }

  // cout<<endl;

  // bool rear_center_track_exists = false;

  // for(short i=0; i!=occupancy_grid_flag.size(); i++){

  //   if(occupancy_grid_flag[i]==1){
  //     rear_center_track_exists = true;
  //   }
  // }
  
  // if(rear_center_track_exists == false){

  //   for(short i=0; i!=occupancy_grid_flag.size(); i++){
  //     ref = track->data[idx[i]];
      
  //     if((occupancy_grid_flag[i] == -1 || ref.x >= 80.0) && (fabs(ref.y) <= default_lane_width * 1.5)){
  //       occupancy_grid_flag[i] = 1;
  //     }
  //   }
  // }
  
  vector<short> idx_nearest = {1000, 1000, 1000}; // order: left, center, right
  
  // sorting nearest tracks with position x
  for(short i=0; i!=occupancy_grid_flag.size(); i++){
    ref = track.data[idx[i]];
    
    if(occupancy_grid_flag[i] >= 0 && ref.x < 0.0){
      
      if(idx_nearest[occupancy_grid_flag[i]] == 1000){
        idx_nearest[occupancy_grid_flag[i]] = idx[i];

      }else if(ref.x > track.data[ idx_nearest[ occupancy_grid_flag[i] ] ].x){
        idx_nearest[occupancy_grid_flag[i]] = idx[i];
      }
    }
  }

  // // publishing topics
  // mmc_msgs::object_array_msg msg;
  // mmc_msgs::object_msg temp_msg;
  // msg.time = ros::Time::now();
  
  // for(int i=0; i!=idx_nearest.size(); i++){

  //   if(idx_nearest[i] != 1000 && i ==1){
  //     ref = track.data[ idx_nearest[i] ];
  //     temp_msg.id = ref.id;
  //     temp_msg.x  = ref.x;
  //     temp_msg.y  = ref.y;
  //     temp_msg.vx = ref.vx;
  //     temp_msg.vy = ref.vy;
  //     temp_msg.ax = ref.ax;
  //     temp_msg.ay = ref.ay;

  //     // //**************************** new ************************************//
  //     // int idx = Nearest_idx(ref.x, ref.y);
  //     // if (idx == -1){
  //     //   temp_msg.size_x = 4.0f;
  //     //   temp_msg.size_y = 2.0f;
  //     //   temp_msg.orientation = 0.0f;
  //     //   track.data[ idx_nearest[i] ].size_x = 4.0f;
  //     //   track.data[ idx_nearest[i] ].size_y = 2.0f;
  //     //   track.data[ idx_nearest[i] ].orientation = 0.0f;

  //     // }else{
  //     //   temp_msg.size_x = track_box->data[idx].size_x;
  //     //   temp_msg.size_y = track_box->data[idx].size_y;
  //     //   temp_msg.orientation = track_box->data[idx].orientation;
  //     //   track.data[ idx_nearest[i] ].size_x = track_box->data[idx].size_x;
  //     //   track.data[ idx_nearest[i] ].size_y = track_box->data[idx].size_y;
  //     //   track.data[ idx_nearest[i] ].orientation = track_box->data[idx].orientation;
  //     // }
  //     // //**************************** new ************************************//

  //     msg.data.push_back(temp_msg);
  //   }
  // }

  // pub4.publish(msg);

  // CAN write: track rear
  unsigned char can_data[dlc];
  char* target_msg;
  char* target_msg_add;
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  vector<double> temp_data;
  vector<double> temp_data_add;
  unsigned int id_write;
  unsigned int flag = 0;

  // cout<<endl;
  for(short i=0; i!=idx_nearest.size(); i++){

    switch(i){

      case(0):
        target_msg = (char*)"REAR_01";
        target_msg_add = (char*)"REAR_01_ADD";

        // if(idx_nearest[i] != 1000){
        //   mmc_msgs::object_msg test1 = track.data[ idx_nearest[i] ];
        //   cout<<"L   x = "<<test1.x<<",  y = "<<test1.y<<endl;
        // }
        
      break;

      case(1):
        target_msg = (char*)"REAR_02";
        target_msg_add = (char*)"REAR_02_ADD";

        // if(idx_nearest[i] != 1000){
        //   mmc_msgs::object_msg test1 = track.data[ idx_nearest[i] ];
        //   cout<<"C   x = "<<test1.x<<",  y = "<<test1.y<<endl;
        // }

      break;

      case(2):
        target_msg = (char*)"REAR_03";
        target_msg_add = (char*)"REAR_03_ADD";

        // if(idx_nearest[i] != 1000){
        //   mmc_msgs::object_msg test1 = track.data[ idx_nearest[i] ];
        //   cout<<"R   x = "<<test1.x<<",  y = "<<test1.y<<endl;
        // }

      break;
    }
    
    if(idx_nearest[i] != 1000 && fabs(ref.y) < 40.0){
      ref = track.data[ idx_nearest[i] ];
      temp_data = {(double)ref.id, (double)1, ref.vy, ref.vx, ref.y, ref.x};
      temp_data_add = {(double)ref.id, ref.ay, ref.ax, ref.orientation, ref.size_y, ref.size_x};
      
    }else{
      temp_data = {0, 0, 0, 0, 0, 0};
      temp_data_add = {0, 0, 0, 0, 0, 0};
    }

    // original message(x,y,vx,vy,id,status)
    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[i]);
    }

    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));

    // additional message(size_x,size_y,orientation,ax,ay,id)
    msg_idx = FIND_MSG_IDX(target_msg_add, &msg_list);
    kvaDbGetMsgByName(dh, target_msg_add, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_add[i]);
    }

    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));
  }
}


short TRACK_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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


void TRACK_CAN_WRITER::GET_DATETIME(){
  ftime(&international_time);
  local_time = localtime(&international_time.time);

  time_now.millisecond = international_time.millitm;
  time_now.second = local_time->tm_sec;
  time_now.minute = local_time->tm_min;
  time_now.hour = local_time->tm_hour;
  time_now.day = local_time->tm_mday;
  time_now.month = local_time->tm_mon + 1;
  time_now.year = local_time->tm_year + 1900;

  // time_t curr_time;
	// curr_time = time(NULL);

	// tm *tm_local = localtime(&curr_time);
	// cout << "Current local time : " << tm_local->tm_hour << ":" << tm_local->tm_min << ":" << tm_local->tm_sec;

} 


float TRACK_CAN_WRITER::GET_FUNC_VAL(const struct lane& lane, const short& x){
  // the y-axis of Mobileye coordinates is the opposite of the y-axis of ISO coordinates
  // float func_val = (lane.center.a * pow(x, 3) + lane.center.b * pow(x, 2) + lane.center.c * x + lane.center.d);
  /////////////// revised
  float func_val = (lane.center.a * pow(x, 3) + lane.center.b * pow(x, 2) + lane.center.c * x + lane.center.d);
  // cout<<endl;
  // cout<<"x = "<<x<<endl;
  // cout<<"lane.center.a = "<<lane.center.a<<endl;
  // cout<<"lane.center.b = "<<lane.center.b<<endl;
  // cout<<"lane.center.c = "<<lane.center.c<<endl;
  // cout<<"lane.center.d = "<<lane.center.d<<endl;
  // cout<<endl;
  // cout<<"lane.center.a * pow(x, 3) = "<<lane.center.a * pow(x, 3)<<endl;
  // cout<<"lane.center.b * pow(x, 2) = "<<lane.center.b * pow(x, 2)<<endl;
  // cout<<"lane.center.c * x = "<<lane.center.c * x<<endl;
  // cout<<"lane.center.d = "<<lane.center.d<<endl;
  // cout<<"func_val = "<<func_val<<endl;
  return func_val;
}


// canStatus TRACK_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
//   canInitializeLibrary();

//   int open_flag;

//   if(init_access_flag == true){
//       open_flag = canOPEN_REQUIRE_INIT_ACCESS;

//     }else{
//       open_flag = canOPEN_NO_INIT_ACCESS;
//     }


//     cout<<"Opening the channel "<<channel_num<<"..."<<endl;
//     hCAN = canOpenChannel(channel_num, open_flag);
  
//   if(hCAN == canOK){
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

//   if(hCAN == canOK){
//     return canOK;

//   }else{
//     return canERR_NOMSG;
//   }
// }


void TRACK_CAN_WRITER::LOOP(){
  ros::Rate rate(freq_for_channel_2);

  while(ros::ok()){

    time1 = ros::Time::now();

    // if(track.time.toSec() < 10){
    //   dt = ros::Duration(0);

    // }else{
    //   dt = time1 - track.time;
    // }
    
    if(track_calling == false){
      track.data.clear();
      track = *track_stored;
    }

    if(track.time.toSec() < 10){
      dt = ros::Duration(0);

    }else{
      dt = time1 - track.time;
    }

    mmc_msgs::object_msg ref;
    vector<short> idx_front, idx_left, idx_right, idx_rear;

    // cout<<"dt = "<<dt<<endl;

    for(short i=0; i!=track.data.size(); i++){ // predicting the state with the CV model
      ref = track.data[i];
      ref.x += (ref.vx - speed_ego) * dt.toSec();
      ref.y += ref.vy * dt.toSec();

      if      (ref.x >  200) ref.x =  200;
      else if (ref.x < -200) ref.x = -200;
      
      if      (ref.y >  40) ref.y =  40;
      else if (ref.y < -40) ref.y = -40;

      track.data[i] = ref;

      // sorting by an occupancy grid
      if(ref.x > 6.f){
        idx_front.push_back(i);

      }else if(ref.x > -2.f && ref.y >= 0.f){
        idx_left.push_back(i);

      }else if(ref.x > -2.f && ref.y < 0.f){
        idx_right.push_back(i);

      }else{
        idx_rear.push_back(i);
      }
    }

    TRACK_FRONT_PROCESS(track, idx_front);
    TRACK_CORNER_LEFT_PROCESS(track, idx_left);
    TRACK_CORNER_RIGHT_PROCESS(track, idx_right);
    TRACK_REAR_PROCESS(track, idx_rear);
    
    // for 공인인증평가 sensor fault scenarios ////////////////////////////////////
    ros::NodeHandle nh;
    nh.param<bool>("param_radar_fault_input",  param_radar_fault_input,  false);
    nh.param<bool>("param_lidar_fault_input",  param_lidar_fault_input,  false);
    nh.param<bool>("param_vision_fault_input", param_vision_fault_input, false);
    nh.param<bool>("param_V2V_fault_input",    param_V2V_fault_input,    false);
    /////////////////////////////////////////////////////////////////////////////
    
    radar_fault_input = 0;
    lidar_fault_input = 0;
    vision_fault_input = 0;
    V2V_fault_input = 0;
    
    if(time1 - sensors_front_radar_time > alive_timeout || param_radar_fault_input == true){
      radar_fault_input = 1;
    }

    if(time1 - sensors_front_lidar_time > alive_timeout || param_lidar_fault_input == true){
      lidar_fault_input = 1;
    }

    if(time1 - sensors_front_vision_time > alive_timeout || param_vision_fault_input == true){
      vision_fault_input = 1;
    }

    // if(time1 - sensors_front_V2V_time > alive_timeout || param_V2V_fault_input == true){
    if(param_V2V_fault_input == true){
    
      V2V_fault_input = 1;
    }


    // for 공인인증평가 radar fault signal
    if(radar_fault_input==0){
      time_alive_radar = ros::Time::now();

    }else if(radar_fault_input==1){
      time_fault_radar = ros::Time::now();
    }

    if(time_fault_radar - time_alive_radar > ros::Duration(0.5)){
      radar_fault_detect = 0;

    }else if(time_alive_radar - time_fault_radar> ros::Duration(0.1)){
      radar_fault_detect = 1;
    }

    // for 공인인증평가 lidar fault signal
    if(lidar_fault_input==0){
      time_alive_lidar = ros::Time::now();

    }else if(lidar_fault_input==1){
      time_fault_lidar = ros::Time::now();
    }

    if(time_fault_lidar - time_alive_lidar > ros::Duration(0.5)){
      lidar_fault_detect = 0;

    }else if(time_alive_lidar - time_fault_lidar> ros::Duration(0.1)){
      lidar_fault_detect = 1;
    }

    // for 공인인증평가 vision fault signal
    if(vision_fault_input==0){
      time_alive_vision = ros::Time::now();

    }else if(vision_fault_input==1){
      time_fault_vision = ros::Time::now();
    }

    if(time_fault_vision - time_alive_vision > ros::Duration(0.5)){
      vision_fault_detect = 0;

    }else if(time_alive_vision - time_fault_vision> ros::Duration(0.1)){
      vision_fault_detect = 1;
    }

    // for 공인인증평가 V2V fault signal
    if(V2V_fault_input==0){
      time_alive_V2V = ros::Time::now();

    }else if(V2V_fault_input==1){
      time_fault_V2V = ros::Time::now();
    }

    if(time_fault_V2V - time_alive_V2V > ros::Duration(0.5)){
      V2V_fault_detect = 0;

    }else if(time_alive_V2V - time_fault_V2V> ros::Duration(0.1)){
      V2V_fault_detect = 1;
    }


    /////////////////////////////////////////////////////////////////////////////
  
    vector<int> temp_data = {(int)V2V_fault_input,\
                             (int)vision_fault_input,\
                             (int)radar_fault_input,\
                             (int)lidar_fault_input};

    unsigned char can_data_for_fault[1];
    unsigned char can_data[dlc];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    unsigned int id_write;
    unsigned int flag = 0;

    // CAN write: alive count
    target_msg = (char*)"FAULT_INPUT_MESSAGE";
    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data_for_fault, sizeof(can_data_for_fault), temp_data[i]);
    }
    
    canWrite(hCAN, id_write, &can_data_for_fault, 1, canMSG_STD);
    memset(can_data_for_fault, 0, sizeof(can_data_for_fault));
    
    // CAN write: fault signal
    target_msg = (char*)"FAULT_DETECT_MESSAGE";
    temp_data.clear();
    
    temp_data = {(int)V2V_fault_detect,\
                 (int)vision_fault_detect,\
                 (int)radar_fault_detect,\
                 (int)lidar_fault_detect};
    
    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data_for_fault, sizeof(can_data_for_fault), temp_data[i]);
    }
    
    canWrite(hCAN, id_write, &can_data_for_fault, 1, canMSG_STD);
    memset(can_data_for_fault, 0, sizeof(can_data_for_fault));

    // CAN write: time information
    target_msg= (char*)"TimeInformation";
    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    vector<double> TimeInformation( get<1>(msg_list[msg_idx]).size() );
    GET_DATETIME();

    TimeInformation = {time_now.millisecond,\
                       time_now.second,\
                       time_now.minute,\
                       time_now.hour,\
                       time_now.day,\
                       time_now.month,\
                       time_now.year};

    for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), TimeInformation[i]);
    }
    
    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));

    time2 = ros::Time::now();
    ros::Duration ex_time = (time2 - time1) * 1000;
    // cout<<"track CAN write execution time: "<<ex_time<<" ms"<<endl;

    rate.sleep();
  }
}


int main(int argc, char **argv){
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "Track_CAN_Writer");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  // strcpy(filename, (relative_path + "/dbc/sensor_fusion_result.dbc").c_str());
  strcpy(filename, (relative_path + "/dbc/sensor_fusion_result_revised.dbc").c_str());
  int channel_num = 1;
  bool init_access_flag = false; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  TRACK_CAN_WRITER TCW;

  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  

  ros::Subscriber sub1 = node.subscribe("/sensors/front_radar", 1, &TRACK_CAN_WRITER::CALLBACK_FRONT_RADAR, &TCW);
  ros::Subscriber sub2 = node.subscribe("/sensors/front_lidar", 1, &TRACK_CAN_WRITER::CALLBACK_FRONT_LIDAR, &TCW);
  ros::Subscriber sub3 = node.subscribe("/sensors/vision_pos",  1, &TRACK_CAN_WRITER::CALLBACK_VISION_POS,  &TCW);
  ros::Subscriber sub4 = node.subscribe("/sensors/vision_lane", 1, &TRACK_CAN_WRITER::CALLBACK_VISION_LANE, &TCW);
  ros::Subscriber sub5 = node.subscribe("/track_cukf",               1, &TRACK_CAN_WRITER::CALLBACK_TRACK,       &TCW);
  ros::Subscriber sub6 = node.subscribe("/point_cloud/object",  1, &TRACK_CAN_WRITER::CALLBACK_BOX,         &TCW);
  ros::Subscriber sub7 = node.subscribe("/sensors/V2V",         1, &TRACK_CAN_WRITER::CALLBACK_V2V,         &TCW);
  ros::Subscriber sub8 = node.subscribe("/sensors/chassis",     1, &TRACK_CAN_WRITER::CALLBACK_CHASSIS,     &TCW);

  TCW.pub1 = node.advertise<mmc_msgs::object_array_msg>("/track_front",        1);
  TCW.pub2 = node.advertise<mmc_msgs::object_array_msg>("/track_corner_left",  1);
  TCW.pub3 = node.advertise<mmc_msgs::object_array_msg>("/track_corner_right", 1);
  TCW.pub4 = node.advertise<mmc_msgs::object_array_msg>("/track_rear",         1);

  // overlay text /////////////////////////////////////////////////////////////////
  TCW.pub5 = node.advertise<jsk_rviz_plugins::OverlayText>("/rviz/jsk/senfu_CAN_status", 10, true);
  jsk_rviz_plugins::OverlayText msg;
  string can_status_str;

  if(can_status == canOK){
    can_status_str = "OK";

  }else{
    can_status_str = "ERR";
  }

  msg.text.append("- CAN SenFu .... " + can_status_str + "\n");

  std_msgs::ColorRGBA state_color;
  
  int32_t width = 250;
  int32_t height = width*2;

  msg.action = msg.ADD;
  msg.font = "DejaVu Sans Mono";
  msg.text_size = 12;
  msg.width = width;
  msg.height = height;
  msg.left = 10;
  msg.top = 500;

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
  TCW.pub5.publish(msg);

  TCW.LOOP();
  
  ros::waitForShutdown();   
  canBusOff(hCAN);
  canClose(hCAN);
}