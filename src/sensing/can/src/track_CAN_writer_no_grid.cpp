//can 1대1 매칭용//
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <string.h>
#include <vector>
#include <tuple>
#include <std_msgs/Header.h>
#include <perception_ros_msg/object_array_msg.h>
#include <perception_ros_msg/object_msg.h>
#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>
#include <canlib.h>
#include <kvaDbLib.h>
#include <can_pub.h>

using namespace std;

class RSLiDAR_CAN_WRITER {
public:
    ros::Subscriber sub1;
    ros::Publisher pub5;
    unsigned int kvaDb_flags = 0;
    unsigned int dlc = 8;
    double time1 = -1;
    float new_time = 0;
    int alive_count = 0;
    bool local_calling = false;
    vector<tuple<string, vector<string>>> msg_list;

    RSLiDAR_CAN_WRITER();
    void CALLBACK_TRACK(const perception_ros_msg::object_array_msg& data);
    short FIND_MSG_IDX(const string& target_msg);
    canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, const string& filename, bool init_access_flag);
    void LOOP();
};

RSLiDAR_CAN_WRITER::RSLiDAR_CAN_WRITER() {
    for (int i = 0; i < 14; i++) {
        msg_list.push_back(make_tuple("TRACK_" + to_string(i), vector<string>{"TRACK_ID", "TRACK_STATUS", "TRACK_X", "TRACK_Y", "TRACK_VX", "TRACK_VY"}));
        msg_list.push_back(make_tuple("TRACK_ADD_" + to_string(i), vector<string>{"TRACK_ID", "TRACK_AX", "TRACK_AY", "TRACK_ORIENTATION", "TRACK_SIZE_X", "TRACK_SIZE_Y"}));
        msg_list.push_back(make_tuple("TRACK_ADDii_" + to_string(i), vector<string>{"TRACK_ORIENTATION_V", "TRACK_NEAREST_X", "TRACK_NEAREST_Y", "TRACK_NEAREST_Z", "TRACK_ID"}));
    }
}

void RSLiDAR_CAN_WRITER::CALLBACK_TRACK(const perception_ros_msg::object_array_msg& msg) {
    local_calling = true;
    alive_count = (alive_count + 1) % 256;

    if (time1 == -1) {
        time1 = msg.time.sec % 10000 + msg.time.nsec / 1000000000.0;
    }
    new_time = (msg.time.sec % 10000 + msg.time.nsec / 1000000000.0) - time1;
    
    unsigned char can_data[dlc] = {0};
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    unsigned int id_write, flag = 0;

    for (size_t i = 0; i < 14; i++) {
        string track_msg = "TRACK_" + to_string(i);
        string track_add_msg = "TRACK_ADD_" + to_string(i);
        string track_addii_msg = "TRACK_ADDii_" + to_string(i);
        
        vector<double> track_data(6, 0.0);
        vector<double> track_add_data(6, 0.0);
        vector<double> track_addii_data(5, 0.0);
        
        if (i < msg.data.size()) {
            auto& obj = msg.data[i];
            track_data = {obj.id, obj.status, obj.x, obj.y, obj.vx, obj.vy};
            track_add_data = {obj.id, obj.ax, obj.ay, obj.orientation, obj.size_x, obj.size_y};
            track_addii_data = {obj.orientation_v, obj.nearest_point_x, obj.nearest_point_y, obj.nearest_point_z, obj.id};
        }
        
        for (int j = 0; j < 3; j++) {
            string target_msg = (j == 0) ? track_msg : ((j == 1) ? track_add_msg : track_addii_msg);
            short msg_idx = FIND_MSG_IDX(target_msg);
            if (msg_idx == -1) continue;
            kvaDbGetMsgByName(dh, target_msg.c_str(), &mh);
            kvaDbGetMsgId(mh, &id_write, &flag);
            
            vector<double>& temp_data = (j == 0) ? track_data : ((j == 1) ? track_add_data : track_addii_data);
            for (size_t k = 0; k < temp_data.size(); k++) {
                kvaDbGetSignalByName(mh, const_cast<char*>(std::get<1>(msg_list[msg_idx])[k].c_str()), &sh);
                kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[k]);
            }
            canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
            memset(can_data, 0, sizeof(can_data));
        }
    }
}

short RSLiDAR_CAN_WRITER::FIND_MSG_IDX(const string& target_msg) {
    for (size_t i = 0; i < msg_list.size(); i++) {
        if (std::get<0>(msg_list[i]) == target_msg) return i;
    }
    cerr << "No matched message! : " << target_msg << endl;
    return -1;
}

canStatus RSLiDAR_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, const string& filename, bool init_access_flag) {
    canInitializeLibrary();
    int open_flag = init_access_flag ? canOPEN_REQUIRE_INIT_ACCESS : canOPEN_NO_INIT_ACCESS;
    hCAN = canOpenChannel(channel_num, open_flag);
    if (hCAN < 0) {
        cerr << "Failed to open CAN channel " << channel_num << endl;
        return canERR_NOTFOUND;
    }
    canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
    canBusOn(hCAN);
    
    kvaDbOpen(&dh);
    kvaDbReadFile(dh, const_cast<char*>(filename.c_str()));
    kvaDbGetFlags(dh, &kvaDb_flags);
    return canOK;
}

void RSLiDAR_CAN_WRITER::LOOP() {
    ros::Rate rate(20);
    while (ros::ok()) {
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "LOCAL_CAN_WRITER");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    string relative_path = ros::package::getPath("can");
    string filename = relative_path + "/dbc/CANdb_IONIQev_PCAN2.dbc";
    int channel_num = 1;
    bool init_access_flag = false;
    
    RSLiDAR_CAN_WRITER RCW;
    RCW.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);
    RCW.sub1 = node.subscribe("/track_Multi_RS", 1, &RSLiDAR_CAN_WRITER::CALLBACK_TRACK, &RCW);
    RCW.LOOP();
    
    ros::waitForShutdown();
    canBusOff(hCAN);
    canClose(hCAN);
    return 0;
}




// 자차량 time 보상 //
// #include <stdio.h>
// #include <stdlib.h>
// #include <fstream>
// #include <iostream>

// #include <ros/ros.h>
// #include <ros/package.h>

// #include <string.h>
// #include <cstring>
// #include <vector>
// #include <tuple>
// #include <queue>
    
// #include <perception_ros_msg/object_array_msg.h>
// #include <perception_ros_msg/object_msg.h>
// #include <novatel_gps_msgs/Inspva.h>


// #include <algorithm>
// #include <math.h>
// #include <ctime>
// #include <time.h>
// #include <sys/timeb.h>
// #include <chrono>
// #include <thread>

// #include <kvaDbLib.h>
// #include <canlib.h>

// using namespace std;

// canHandle hCAN;
// canStatus can_status;
// KvaDbStatus kvaDb_status;
// KvaDbHnd dh = 0;

// #define TRACK_MAX_SIZE 14

// class TRACK_CAN_WRITER_NO_GRID{
//   public:
//     // ros::Subscriber sub1, sub2, sub3, sub4, sub5, sub6, sub7;
    
//     unsigned short freq_for_channel_2 = 1000; // Hz
    
//     ros::Time track_time = ros::Time::now();
//     ros::Time time1 = ros::Time::now();
//     ros::Time time2 = ros::Time::now();
//     ros::Duration alive_timeout = ros::Duration(1.0);
//     ros::Duration dt;

//     ros::Time time_now_ros_time = ros::Time::now();
//     ros::Time time_prev_ros_time = ros::Time::now();
//     ros::WallTime time_test_ros_walltime = ros::WallTime::now();


//     std::chrono::_V2::steady_clock::time_point time_now_std_time = std::chrono::steady_clock::now();
//     std::chrono::_V2::steady_clock::time_point time_prev_step = std::chrono::steady_clock::now();
//     std::chrono::duration<double> ex_time_std_time;
//     std::chrono::duration<double> loop_time_std_time;
    
//     double sleep_time_std_time;
//     bool track_calling = false;

//     // unsigned int id_write, flag = 0;
//     unsigned int kvaDb_flags = 0;
//     unsigned int dlc = 8;

//     int alive_count = 0;

//     float speed_ego = 0;
//     float east_velocity = 0;
//     float north_velocity = 0;
//     struct time_struct{double millisecond, second, minute, hour, day, month, year;};
//     struct timeb international_time;
//     struct tm *local_time;
//     time_struct time_now;
//     vector<tuple<char*, vector<char*>>> msg_list;
    
//     perception_ros_msg::object_array_msg *track_stored = new perception_ros_msg::object_array_msg;
//     perception_ros_msg::object_array_msg track;

//     TRACK_CAN_WRITER_NO_GRID();

//     void CALLBACK_INSPVA(const novatel_gps_msgs::InspvaConstPtr& data);
//     void CALLBACK_TRACK      (const perception_ros_msg::object_array_msg& data);
//     short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
//     void GET_DATETIME();
//     float GET_FUNC_VAL(const struct lane& lane, const short& x);
//     canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
//     void LOOP();
    

//     struct track_info{
//       int track_order;
//       int track_id;
//       float mahal_dist;

//       bool operator<(const track_info &b)const{
//         return mahal_dist < b.mahal_dist;
//       };
//     };

//     struct find_struct{
//         bool isIn;
//         track_info track_data;
//     };

//     vector<track_info> can_msg_info;
//     vector<float> cov_diag_element = {1.0f, 2.5f};
//     find_struct isIn_func(vector<track_info> v, track_info track_data);

//     bool write_or_not[14] = {0};
//     KvaDbMessageHnd mh = 0;
//     KvaDbSignalHnd sh = 0;
//     std::vector<track_info> track_ordered_info;  // track_Multi_RS 순서 저장
//     float MAHAL_DIST(const perception_ros_msg::object_array_msg& track_data);
//     void CAN_MSG_PREPARE(int track_idx, int msg_num);
//     void WRITE_CAN_MSG(vector<double> temp_data, char* track_msg);

// };

// struct TRACK_CAN_WRITER_NO_GRID::find_struct TRACK_CAN_WRITER_NO_GRID::isIn_func(vector<track_info> v, track_info track_data){
//   find_struct ret;
//   int target = track_data.track_id;

//   for(int i=0; i!=v.size();i++){
//     // if(v.at(i).track_id == target){
//     if(v.at(i).track_id == target && target!=-1){
//       ret.isIn = true;
//       ret.track_data = track_data;
//       write_or_not[i] = true;
//       return ret;
//     }
//   }

//   ret.isIn = false;
//   ret.track_data = track_data;     
//   return ret;
// }

// TRACK_CAN_WRITER_NO_GRID::TRACK_CAN_WRITER_NO_GRID(){
//   track_info empty_info;
//   empty_info.track_order=-1;
//   empty_info.track_id=-1;
//   empty_info.mahal_dist=-1;
  
//   for(short i=0; i<TRACK_MAX_SIZE; i++){
//     can_msg_info.push_back(empty_info);
//   }
  
//   memset(write_or_not,false,sizeof(write_or_not));

//   // List of CAN messages that you want to send to a CAN bus /////////////////////////////////////////////////
//   msg_list.push_back(make_tuple((char*)"TimeInformation",  vector<char*> {(char*)"MilliSecond",\
//                                                                           (char*)"Second",\
//                                                                           (char*)"Minute",\
//                                                                           (char*)"Hour",\
//                                                                           (char*)"Day",\
//                                                                           (char*)"Month",\
//                                                                           (char*)"Year"}));

//   msg_list.push_back(make_tuple((char*)"FAULT_INPUT_MESSAGE",  vector<char*> {(char*)"FAULT_INPUT_V2V",\
//                                                                               (char*)"FAULT_INPUT_FRONT_VISION",\
//                                                                               (char*)"FAULT_INPUT_FRONT_RADAR",\
//                                                                               (char*)"FAULT_INPUT_FRONT_LIDAR"}));

//   msg_list.push_back(make_tuple((char*)"FAULT_DETECT_MESSAGE",  vector<char*> {(char*)"FAULT_DETECT_V2V",\
//                                                                                (char*)"FAULT_DETECT_FRONT_VISION",\
//                                                                                (char*)"FAULT_DETECT_FRONT_RADAR",\
//                                                                                (char*)"FAULT_DETECT_FRONT_LIDAR"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_0", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_0", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));  
                                                                    
//   msg_list.push_back(make_tuple((char*)"TRACK_1", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_1", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));  
                                                                    
//   msg_list.push_back(make_tuple((char*)"TRACK_2", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_2", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));     

//   msg_list.push_back(make_tuple((char*)"TRACK_3", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_3", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));     

//   msg_list.push_back(make_tuple((char*)"TRACK_4", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_4", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));       

//   msg_list.push_back(make_tuple((char*)"TRACK_5", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_5", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_6", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_6", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));    

//   msg_list.push_back(make_tuple((char*)"TRACK_7", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_7", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));   

//   msg_list.push_back(make_tuple((char*)"TRACK_8", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_8", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));     

//   msg_list.push_back(make_tuple((char*)"TRACK_9", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_9", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));    

//   msg_list.push_back(make_tuple((char*)"TRACK_10", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_10", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));  

//   msg_list.push_back(make_tuple((char*)"TRACK_11", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_11", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));      

//   msg_list.push_back(make_tuple((char*)"TRACK_12", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_12", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));    

//   msg_list.push_back(make_tuple((char*)"TRACK_13", vector<char*> {(char*)"TRACK_ID",\
//                                                                 (char*)"TRACK_STATUS",\
//                                                                 (char*)"TRACK_X",\
//                                                                 (char*)"TRACK_Y",\
//                                                                 (char*)"TRACK_VX",\
//                                                                 (char*)"TRACK_VY"}));

//   msg_list.push_back(make_tuple((char*)"TRACK_ADD_13", vector<char*> {(char*)"TRACK_ID",\
//                                                                     (char*)"TRACK_AX",\
//                                                                     (char*)"TRACK_AY",\
//                                                                     (char*)"TRACK_ORIENTATION",\
//                                                                     (char*)"TRACK_SIZE_X",\
//                                                                     (char*)"TRACK_SIZE_Y"}));                                                                                                                                         
// }


// void TRACK_CAN_WRITER_NO_GRID::CALLBACK_INSPVA(const novatel_gps_msgs::InspvaConstPtr& data){
//     double east_velocity = data->east_velocity;
//     double north_velocity = data->north_velocity;
//     speed_ego = std::sqrt(east_velocity * east_velocity + north_velocity * north_velocity);
// }

// void TRACK_CAN_WRITER_NO_GRID::CALLBACK_TRACK(const perception_ros_msg::object_array_msg& data) {
//     track_calling = true;

//     track_time = data.time;
//     track_stored->data.clear();
//     *track_stored = data;

//     track_calling = false;
// }

// short TRACK_CAN_WRITER_NO_GRID::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
//   short idx = -1;

//   for(short i=0; i!=msg_list->size(); i++){

//     if(strcmp(target_msg, get<0>(msg_list->at(i))) == 0){
//       idx = i;
//       break;
//     }
//   }

//   if(idx != -1){
//     return idx;

//   }else{
//     cout<<"No matched message! : "<<target_msg<<endl;
//     return -1;
//   }
// }



// void TRACK_CAN_WRITER_NO_GRID::GET_DATETIME(){
//   ftime(&international_time);
//   local_time = localtime(&international_time.time);

//   time_now.millisecond = international_time.millitm;
//   time_now.second = local_time->tm_sec;
//   time_now.minute = local_time->tm_min;
//   time_now.hour = local_time->tm_hour;
//   time_now.day = local_time->tm_mday;
//   time_now.month = local_time->tm_mon + 1;
//   time_now.year = local_time->tm_year + 1900;\
// } 


// canStatus TRACK_CAN_WRITER_NO_GRID::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
//   canInitializeLibrary();

//   int open_flag;

//   if(init_access_flag == true){
//       open_flag = canOPEN_REQUIRE_INIT_ACCESS;

//     }else{
//       open_flag = canOPEN_NO_INIT_ACCESS;
//     }


//   cout<<"Opening the channel "<<channel_num<<"..."<<endl;
//   hCAN = canOpenChannel(channel_num, open_flag);
  
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

//   return canOK;
// }

// //////////// hyerim edit////start/////////////////////////////////////////////////////////////////////////////////////////////////////
// float TRACK_CAN_WRITER_NO_GRID::MAHAL_DIST(const perception_ros_msg::object_array_msg& track_data){
//   float mah_dist_output = 0;

//   for (int i = 0; i < track_data.data.size(); i++)
//     if (fabs(track_data.data[i].x) > 10.0f || fabs(track_data.data[i].y) > 5.0f){
//       mah_dist_output = 999;
//       }
//     else if (fabs(track_data.data[i].x) < 0.4f || fabs(track_data.data[i].y) < 0.4f){
//       mah_dist_output = 0.4;
//       }
//     else{
//       mah_dist_output = sqrt(pow(track_data.data[i].x,2.0f) * cov_diag_element[0] + pow(track_data.data[i].y, 2.0f) * cov_diag_element[1]);
//       }
  
//   return mah_dist_output;
// }


// void TRACK_CAN_WRITER_NO_GRID::CAN_MSG_PREPARE(int track_order, int msg_num) {
//     vector<double> temp_data;
//     vector<double> temp_data_add;

//     temp_data.clear();
//     temp_data_add.clear();

//     if (track_order == -1) {
//         // 빈 슬롯 처리
//         temp_data = {0, 0, 0, 0, 0, 0};
//         temp_data_add = {0, 0, 0, 0, 0, 0};
//     } else {
//         // track_order 순서대로 데이터를 채운다 (track_order = /track_Multi_RS 인덱스 그대로 사용)
//         temp_data = {
//             (double)track.data[track_order].id,
//             (double)track.data[track_order].status,
//             track.data[track_order].x,
//             track.data[track_order].y,
//             track.data[track_order].vx,
//             track.data[track_order].vy
//         };

//         temp_data_add = {
//             (double)track.data[track_order].id,
//             track.data[track_order].ax,
//             track.data[track_order].ay,
//             track.data[track_order].orientation,
//             track.data[track_order].size_x,
//             track.data[track_order].size_y
//         };
//     }


//     string str = "TRACK_" + to_string(msg_num);
//     char* msg_name = &str[0];
//     WRITE_CAN_MSG(temp_data, msg_name);

//     string strr = "TRACK_ADD_" + to_string(msg_num);
//     char* msg_add_name = &strr[0];
//     WRITE_CAN_MSG(temp_data_add, msg_add_name);

// }

// void TRACK_CAN_WRITER_NO_GRID::WRITE_CAN_MSG(vector<double> temp_data, char* track_msg){
//     unsigned char can_data[dlc];
//     unsigned short msg_idx;
//     char* target_msg;
//     unsigned int id_write, flag = 0;

//     target_msg = (char*)track_msg;
//     msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
//     kvaDbGetMsgByName(dh, target_msg, &mh);
//     kvaDbGetMsgId(mh, &id_write, &flag);

//     for(int i=0; i!=get<1>(msg_list[msg_idx]).size(); i++){
//       kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
//       kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[i]);
//     }
    
//     canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
//     memset(can_data, 0, sizeof(can_data));
// }


// void TRACK_CAN_WRITER_NO_GRID::LOOP(){
//   ros::Rate rate(freq_for_channel_2);

//   while(ros::ok()){
     
//     time1 = ros::Time::now();
//     std::chrono::_V2::steady_clock::time_point time_start_std_time = std::chrono::steady_clock::now();

    
//     if(track_calling == false){
//       track.data.clear();
//       track = *track_stored; // track을 쓰면 된다.
//     }
    
//     // 센서퓨전은 70ms 마다 일어나지만 AutoBox로 10ms 마다 트랙정보를 전달해야 하므로 CV모델로 사이값을 보간하여 송신한다
//     if(track.time.toSec() < 5){
//       dt = ros::Duration(0);

//     }else{
//       dt = time1 - track.time;
//     }

//     perception_ros_msg::object_msg ref;

//     for(short i=0; i!=track.data.size(); i++){ 
//       ref = track.data[i];
//       ref.x += (ref.vx - speed_ego) * dt.toSec();
//       ref.y += ref.vy * dt.toSec();
//     }

//     time_now_std_time = std::chrono::steady_clock::now();
//     ex_time_std_time = (time_now_std_time - time_start_std_time);
//     time_start_std_time = std::chrono::steady_clock::now();
                         
                           
//     unsigned char can_data_for_fault[1];
//     unsigned char can_data[dlc];
//     char* target_msg;
//     unsigned short msg_idx;
//     KvaDbMessageHnd mh = 0;
//     KvaDbSignalHnd sh = 0;
//     unsigned int id_write, flag = 0;

    
//     time_now_std_time = std::chrono::steady_clock::now();
//     ex_time_std_time = (time_now_std_time - time_start_std_time);
//     time_start_std_time = std::chrono::steady_clock::now();

//     ////////////////////////////////////////// 혜림 코드 시작 ///////////////////////////////////////////////////////////////////////////////////////
//     /* 1. Select max 14 track using mahalanobis distance  */
//     vector<track_info> mahal_output;  // size : # of track at this timestep
//     vector<track_info> now_track;     // size : max 14
//     track_info empty_now_track;

//     empty_now_track.track_order = -1;
//     empty_now_track.track_id = -1;
//     empty_now_track.mahal_dist = -1;   

//     for(int i=0; i!=track.data.size(); i++){
//       track_info t;
//       t.track_order = i;
//       t.track_id = track.data[i].id;
//       t.mahal_dist = (float)MAHAL_DIST(track);
//       mahal_output.push_back(t);
//     } 

//     // sort current track data w.r.t. mahal dist output (ascending order)
//     sort(mahal_output.begin(), mahal_output.end());

//     // select max 14 tracks from sorted data
//     if(mahal_output.size() >= TRACK_MAX_SIZE){
//       for(int i=0; i<TRACK_MAX_SIZE; i++){
//           now_track.push_back(mahal_output.at(i));
//       } 
//     }

//     else{
//       for(int i=0; i!=mahal_output.size(); i++){
//         now_track.push_back(mahal_output.at(i));
//       }

//       for(int i=mahal_output.size(); i<TRACK_MAX_SIZE; i++){
//         now_track.push_back(empty_now_track);
//       }
//     }

//     /* 2. CAN msg number assignment  */
//     // write_or_not : check the continuity of track between previous and current timestep
//     //                if true, that track existed at previous timestep
    
//     memset(write_or_not,false,sizeof(write_or_not));


        
//     find_struct output[TRACK_MAX_SIZE];    
//     for(int i=0; i!=now_track.size(); i++){
//       output[i] = isIn_func(can_msg_info, now_track.at(i));
//     }
  
//     // queue q : compare current track data to previous timestep
//     //           value of queue will be assigned to newly made track
//     queue<int> q;
//     for(int i=0;i!=sizeof(write_or_not);i++){
//       if(write_or_not[i] == false){
//         q.push(i);
//         can_msg_info.at(i) = empty_now_track;
//       }
//     }
   
//     // store track data that detects newly at current timestep
//     for(int i=0; i!=(sizeof(output)/sizeof(find_struct)); i++){
//       if(output[i].isIn == false){
//         int idx = q.front();
//         q.pop();
//         can_msg_info.at(idx) = output[i].track_data;
//         }
//     }
    
//     vector<double> data_data(6);
//     vector<double> data_data_add(6);

 

//     /* 3. CAN msg write    */ 

//     for(int i=0; i!=can_msg_info.size(); i++){
//       int msg_num = i;
//       int track_order = -1;

//       if(can_msg_info[i].track_id == -1){
//         data_data = {0,0,0,0,0,0};
//         data_data_add = {0,0,0,0,0,0};

//         string str = "TRACK_" + to_string(i);
//         char *msg_name = &str[0];
//         WRITE_CAN_MSG(data_data, msg_name);

//         string strr = "TRACK_ADD_" + to_string(i);
//         char *msg_add_name = &strr[0];   
//         WRITE_CAN_MSG(data_data_add, msg_add_name);

//       }
//       else{
//         // find track_order from track_id
//         for(int j=0; j!=track.data.size(); j++){
//           if(track.data[j].id == can_msg_info[i].track_id){
//             track_order = j;
//             CAN_MSG_PREPARE(track_order, msg_num);
//           }
//         }
//       }
//     }

//     /* Check Result */
//     for(int i=0; i!=can_msg_info.size(); i++){
//         cout << can_msg_info[i].track_id << " ";
//     }
//     cout << endl;


//     time_now_std_time = std::chrono::steady_clock::now();
//     ex_time_std_time = (time_now_std_time - time_start_std_time);
//     // cout<<"ex time(sort):   "<<ex_time_std_time.count()*1000<<" ms"<<endl;
//     time_start_std_time = std::chrono::steady_clock::now();

    
//     time_now_std_time = std::chrono::steady_clock::now();
//     std::chrono::duration<double> ex_time_total = time_now_std_time - time_prev_step;
//     // cout<<"ex time(total): "<<ex_time_total.count()*1000<<" ms"<<endl;

//     sleep_time_std_time = (double)0.01 - ex_time_total.count();
//     // cout<<"sleep time: "<<sleep_time_std_time*1000<<" ms"<<endl;


//     if(sleep_time_std_time > 0){
//       ros::Duration(sleep_time_std_time).sleep();
//     }
    
//     // rate.sleep();

//     time_now_std_time = std::chrono::steady_clock::now();
//     loop_time_std_time = time_now_std_time - time_prev_step;
//     // cout<<"loop time:  "<<loop_time_std_time.count()*1000<<" ms"<<endl;
//     time_prev_step = time_now_std_time;

//     time_now_ros_time = ros::Time::now();
//     ros::Duration loop_time_ros_time = time_now_ros_time - time_prev_ros_time;
//     // cout<<"loop time(ros):  "<<loop_time_ros_time.toSec()*1000<<" ms"<<endl;
//     time_prev_ros_time = time_now_ros_time;

//     // cout<<"-------------------------------------------------------"<<endl;
//   }
// }



// int main(int argc, char **argv){
//   cout<<"Initializing ..."<<endl;
//   ros::init(argc, argv, "Track_CAN_Writer_NO_GRID");
//   ros::NodeHandle node("~");
//   ros::AsyncSpinner spinner(0);
//   spinner.start();

//   string relative_path = ros::package::getPath("can");
//   char filename[100];

//   strcpy(filename, (relative_path + "/dbc/CANdb_IONIQev_PCAN2.dbc").c_str());
  
//   // int channel_num = 0; // for test
//   int channel_num = 1;
//   bool init_access_flag = false; // Init access: no (= CAN handle will be used in multithread)
//   ///////////////////////////////////////////////////////////////////////////////////////

//   TRACK_CAN_WRITER_NO_GRID TCW;

//   can_status = TCW.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);


//   ros::Subscriber sub5 = node.subscribe("/track_Multi_RS_filtered",               1, &TRACK_CAN_WRITER_NO_GRID::CALLBACK_TRACK,       &TCW);
//   ros::Subscriber sub6 = node.subscribe("/sensors/gps/inspva",           1, &TRACK_CAN_WRITER_NO_GRID::CALLBACK_INSPVA,       &TCW);


//   TCW.LOOP();
  
//   ros::waitForShutdown();   
//   canBusOff(hCAN);
//   canClose(hCAN);
// }