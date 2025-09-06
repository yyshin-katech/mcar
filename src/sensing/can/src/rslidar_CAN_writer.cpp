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
        
        vector<double> track_data(6, 0.0);
        vector<double> track_add_data(6, 0.0);
        
        if (i < msg.data.size()) {
            auto& obj = msg.data[i];
            track_data = {obj.id, obj.status, obj.x, obj.y, obj.vx, obj.vy};
            track_add_data = {obj.id, obj.ax, obj.ay, obj.orientation, obj.size_x, obj.size_y};
        }
        
        for (int j = 0; j < 2; j++) {
            string target_msg = (j == 0) ? track_msg : track_add_msg;
            short msg_idx = FIND_MSG_IDX(target_msg);
            if (msg_idx == -1) continue;
            kvaDbGetMsgByName(dh, target_msg.c_str(), &mh);
            kvaDbGetMsgId(mh, &id_write, &flag);
            
            vector<double>& temp_data = (j == 0) ? track_data : track_add_data;
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
