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

#include <std_msgs/Header.h>
#include <diagnostic_msgs/cpt7_gps_diagnostic_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

using namespace std;

class GPS_DIAGNOSTIC_CAN_WRITER{
    public:
        GPS_DIAGNOSTIC_CAN_WRITER();
        ~GPS_DIAGNOSTIC_CAN_WRITER();

    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;

        uint8_t kvaDb_flags = 0;
        uint8_t dls = 8;

        vector<tuple<char*, vector<char*>>> msg_list;

        void diagnostic_gps_callback(const diagnostic_msgs::cpt7_gps_diagnostic_msg::ConstPtr& msg); 
        short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
        canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
}

GPS_DIAGNOSTIC_CAN_WRITER::GPS_DIAGNOSTIC_CAN_WRITER()
{
    msg_list.push_back(make_tuple((char*)"DiagnosticGPS",  vector<char*> {(char*)"IMU_StatCode", \
                                                                          (char*)"GPS_StatCode", \
                                                                          (char*)"GPS_INS_AliveCnt", \
                                                                          (char*)"INS_StatCode", \
                                                                          (char*)"GPSRTK_StatCode", \
                                                                          (char*)"GPS_INS_SolutionStat"}));
}

GPS_DIAGNOSTIC_CAN_WRITER::~GPS_DIAGNOSTIC_CAN_WRITER()
{

}

void GPS_DIAGNOSTIC_CAN_WRITER::diagnostic_gps_callback(const diagnostic_msgs::cpt7_gps_diagnostic_msg::ConstPtr& msg)
{
    unsigned char can_data[dlc];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data;
    unsigned int id_write, flag = 0;

    target_msg = (char*)"DiagnosticGPS";
    temp_data = {msg.IMU_StatCode, msg.GPS_StatCode, msg.GPS_INS_AliveCnt, msg.GPSRTK_StatCode, GPS_INS_SolutionStat};

    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
        kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
        kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[j]);
    }

    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));
}

short GPS_DIAGNOSTIC_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list)
{
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

canStatus GPS_DIAGNOSTIC_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag)
{
    canInitializeLibrary();

    int open_flag;

    if(init_access_flag == true){
        open_flag = canOPEN_REQUIRE_INIT_ACCESS;

    }else{
        open_flag = canOPEN_NO_INIT_ACCESS;
    }

    // hCAN = canOpenChannel(channel_num, open_flag);

    // while(hCAN!=canOK){
    cout<<"Opening the channel "<<channel_num<<"..."<<endl;
    hCAN = canOpenChannel(channel_num, open_flag);
    // }

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

int main(int argc, char **argv)
{
    cout<<"Initializing ..."<<endl;
    ros::init(argc, argv, "gps_diagnostic_can_writer_node");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    string relative_path = ros::package::getPath("can");
    char filename[100];

    strcpy(filename, (relative_path + "/dbc/CANdb_IONIQev_PCAN1.dbc").c_str());
    int channel_num = 0;
    bool init_access_flag = false;

    GPS_DIAGNOSTIC_CAN_WRITER GPS_DCAN_W;

    can_status = GPS_DCAN_W.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

    ros::waitForShutdown();   
    canBusOff(hCAN);
    canClose(hCAN);
    return 0;
}