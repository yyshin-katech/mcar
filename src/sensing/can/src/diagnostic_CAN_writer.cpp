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

#include <katech_diagnostic_msgs/cpt7_gps_diagnostic_msg.h>
#include <katech_diagnostic_msgs/k_adcu_diagnostic_msg.h>
#include <katech_diagnostic_msgs/lidar_diagnostic_msg.h>
#include <katech_diagnostic_msgs/radar_diagnostic_msg.h>
#include <katech_diagnostic_msgs/v2x_diagnostic_msg.h>
#include <katech_diagnostic_msgs/hmi_diagnostic_msg.h>
#include <katech_diagnostic_msgs/vcu_diagnostic_msg.h>
#include <katech_diagnostic_msgs/ipc_diagnostic_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

class DIAGNOSTIC_CAN_WRITER{
    public:
        DIAGNOSTIC_CAN_WRITER();
        ~DIAGNOSTIC_CAN_WRITER();

        canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);

    private:
        ros::NodeHandle nh;

        ros::Subscriber adcu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber radar_sub;
        ros::Subscriber v2x_sub;
        ros::Subscriber hmi_sub;
        ros::Subscriber vcu_sub;
        ros::Subscriber ipc_sub;

        ros::Timer timer_;

        vector<tuple<char*, vector<char*>>> msg_list;

        void timerCallback(const ros::TimerEvent&);

        void cpt7_callback(const katech_diagnostic_msgs::cpt7_gps_diagnostic_msg& msg);
        void lidar_callback(const katech_diagnostic_msgs::lidar_diagnostic_msg& msg);
        void v2x_callback(const katech_diagnostic_msgs::v2x_diagnostic_msg& msg);
        void hmi_callback(const katech_diagnostic_msgs::hmi_diagnostic_msg& msg);
        void radar_callback(const katech_diagnostic_msgs::radar_diagnostic_msg& msg);
        void ipc_callback(const katech_diagnostic_msgs::ipc_diagnostic_msg& msg);

        short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
        
};

DIAGNOSTIC_CAN_WRITER::DIAGNOSTIC_CAN_WRITER()
{
    // adcu_sub = nh.subscribe("/diagnostic/cpt7_gps", 1, &DIAGNOSTIC_CAN_WRITER::cpt7_callback, this);
    gps_sub = nh.subscribe("/diagnostic/cpt7_gps", 1, &DIAGNOSTIC_CAN_WRITER::cpt7_callback, this);
    lidar_sub = nh.subscribe("/diagnostic/lidar", 1, &DIAGNOSTIC_CAN_WRITER::lidar_callback, this);
    v2x_sub = nh.subscribe("/diagnostic/v2x", 1, &DIAGNOSTIC_CAN_WRITER::v2x_callback, this);
    hmi_sub = nh.subscribe("/diagnostic/hmi", 1, &DIAGNOSTIC_CAN_WRITER::hmi_callback, this);
    radar_sub = nh.subscribe("/diagnostic/radar", 1, &DIAGNOSTIC_CAN_WRITER::radar_callback, this);
    ipc_sub = nh.subscribe("/diagnostic/ipc", 1, &DIAGNOSTIC_CAN_WRITER::ipc_callback, this);

    timer_ = nh.createTimer(ros::Duration(1.0), &DIAGNOSTIC_CAN_WRITER::timerCallback, this);

    msg_list.push_back(make_tuple((char*)"DiagnosticGPS",  vector<char*> {(char*)"IMU_StatCode",\
                                                                        (char*)"GPS_StatCode",\
                                                                        (char*)"GPS_INS_AliveCnt",\
                                                                        (char*)"INS_StatCode", \
                                                                        (char*)"GPSRTK_StatCode",\
                                                                        (char*)"GPS_INS_SolutionStat"}));

    msg_list.push_back(make_tuple((char*)"DiagnosticLiDAR",  vector<char*> {(char*)"Left_Lidar_StatCode",\
                                                                        (char*)"Right_Lidar_StatCode",\
                                                                        (char*)"Center_Lidar_StatCode"}));
                                                                    
    msg_list.push_back(make_tuple((char*)"DiagnosticOBU",  vector<char*> {(char*)"OBU_AliveCnt",\
                                                                       (char*)"OBU_StatCode"}));  
                                                                       
    msg_list.push_back(make_tuple((char*)"DiagnosticHMI",  vector<char*> {(char*)"HMI_AliveCnt",\
                                                                       (char*)"HMI_StatCode"}));

    msg_list.push_back(make_tuple((char*)"DiagnosticRADAR",  vector<char*> {(char*)"Radar_AliveCnt"}));                                                                       
                                                                       
    msg_list.push_back(make_tuple((char*)"DiagnosticIPC",  vector<char*> {(char*)"IPC_SWC_StatCode",\
                                                                       (char*)"IPC_AliveCount"}));
}

DIAGNOSTIC_CAN_WRITER::~DIAGNOSTIC_CAN_WRITER()
{

}

void DIAGNOSTIC_CAN_WRITER::timerCallback(const ros::TimerEvent&)
{

}

void DIAGNOSTIC_CAN_WRITER::cpt7_callback(const katech_diagnostic_msgs::cpt7_gps_diagnostic_msg& msg)
{
    uint8_t can_data[8];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data;
    unsigned int id_write, flag = 0;
    int re_value = 0;

    target_msg = (char*)"DiagnosticGPS";
    temp_data = {msg.IMU_StatCode, msg.GPS_StatCode, msg.GPS_INS_AliveCnt, msg.INS_StatCode, msg.GPSRTK_StatCode, msg.GPS_INS_SolutionStat};

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

void DIAGNOSTIC_CAN_WRITER::lidar_callback(const katech_diagnostic_msgs::lidar_diagnostic_msg& msg)
{
    uint8_t can_data[3];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data;
    unsigned int id_write, flag = 0;
    int re_value = 0;

    target_msg = (char*)"DiagnosticLiDAR";
    temp_data = {msg.LIDAR_Left_StatCode, msg.LIDAR_Right_StatCode, msg.LIDAR_Center_StatCode};

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

void DIAGNOSTIC_CAN_WRITER::v2x_callback(const katech_diagnostic_msgs::v2x_diagnostic_msg& msg)
{
    uint8_t can_data[2];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data;
    unsigned int id_write, flag = 0;
    int re_value = 0;

    target_msg = (char*)"DiagnosticOBU";
    temp_data = {msg.V2X_AliveCount, msg.V2X_StatCode};

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

void DIAGNOSTIC_CAN_WRITER::hmi_callback(const katech_diagnostic_msgs::hmi_diagnostic_msg& msg)
{
    uint8_t can_data[2];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data;
    unsigned int id_write, flag = 0;
    int re_value = 0;

    target_msg = (char*)"DiagnosticHMI";
    temp_data = {msg.HMI_AliveCount, msg.HMI_StatCode};

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

void DIAGNOSTIC_CAN_WRITER::radar_callback(const katech_diagnostic_msgs::radar_diagnostic_msg& msg)
{
    uint8_t can_data[1];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data;
    unsigned int id_write, flag = 0;
    int re_value = 0;

    target_msg = (char*)"DiagnosticRADAR";
    temp_data = {msg.RADAR_AliveCount};

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

void DIAGNOSTIC_CAN_WRITER::ipc_callback(const katech_diagnostic_msgs::ipc_diagnostic_msg& msg)
{
    uint8_t can_data[2];
    char* target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data;
    unsigned int id_write, flag = 0;
    int re_value = 0;

    target_msg = (char*)"DiagnosticIPC";
    temp_data = {msg.IPC_SWC_StatCode, msg.IPC_AliveCount};

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

short DIAGNOSTIC_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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

canStatus DIAGNOSTIC_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
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
    ros::init(argc, argv, "Diagnostic_CAN_Writer");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    std::string relative_path = ros::package::getPath("can");
    char filename[100];

    strcpy(filename, (relative_path + "/dbc/CANdb_IONIQev_PCAN1.dbc").c_str());
    int channel_num = 0;
    bool init_access_flag = false;

    DIAGNOSTIC_CAN_WRITER diagnostic_can_writer;

    can_status = diagnostic_can_writer.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

    ros::waitForShutdown();   
    canBusOff(hCAN);
    canClose(hCAN);


}