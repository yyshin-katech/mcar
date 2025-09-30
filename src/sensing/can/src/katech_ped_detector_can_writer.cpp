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
#include <katech_custom_msgs/ped_crosswalk_check_array_msg.h>
#include <katech_custom_msgs/ped_crosswalk_check_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

class PED_DETECTOR_CAN_WRITTER{
    public:
        PED_DETECTOR_CAN_WRITTER();
        ~PED_DETECTOR_CAN_WRITTER();

        canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);

    private:
        ros::NodeHandle nh;

        ros::Subscriber sub;

        ros::Timer timer_;

        unsigned int kvaDb_flags = 0;
        unsigned int dlc = 8;

        vector<tuple<char*, vector<char*>>> msg_list;

        void timerCallback(const ros::TimerEvent&);
        void PED_DETECTOR_CALLBACK(const katech_custom_msgs::ped_crosswalk_check_array_msg& msg);
        short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
        void LOOP();
};

PED_DETECTOR_CAN_WRITTER::PED_DETECTOR_CAN_WRITTER()
{
    sub = nh.subscribe("/katech_msg/crosswalk_detection", 1, &PED_DETECTOR_CAN_WRITTER::PED_DETECTOR_CALLBACK, this);

    msg_list.push_back(make_tuple((char*)"Pedestrian_Stat",  vector<char*> {(char*)"rel_pos_y_1",\
                                                                    (char*)"rel_pos_x_1",\
                                                                    (char*)"on_crosswalk_1",\
                                                                    (char*)"obj_type_1", \
                                                                    (char*)"ped_obj_id_1",\
                                                                    (char*)"rel_pos_y",\
                                                                    (char*)"rel_pos_x",\
                                                                    (char*)"on_crosswalk",\
                                                                    (char*)"obj_type",\
                                                                    (char*)"ped_obj_id"}));

    msg_list.push_back(make_tuple((char*)"Pedestrian_Stat_1",  vector<char*> {(char*)"rel_pos_y_3",\
                                                                    (char*)"rel_pos_x_3",\
                                                                    (char*)"on_crosswalk_3",\
                                                                    (char*)"obj_type_3", \
                                                                    (char*)"ped_obj_id_3",\
                                                                    (char*)"rel_pos_y_2",\
                                                                    (char*)"rel_pos_x_2",\
                                                                    (char*)"on_crosswalk_2",\
                                                                    (char*)"obj_type_2",\
                                                                    (char*)"ped_obj_id_2"}));
}

PED_DETECTOR_CAN_WRITTER::~PED_DETECTOR_CAN_WRITTER()
{

}

void PED_DETECTOR_CAN_WRITTER::timerCallback(const ros::TimerEvent&)
{

}

void PED_DETECTOR_CAN_WRITTER::PED_DETECTOR_CALLBACK(const katech_custom_msgs::ped_crosswalk_check_array_msg& msg)
{
    uint8_t can_data[8];
    char* target_msg_1;
    char* target_msg_2;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    vector<double> temp_data_1, temp_data_2;
    unsigned int id_write, flag = 0;
    int re_value = 0;

    if(msg.data.empty())
    {
        target_msg_1 = (char*)"Pedestrian_Stat";
        temp_data_1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
        msg_idx = FIND_MSG_IDX(target_msg_1, &msg_list);
        kvaDbGetMsgByName(dh, target_msg_1, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);  

        for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_1[j]);
        }
        re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
        memset(can_data, 0, sizeof(can_data));

        target_msg_2 = (char*)"Pedestrian_Stat_1";
        temp_data_2 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        msg_idx = FIND_MSG_IDX(target_msg_2, &msg_list);
        kvaDbGetMsgByName(dh, target_msg_2, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);  

        for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_2[j]);
        }
        re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
        memset(can_data, 0, sizeof(can_data));
    }

    if(msg.data.size() == 1)
    {
        target_msg_1 = (char*)"Pedestrian_Stat";
        temp_data_1 = {msg.data[0].rel_pos_y, msg.data[0].rel_pos_x, msg.data[0].on_crosswalk, msg.data[0].status, msg.data[0].id, 
                     0, 0, 0, 0, 0};

        msg_idx = FIND_MSG_IDX(target_msg_1, &msg_list);
        kvaDbGetMsgByName(dh, target_msg_1, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);  

        for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_1[j]);
        }
        re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
        memset(can_data, 0, sizeof(can_data));

    }
    else if(msg.data.size() == 2)
    {
        target_msg_1 = (char*)"Pedestrian_Stat";
        temp_data_1 = {msg.data[0].rel_pos_y, msg.data[0].rel_pos_x, msg.data[0].on_crosswalk, msg.data[0].status, msg.data[0].id, 
                     msg.data[1].rel_pos_y, msg.data[1].rel_pos_x, msg.data[1].on_crosswalk, msg.data[1].status, msg.data[1].id};

        msg_idx = FIND_MSG_IDX(target_msg_1, &msg_list);
        kvaDbGetMsgByName(dh, target_msg_1, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);  

        for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_1[j]);
        }
        re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
        memset(can_data, 0, sizeof(can_data));
    }
    else if(msg.data.size() == 3)
    {
        target_msg_1 = (char*)"Pedestrian_Stat";
        temp_data_1 = {msg.data[0].rel_pos_y, msg.data[0].rel_pos_x, msg.data[0].on_crosswalk, msg.data[0].status, msg.data[0].id, 
                     msg.data[1].rel_pos_y, msg.data[1].rel_pos_x, msg.data[1].on_crosswalk, msg.data[1].status, msg.data[1].id};

        msg_idx = FIND_MSG_IDX(target_msg_1, &msg_list);
        kvaDbGetMsgByName(dh, target_msg_1, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);  

        for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_1[j]);
        }
        re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
        memset(can_data, 0, sizeof(can_data));

        target_msg_2 = (char*)"Pedestrian_Stat_1";
        temp_data_2 = {msg.data[2].rel_pos_y, msg.data[2].rel_pos_x, msg.data[2].on_crosswalk, msg.data[2].status, msg.data[2].id, 
                     0, 0, 0, 0, 0};

        msg_idx = FIND_MSG_IDX(target_msg_2, &msg_list);
        kvaDbGetMsgByName(dh, target_msg_2, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);  

        for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_2[j]);
        }
        re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
        memset(can_data, 0, sizeof(can_data));
    }
    else if(msg.data.size() == 4)
    {
        target_msg_1 = (char*)"Pedestrian_Stat";
        temp_data_1 = {msg.data[0].rel_pos_y, msg.data[0].rel_pos_x, msg.data[0].on_crosswalk, msg.data[0].status, msg.data[0].id, 
                     msg.data[1].rel_pos_y, msg.data[1].rel_pos_x, msg.data[1].on_crosswalk, msg.data[1].status, msg.data[1].id};
    
        msg_idx = FIND_MSG_IDX(target_msg_1, &msg_list);
        kvaDbGetMsgByName(dh, target_msg_1, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);  

        for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_1[j]);
        }
        re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
        memset(can_data, 0, sizeof(can_data));

        target_msg_2 = (char*)"Pedestrian_Stat_1";
        temp_data_2 = {msg.data[2].rel_pos_y, msg.data[2].rel_pos_x, msg.data[2].on_crosswalk, msg.data[2].status, msg.data[2].id, 
                     msg.data[3].rel_pos_y, msg.data[3].rel_pos_x, msg.data[3].on_crosswalk, msg.data[3].status, msg.data[3].id};

        msg_idx = FIND_MSG_IDX(target_msg_2, &msg_list);
        kvaDbGetMsgByName(dh, target_msg_2, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);  

        for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data_2[j]);
        }
        re_value = canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
        memset(can_data, 0, sizeof(can_data));
    }

}

short PED_DETECTOR_CAN_WRITTER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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

canStatus PED_DETECTOR_CAN_WRITTER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
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
    ros::init(argc, argv, "ped_detector_can_writer");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    std::string relative_path = ros::package::getPath("can");
    char filename[100];

    strcpy(filename, (relative_path + "/dbc/CANdb_IONIQev_PCAN1.dbc").c_str());
    int channel_num = 0;
    bool init_access_flag = false;

    PED_DETECTOR_CAN_WRITTER ped_detector_can_writer;

    can_status = ped_detector_can_writer.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

    ros::waitForShutdown();   
    canBusOff(hCAN);
    canClose(hCAN);
}