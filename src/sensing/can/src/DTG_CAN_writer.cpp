#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>
#include <katech_custom_msgs/v_can_msg.h>

using namespace std;

class DTG_CAN_WRITER {
  public:
    unsigned int kvaDb_flags = 0;

    // ESP12 signals
    double lat_accel = 0.0;
    double long_accel = 0.0;
    double cyl_pres = 0.0;
    double yaw_rate = 0.0;

    // SAS11 signals
    double sas_angle = 0.0;
    double sas_speed = 0.0;

    // CGW_PC4 signals
    double motor_rpm = 0.0;
    double vehicle_speed = 0.0;

    // WHL_SPD11 signals
    double whl_spd_fl = 0.0;
    double whl_spd_fr = 0.0;
    double whl_spd_rl = 0.0;
    double whl_spd_rr = 0.0;

    bool new_data = false;

    vector<tuple<char*, vector<char*>>> msg_list;

    DTG_CAN_WRITER();

    void v_can_callback(const katech_custom_msgs::v_can_msg::ConstPtr& msg) {
        // ESP12
        lat_accel = msg->lat_acceleration;
        long_accel = msg->long_acceleration;
        cyl_pres = 0.0;  // not available from v_can
        yaw_rate = msg->yaw_rate;

        // SAS11
        sas_angle = msg->steering_angle;
        sas_speed = 0.0;  // not available from v_can

        // CGW_PC4
        motor_rpm = msg->motor_rpm;
        double avg_speed_ms = (msg->wheel_speed_fl + msg->wheel_speed_fr +
                               msg->wheel_speed_rl + msg->wheel_speed_rr) / 4.0;
        vehicle_speed = avg_speed_ms * 3.6;  // m/s -> km/h

        // WHL_SPD11 (m/s -> km/h)
        whl_spd_fl = msg->wheel_speed_fl * 3.6;
        whl_spd_fr = msg->wheel_speed_fr * 3.6;
        whl_spd_rl = msg->wheel_speed_rl * 3.6;
        whl_spd_rr = msg->wheel_speed_rr * 3.6;

        new_data = true;
    }

    short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
    void SEND_MSG();
    canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char* filename, bool init_access_flag);
    void LOOP();
};

DTG_CAN_WRITER::DTG_CAN_WRITER() {
    msg_list.push_back(make_tuple((char*)"ESP12", vector<char*>{(char*)"LAT_ACCEL",
                                                                 (char*)"LONG_ACCEL",
                                                                 (char*)"CYL_PRES",
                                                                 (char*)"YAW_RATE"}));

    msg_list.push_back(make_tuple((char*)"SAS11", vector<char*>{(char*)"SAS_Angle",
                                                                 (char*)"SAS_Speed"}));

    msg_list.push_back(make_tuple((char*)"CGW_PC4", vector<char*>{(char*)"N",
                                                                    (char*)"VS"}));

    msg_list.push_back(make_tuple((char*)"WHL_SPD11", vector<char*>{(char*)"WHL_SPD_RR",
                                                                      (char*)"WHL_SPD_RL",
                                                                      (char*)"WHL_SPD_FR",
                                                                      (char*)"WHL_SPD_FL"}));
}

short DTG_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list) {
    short idx = -1;
    for (short i = 0; i != msg_list->size(); i++) {
        if (strcmp(target_msg, get<0>(msg_list->at(i))) == 0) {
            idx = i;
            break;
        }
    }
    if (idx != -1) {
        return idx;
    } else {
        cout << "No matched message! : " << target_msg << endl;
        return -1;
    }
}

void DTG_CAN_WRITER::SEND_MSG() {
    unsigned char can_data[8];
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    unsigned int id_write, flag = 0;
    unsigned short msg_idx;
    char* target_msg;
    vector<double> temp_data;

    for (short i = 0; i < 4; i++) {
        memset(can_data, 0, sizeof(can_data));

        switch (i) {
            case (0):
                target_msg = (char*)"ESP12";
                temp_data = {lat_accel, long_accel, cyl_pres, yaw_rate};
                break;
            case (1):
                target_msg = (char*)"SAS11";
                temp_data = {sas_angle, sas_speed};
                break;
            case (2):
                target_msg = (char*)"CGW_PC4";
                temp_data = {motor_rpm, vehicle_speed};
                break;
            case (3):
                target_msg = (char*)"WHL_SPD11";
                temp_data = {whl_spd_rr, whl_spd_rl, whl_spd_fr, whl_spd_fl};
                break;
        }

        msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
        kvaDbGetMsgByName(dh, target_msg, &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);

        for (int j = 0; j != get<1>(msg_list[msg_idx]).size(); j++) {
            kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
            kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[j]);
        }

        canWrite(hCAN, id_write, &can_data, 8, canMSG_STD);
    }
}

canStatus DTG_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char* filename, bool init_access_flag) {
    canInitializeLibrary();

    int open_flag = init_access_flag ? canOPEN_REQUIRE_INIT_ACCESS : canOPEN_NO_INIT_ACCESS;

    cout << "Opening the channel " << channel_num << "..." << endl;
    hCAN = canOpenChannel(channel_num, open_flag);

    if (hCAN == canOK) {
        cout << "The CAN channel " << channel_num << " has been opened successfully..." << endl;
    }

    can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
    can_status = canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
    can_status = canBusOn(hCAN);

    if (can_status == canOK) {
        cout << "The CAN bus is on..." << endl;
    }

    kvaDb_status = kvaDbOpen(&dh);
    kvaDb_status = kvaDbReadFile(dh, filename);
    kvaDb_status = kvaDbGetFlags(dh, &kvaDb_flags);

    if (kvaDb_status == kvaDbOK) {
        cout << "Database file has been loaded successfully..." << endl;
    }

    return canOK;
}

void DTG_CAN_WRITER::LOOP() {
    ros::Rate rate(50);  // 50Hz (20ms)

    while (ros::ok()) {
        if (new_data) {
            SEND_MSG();
            new_data = false;
        }
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    cout << "Initializing ..." << endl;
    ros::init(argc, argv, "DTG_CAN_writer");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    string relative_path = ros::package::getPath("can");
    char filename[100];
    strcpy(filename, (relative_path + "/dbc/2gen-2ch-C_IoniqEV_v2.dbc").c_str());

    int channel_num = 3;
    bool init_access_flag = false;

    DTG_CAN_WRITER dcw;
    can_status = dcw.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

    ros::Subscriber sub1 = node.subscribe("/sensors/v_can", 1,
                                          &DTG_CAN_WRITER::v_can_callback, &dcw);

    dcw.LOOP();

    canBusOff(hCAN);
    canClose(hCAN);

    return 0;
}
