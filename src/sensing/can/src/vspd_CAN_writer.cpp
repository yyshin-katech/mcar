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

class VSPD_CAN_WRITER {
  public:
    unsigned int kvaDb_flags = 0;

    double veh_speed_kmh = 0.0;
    uint8_t veh_direction = 0;
    bool new_data = false;

    VSPD_CAN_WRITER() {}

    // /sensors/v_can 콜백: 휠 스피드 평균 → km/h, 기어 R → reverse
    void v_can_callback(const katech_custom_msgs::v_can_msg::ConstPtr& msg) {
        double avg_speed_ms = (msg->wheel_speed_fl + msg->wheel_speed_fr +
                               msg->wheel_speed_rl + msg->wheel_speed_rr) / 4.0;
        veh_speed_kmh = avg_speed_ms * 3.6;

        // gear_status: 2 = R → reverse(1), others → forward(0)
        veh_direction = (msg->gear_status == 2) ? 1 : 0;

        new_data = true;
    }

    void SEND_MSG() {
        unsigned char can_data[8];
        memset(can_data, 0, sizeof(can_data));

        KvaDbMessageHnd mh = 0;
        KvaDbSignalHnd sh = 0;
        unsigned int id_write, flag = 0;

        kvaDbGetMsgByName(dh, (char*)"MGI_vSpd_Gateway", &mh);
        kvaDbGetMsgId(mh, &id_write, &flag);

        kvaDbGetSignalByName(mh, (char*)"mgi_veh_speed", &sh);
        kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), veh_speed_kmh);

        kvaDbGetSignalByName(mh, (char*)"mgi_veh_direction", &sh);
        kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), (double)veh_direction);

        canWrite(hCAN, id_write, &can_data, 2, canMSG_STD);
    }

    canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char* filename, bool init_access_flag) {
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

    void LOOP() {
        ros::Rate rate(100);  // 100Hz (DBC cycle 10ms)

        while (ros::ok()) {
            if (new_data) {
                SEND_MSG();
                new_data = false;
            }
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    cout << "Initializing ..." << endl;
    ros::init(argc, argv, "vspd_CAN_writer");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    string relative_path = ros::package::getPath("can");
    char filename[100];
    strcpy(filename, (relative_path + "/dbc/CANdb_IONIQ5_AD_CAN_v6.dbc").c_str());

    int channel_num = 0;
    bool init_access_flag = false;

    VSPD_CAN_WRITER vcw;
    can_status = vcw.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

    ros::Subscriber sub1 = node.subscribe("/sensors/v_can", 1,
                                          &VSPD_CAN_WRITER::v_can_callback, &vcw);

    vcw.LOOP();

    canBusOff(hCAN);
    canClose(hCAN);

    return 0;
}
