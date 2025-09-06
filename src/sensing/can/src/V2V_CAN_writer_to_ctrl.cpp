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
#include <mmc_msgs/V2V.h>
#include <sensor_msgs/NavSatFix.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <kvaDbLib.h>
#include <canlib.h>

using namespace std;

canHandle hCAN;
canStatus can_status;
KvaDbStatus kvaDb_status;
KvaDbHnd dh = 0;

class Tucson_CAN_writer_to_ctrl{
  public:
  ros::Subscriber sub1, sub2;

  unsigned short freq_for_channel_2 = 100; // Hz
  // unsigned int id_write, flag = 0;
  unsigned int kvaDb_flags = 0;
  unsigned int dlc = 8;

  unsigned int vehicle_status = 0;
  unsigned int turn_sig_left = 0;
  unsigned int turn_sig_right = 0;
  double vel_x_rel = 0;
  double vel_y_rel = 0;
  double pos_x_rel = 0;
  double pos_y_rel = 0;
  double SWA_deg = 0;

  vector<tuple<char *, vector<char *>>> msg_list;

  Tucson_CAN_writer_to_ctrl();

  void CALLBACK_V2V(const mmc_msgs::V2V &data);
  void CALLBACK_TRANSFORMED_V2V(const mmc_msgs::object_array_msg &data);
  short FIND_MSG_IDX(char *target_msg, vector<tuple<char *, vector<char *>>> *msg_list);
  canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);

  void LOOP();
};

Tucson_CAN_writer_to_ctrl::Tucson_CAN_writer_to_ctrl(){
  // List of CAN messages that you want to send to a CAN bus /////////////////////////////////////////////////
  msg_list.push_back(make_tuple((char *)"V2V_Info", vector<char *>{(char *)"VehicleStatus",\
                                                                   (char *)"TurnSigRight",\
                                                                   (char *)"TurnSigLeft",\
                                                                   (char *)"SteerWheelAng_deg",\
                                                                   (char *)"RelVelY_mps",\
                                                                   (char *)"RelVelX_mps",\
                                                                   (char *)"RelPosY",\
                                                                   (char *)"RelPosX"}));

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
}


void Tucson_CAN_writer_to_ctrl::CALLBACK_V2V(const mmc_msgs::V2V &data){
  SWA_deg = data.SteeringWheelAngle; // deg
  turn_sig_left = data.BrakeSystemStatus_Auxiliary_Brake_Status; // v2v 메시지 이름과는 다르게 실제로는 tucson의 좌측깜빡이다.
  turn_sig_right = data.BrakeSystemStatus_Brake_Boost_Applied; // v2v 메시지 이름과는 다르게 실제로는 tucson의 우측깜빡이다.
  vehicle_status = data.Vehicle_Event_Flag_eventDisabledVehicle; // 고장상황
}


void Tucson_CAN_writer_to_ctrl::CALLBACK_TRANSFORMED_V2V(const mmc_msgs::object_array_msg &data){
  pos_x_rel = data.data[0].x;
  pos_y_rel = data.data[0].y;
  vel_x_rel = data.data[0].vx;
  vel_y_rel = data.data[0].vy;
}


short Tucson_CAN_writer_to_ctrl::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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


canStatus Tucson_CAN_writer_to_ctrl::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
  canInitializeLibrary();

  int open_flag;

  if (init_access_flag == true){
    open_flag = canOPEN_REQUIRE_INIT_ACCESS;
    // open_flag = 0;

  }else{

    open_flag = canOPEN_NO_INIT_ACCESS;
  }

  hCAN = canOpenChannel(channel_num, open_flag);
  cout << hCAN << endl;

  if (hCAN == canOK){
    cout << "The CAN channel " << channel_num << " has been opened successfully..." << endl;
  }

  can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
  can_status = canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
  can_status = canBusOn(hCAN);

  if (can_status == canOK){
    cout << "The CAN bus is on..." << endl;
  }

  kvaDb_status = kvaDbOpen(&dh);
  kvaDb_status = kvaDbReadFile(dh, filename);
  kvaDb_status = kvaDbGetFlags(dh, &kvaDb_flags);

  if (kvaDb_status == kvaDbOK){
    cout << "Database file has been loaded successfully..." << endl;
  }

  return canOK;
}


void Tucson_CAN_writer_to_ctrl::LOOP(){
  ros::Rate rate(freq_for_channel_2);

  while (ros::ok()){
    unsigned char can_data[dlc];
    char *target_msg;
    unsigned short msg_idx;
    KvaDbMessageHnd mh = 0;
    KvaDbSignalHnd sh = 0;
    unsigned int id_write, flag = 0;

    target_msg = (char *)"V2V_Info";
    msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
    kvaDbGetMsgByName(dh, target_msg, &mh);
    kvaDbGetMsgId(mh, &id_write, &flag);

    vector<double> temp(get<1>(msg_list[msg_idx]).size());

    temp = {(double)vehicle_status,\
            (double)turn_sig_right,\
            (double)turn_sig_left,\
            SWA_deg,\
            vel_y_rel,\
            vel_x_rel,\
            pos_y_rel,\
            pos_x_rel};

    for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++){
      kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
      kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp[i]);
    }

    canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
    memset(can_data, 0, sizeof(can_data));

    rate.sleep();
  }
}

int main(int argc, char **argv){
  cout << "Initializing ..." << endl;
  ros::init(argc, argv, "Tucson_CAN_writer_to_ctrl");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/V2V_to_control_team.dbc").c_str());
  int channel_num = 2;
  bool init_access_flag = false; // Init access: no (= CAN handle will be used in multithread)
  ///////////////////////////////////////////////////////////////////////////////////////

  Tucson_CAN_writer_to_ctrl VCW;

  can_status = VCW.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  ros::Subscriber sub1 = node.subscribe("/V2V",         1, &Tucson_CAN_writer_to_ctrl::CALLBACK_V2V, &VCW);
  ros::Subscriber sub2 = node.subscribe("/sensors/V2V", 1, &Tucson_CAN_writer_to_ctrl::CALLBACK_TRANSFORMED_V2V, &VCW);

  VCW.LOOP();

  ros::waitForShutdown();
  canBusOff(hCAN);
  canClose(hCAN);
}