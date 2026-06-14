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

#include <katech_custom_msgs/v_can_msg.h>
#include <mmc_msgs/chassis_msg.h>
#include <katech_custom_msgs/ioniq5_ad_can_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>
#include <mutex>

#include <kvaDbLib.h>
#include <canlib.h>

using namespace std;

ros::Publisher pub1;

canHandle hCAN;
canStatus can_status;

long temp_id;
unsigned long timestamp;
unsigned int id_write, flag, dlc = 64, canread_flag = 0;
unsigned char can_data[64]; // CAN FD

KvaDbStatus kvaDb_status;
KvaDbHnd dh = 0;
KvaDbMessageHnd mh = 0;
KvaDbSignalHnd sh = 0;
unsigned int kvaDb_flags = 0;

unsigned short freq_for_channel_0 = 4000; // Hz
unsigned short timeout_channel_0 = 100;   // ms

char buff[50];
double value;

katech_custom_msgs::v_can_msg vcan_msg;

mmc_msgs::chassis_msg chassis;
std::mutex chassis_mtx;
ros::Publisher pub2;

void ad_can_callback(const katech_custom_msgs::ioniq5_ad_can_msg::ConstPtr& m)
{
  std::lock_guard<std::mutex> lk(chassis_mtx);
  chassis.vcu_ADMDStatus = m->autonomous_mode;
  chassis.AEB_flag = m->AEB_flag;
  chassis.LC_flag  = m->LC_flag;
}

void chassis_timer_cb(const ros::TimerEvent&)
{
  std::lock_guard<std::mutex> lk(chassis_mtx);
  chassis.time = ros::Time::now();
  pub2.publish(chassis);
}

canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag)
{
  canInitializeLibrary();
  int open_flag;

  if (init_access_flag == true)
  {
    open_flag = canOPEN_REQUIRE_INIT_ACCESS | canOPEN_CAN_FD;
  }
  else
  {
    open_flag = canOPEN_NO_INIT_ACCESS | canOPEN_CAN_FD;
  }

  hCAN = canOpenChannel(channel_num, open_flag);

  cout<<hCAN<<endl;

  if (hCAN == canOK)
  {
    cout << "The CAN channel " << channel_num << " has been opened successfully..." << endl;
  }

  can_status = canSetBusParams(hCAN, canBITRATE_500K, 0, 0, 0, 0, 0);
  can_status = canSetBusParamsFd(hCAN, canFD_BITRATE_1M_80P, 0, 0, 0);
  can_status = canSetBusOutputControl(hCAN, canDRIVER_NORMAL);
  can_status = canBusOn(hCAN);

  if (can_status == canOK)
  {
    cout << "The CAN bus is on..." << endl;
  }

  kvaDb_status = kvaDbOpen(&dh);
  kvaDb_status = kvaDbReadFile(dh, filename);
  kvaDb_status = kvaDbGetFlags(dh, &kvaDb_flags);

  if (kvaDb_status == kvaDbOK)
  {
    cout << "The database file has been loaded successfully..." << endl;
  }

  return canOK;
}

void IONIQ5_CAN_READER()
{
  ros::Rate rate(freq_for_channel_0);

  vector<tuple<char *, vector<char *>>> msg_list;

  // 0: GearInfo (ID 117)
  msg_list.push_back(make_tuple((char *)"GearInfo", vector<char *>{(char *)"life_count",
                                                                    (char *)"gear_status"}));

  // 1: TurnSignalInfo (ID 116)
  msg_list.push_back(make_tuple((char *)"TurnSignalInfo", vector<char *>{(char *)"turn_signal_status",
                                                                          (char *)"life_count"}));

  // 2: LongitudinalInfo (ID 115)
  msg_list.push_back(make_tuple((char *)"LongitudinalInfo", vector<char *>{(char *)"motor_rpm",
                                                                           (char *)"acceleration_pedal_pos",
                                                                           (char *)"brake_pedal_pos",
                                                                           (char *)"brake_pressure",
                                                                           (char *)"life_count"}));

  // 3: SteeringInfo (ID 114)
  msg_list.push_back(make_tuple((char *)"SteeringInfo", vector<char *>{(char *)"steering_angle",
                                                                       (char *)"steering_torque",
                                                                       (char *)"steering_angle_rate",
                                                                       (char *)"life_count"}));

  // 4: WheelInfo (ID 113, CAN FD 16 bytes)
  msg_list.push_back(make_tuple((char *)"WheelInfo", vector<char *>{(char *)"wheel_dir_fl",
                                                                     (char *)"wheel_dir_fr",
                                                                     (char *)"wheel_dir_rl",
                                                                     (char *)"wheel_dir_rr",
                                                                     (char *)"wheel_pulse_fl",
                                                                     (char *)"wheel_pulse_fr",
                                                                     (char *)"wheel_pulse_rl",
                                                                     (char *)"wheel_pulse_rr",
                                                                     (char *)"wheel_speed_fl",
                                                                     (char *)"wheel_speed_fr",
                                                                     (char *)"wheel_speed_rl",
                                                                     (char *)"wheel_speed_rr",
                                                                     (char *)"life_count"}));

  // 5: DynamicInfo (ID 112, CAN FD 12 bytes)
  msg_list.push_back(make_tuple((char *)"DynamicInfo", vector<char *>{(char *)"long_acceleration",
                                                                      (char *)"lat_acceleration",
                                                                      (char *)"roll_rate",
                                                                      (char *)"pitch_rate",
                                                                      (char *)"yaw_rate",
                                                                      (char *)"life_count"}));

  // 6: TurnSignalControl (ID 84)
  msg_list.push_back(make_tuple((char *)"TurnSignalControl", vector<char *>{(char *)"turn_signal_control_mode",
                                                                             (char *)"target_turn_signal"}));

  // 7: LateralControl (ID 83)
  msg_list.push_back(make_tuple((char *)"LateralControl", vector<char *>{(char *)"steering_control_mode",
                                                                          (char *)"target_steering_angle"}));

  // 8: GearControl (ID 82)
  msg_list.push_back(make_tuple((char *)"GearControl", vector<char *>{(char *)"gear_control_mode",
                                                                       (char *)"target_gear"}));

  // 9: LongitudinalControl (ID 81)
  msg_list.push_back(make_tuple((char *)"LongitudinalControl", vector<char *>{(char *)"longitudinal_control_mode",
                                                                               (char *)"target_acceleration"}));

  // 10: GearState (ID 21)
  msg_list.push_back(make_tuple((char *)"GearState", vector<char *>{(char *)"gear_ctrl_state",
                                                                     (char *)"gear_override",
                                                                     (char *)"error_code"}));

  // 11: TurnSignalState (ID 20)
  msg_list.push_back(make_tuple((char *)"TurnSignalState", vector<char *>{(char *)"turnsignal_ctrl_status",
                                                                           (char *)"turnsignal_override",
                                                                           (char *)"error_code"}));

  // 12: LongitudinalState (ID 19)
  msg_list.push_back(make_tuple((char *)"LongitudinalState", vector<char *>{(char *)"longitudinal_ctrl_state",
                                                                             (char *)"longitudinal_override",
                                                                             (char *)"error_code"}));

  // 13: LateralState (ID 18)
  msg_list.push_back(make_tuple((char *)"LateralState", vector<char *>{(char *)"lateral_ctrl_state",
                                                                        (char *)"lateral_override",
                                                                        (char *)"error_code"}));

  while (ros::ok())
  {
    dlc = 64;
    can_status = canReadWait(hCAN, &temp_id, can_data, &dlc, &canread_flag, &timestamp, timeout_channel_0);
    kvaDb_status = kvaDbGetMsgById(dh, temp_id, &mh);

    if (kvaDb_status == kvaDbOK)
    {
      kvaDbGetMsgName(mh, buff, sizeof(buff));

      int msg_idx = -1;
      for(int i=0; i < msg_list.size(); i++){
        if(strcmp(buff, get<0>(msg_list[i])) == 0){
          msg_idx = i;
          break;
        }
      }

      if(msg_idx < 0){
        rate.sleep();
        continue;
      }

      // 시그널 값 읽기
      for (int i = 0; i != get<1>(msg_list[msg_idx]).size(); i++)
      {
        kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[i], &sh);
        kvaDbRetrieveSignalValuePhys(sh, &value, can_data, dlc);

        switch(msg_idx){

          case(0): // GearInfo
            switch(i){
              case(0): vcan_msg.life_count_gearinfo = (uint8_t)value; break;
              case(1): vcan_msg.gear_status = (uint8_t)value; break;
            }
          break;

          case(1): // TurnSignalInfo
            switch(i){
              case(0): vcan_msg.turn_signal_status = (uint8_t)value; break;
              case(1): vcan_msg.life_count_turnsignalinfo = (uint8_t)value; break;
            }
          break;

          case(2): // LongitudinalInfo
            switch(i){
              case(0): vcan_msg.motor_rpm = value; break;
              case(1): vcan_msg.acceleration_pedal_pos = value; break;
              case(2): vcan_msg.brake_pedal_pos = value; break;
              case(3): vcan_msg.brake_pressure = value; break;
              case(4): vcan_msg.life_count_longitudinalinfo = (uint8_t)value; break;
            }
          break;

          case(3): // SteeringInfo
            switch(i){
              case(0): vcan_msg.steering_angle = value; break;
              case(1): vcan_msg.steering_torque = value; break;
              case(2): vcan_msg.steering_angle_rate = value; break;
              case(3): vcan_msg.life_count_steeringinfo = (uint8_t)value; break;
            }
          break;

          case(4): // WheelInfo (CAN FD)
            switch(i){
              case(0):  vcan_msg.wheel_dir_fl = (uint8_t)value; break;
              case(1):  vcan_msg.wheel_dir_fr = (uint8_t)value; break;
              case(2):  vcan_msg.wheel_dir_rl = (uint8_t)value; break;
              case(3):  vcan_msg.wheel_dir_rr = (uint8_t)value; break;
              case(4):  vcan_msg.wheel_pulse_fl = (uint8_t)value; break;
              case(5):  vcan_msg.wheel_pulse_fr = (uint8_t)value; break;
              case(6):  vcan_msg.wheel_pulse_rl = (uint8_t)value; break;
              case(7):  vcan_msg.wheel_pulse_rr = (uint8_t)value; break;
              case(8):  vcan_msg.wheel_speed_fl = value; break;
              case(9):  vcan_msg.wheel_speed_fr = value; break;
              case(10): vcan_msg.wheel_speed_rl = value; break;
              case(11): vcan_msg.wheel_speed_rr = value; break;
              case(12): vcan_msg.life_count_wheelinfo = (uint8_t)value; break;
            }
          break;

          case(5): // DynamicInfo (CAN FD)
            switch(i){
              case(0): vcan_msg.long_acceleration = value; break;
              case(1): vcan_msg.lat_acceleration = value; break;
              case(2): vcan_msg.roll_rate = value; break;
              case(3): vcan_msg.pitch_rate = value; break;
              case(4): vcan_msg.yaw_rate = value; break;
              case(5): vcan_msg.life_count_dynamicinfo = (uint8_t)value; break;
            }
          break;

          case(6): // TurnSignalControl
            switch(i){
              case(0): vcan_msg.turn_signal_control_mode = (uint8_t)value; break;
              case(1): vcan_msg.target_turn_signal = (uint8_t)value; break;
            }
          break;

          case(7): // LateralControl
            switch(i){
              case(0): vcan_msg.lat_steering_control_mode = (uint8_t)value; break;
              case(1): vcan_msg.lat_target_steering_angle = value; break;
            }
          break;

          case(8): // GearControl
            switch(i){
              case(0): vcan_msg.gear_control_mode = (uint8_t)value; break;
              case(1): vcan_msg.target_gear = (uint8_t)value; break;
            }
          break;

          case(9): // LongitudinalControl
            switch(i){
              case(0): vcan_msg.longitudinal_control_mode = (uint8_t)value; break;
              case(1): vcan_msg.target_acceleration = value; break;
            }
          break;

          case(10): // GearState
            switch(i){
              case(0): vcan_msg.gear_ctrl_state = (uint8_t)value; break;
              case(1): vcan_msg.gear_override = (uint8_t)value; break;
              case(2): vcan_msg.gear_error_code = (uint8_t)value; break;
            }
          break;

          case(11): // TurnSignalState
            switch(i){
              case(0): vcan_msg.turnsignal_ctrl_status = (uint8_t)value; break;
              case(1): vcan_msg.turnsignal_override = (uint8_t)value; break;
              case(2): vcan_msg.turnsignal_error_code = (uint8_t)value; break;
            }
          break;

          case(12): // LongitudinalState
            switch(i){
              case(0): vcan_msg.longitudinal_ctrl_state = (uint8_t)value; break;
              case(1): vcan_msg.longitudinal_override = (uint8_t)value; break;
              case(2): vcan_msg.longitudinal_error_code = (uint8_t)value; break;
            }
          break;

          case(13): // LateralState
            switch(i){
              case(0): vcan_msg.lateral_ctrl_state = (uint8_t)value; break;
              case(1): vcan_msg.lateral_override = (uint8_t)value; break;
              case(2): vcan_msg.lateral_error_code = (uint8_t)value; break;
            }
          break;
        }
      }

      pub1.publish(vcan_msg);

      {
        std::lock_guard<std::mutex> lk(chassis_mtx);
        chassis.vcu_EPS_Status = (vcan_msg.lateral_ctrl_state == 1) ? 2 : 0;
        chassis.vcu_ACC_Status = vcan_msg.longitudinal_ctrl_state;
        chassis.vcu_SAS_Angle  = vcan_msg.steering_angle;
        chassis.vcu_LONG_ACCEL = vcan_msg.long_acceleration;
        chassis.vcu_VS = (vcan_msg.wheel_speed_fl + vcan_msg.wheel_speed_fr +
                          vcan_msg.wheel_speed_rl + vcan_msg.wheel_speed_rr) / 4.0 * 3.6;
        chassis.vcu_LeftTurnCtl    = (vcan_msg.target_turn_signal == 1 || vcan_msg.target_turn_signal == 3) ? 1 : 0;
        chassis.vcu_RightTurnCtl   = (vcan_msg.target_turn_signal == 2 || vcan_msg.target_turn_signal == 3) ? 1 : 0;
        chassis.vcu_HazardCtl      = (vcan_msg.target_turn_signal == 3) ? 1 : 0;
        chassis.vcu_LeftTurnState  = (vcan_msg.turn_signal_status == 1 || vcan_msg.turn_signal_status == 3) ? 1 : 0;
        chassis.vcu_RightTurnState = (vcan_msg.turn_signal_status == 2 || vcan_msg.turn_signal_status == 3) ? 1 : 0;
      }
    }
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  cout << "Initializing ..." << endl;
  ros::init(argc, argv, "IONIQ5_CAN_reader");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  pub1 = node.advertise<katech_custom_msgs::v_can_msg>("/sensors/v_can", 1);
  pub2 = node.advertise<mmc_msgs::chassis_msg>("/sensors/chassis", 1);

  ros::Subscriber sub_ad = node.subscribe("/sensors/ioniq5_ad_can", 1, ad_can_callback);
  ros::Timer chassis_timer = node.createTimer(ros::Duration(0.02), chassis_timer_cb);

  string relative_path = ros::package::getPath("can");
  char filename[100];

  //////////////////////////////////// Parameters ///////////////////////////////////////
  strcpy(filename, (relative_path + "/dbc/V_CAN_Release.dbc").c_str());
  int channel_num = 2;
  bool init_access_flag = true;
  ///////////////////////////////////////////////////////////////////////////////////////

  can_status = OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  if (can_status == canOK)
  {
    IONIQ5_CAN_READER();
  }

  ros::waitForShutdown();
  canBusOff(hCAN);
  canClose(hCAN);
}
