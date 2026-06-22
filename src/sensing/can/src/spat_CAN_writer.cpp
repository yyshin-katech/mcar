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
#include <mutex>

#include <v2x_msgs/intersection_msg.h>
#include <v2x_msgs/intersection_array_msg.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>

#include <algorithm>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>

#include <canlib.h>
#include <kvaDbLib.h>

#include <can_pub.h>

using namespace std;

class SPAT_CAN_WRITER{
  public:
    ros::Subscriber sub1;
    uint16_t cur_intersection_id = 0;
    int      cur_signal_group_id = 0;   // to_control_team.look_at_signalGroupID
    int      cur_manuaver = 0;          // to_control_team.MANUAVER (-1=LEFT, 0=STR, 1=RIGHT)
    std::mutex link_mtx_;               // cur_* 보호

    // OBU SPaT 마지막 수신 시각 (MQTT fallback 판정용).
    // default ros::Time(0) → 한 번도 못 받음 → MQTT 첫 메시지에서 곧바로 fallback.
    ros::Time obu_last_seen_;
    std::mutex spat_mtx_;             // obu_last_seen_ 보호용
    double spat_stale_timeout_ = 2.0; // 초; main 에서 rosparam 으로 갱신

    unsigned int kvaDb_flags = 0;
    unsigned int dlc = 8;

    double time1 = -1;
    float new_time = 0;
    int alive_count = 0;

    bool spat_calling = false;

    vector<tuple<char*, vector<char*>>> msg_list;

    SPAT_CAN_WRITER();
    void CALLBACK_SPAT(const v2x_msgs::intersection_array_msg& data);
    void CALLBACK_LOCAL(const mmc_msgs::to_control_team_from_local_msg& data);
    void CALLBACK_MQTT_SPAT(const v2x_msgs::intersection_array_msg& data);
    void WRITE_SPAT_CAN(const v2x_msgs::intersection_array_msg& msg,
                        const char* source_tag);
    bool OBU_IS_FRESH();
    short FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list);
    canStatus OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag);
    void LOOP();
};

SPAT_CAN_WRITER::SPAT_CAN_WRITER(){
  // List of CAN messages that you want to send to a CAN bus /////////////////////////////////////////////////

  msg_list.push_back(make_tuple((char*)"V2X_SPaT_1",  vector<char*> {(char*)"movementName_1",\
                                                                    (char*)"minEndTime_1",\
                                                                    (char*)"eventState_1",\
                                                                    (char*)"signalGroup_1",\
                                                                    (char*)"Intersection_ID_1"}));
}

void SPAT_CAN_WRITER::CALLBACK_LOCAL(const mmc_msgs::to_control_team_from_local_msg& data){
  std::lock_guard<std::mutex> lock(link_mtx_);
  cur_intersection_id  = data.look_at_IntersectionID;
  cur_signal_group_id  = data.look_at_signalGroupID;
  cur_manuaver         = data.MANUAVER;
}

void SPAT_CAN_WRITER::CALLBACK_SPAT(const v2x_msgs::intersection_array_msg& msg){
  spat_calling = true;
  alive_count = (alive_count + 1) % 256;

  if (time1 == -1)
    time1 = msg.time.sec%10000 + msg.time.nsec/1000000000.0;
  new_time = (msg.time.sec%10000 + msg.time.nsec/1000000000.0) - time1;

  // OBU 우선: 들어오면 항상 CAN write + last_seen 갱신
  {
    std::lock_guard<std::mutex> lock(spat_mtx_);
    obu_last_seen_ = ros::Time::now();
  }
  WRITE_SPAT_CAN(msg, "OBU");
}

void SPAT_CAN_WRITER::CALLBACK_MQTT_SPAT(const v2x_msgs::intersection_array_msg& msg){
  // OBU 가 fresh 이면 MQTT 는 skip
  if (OBU_IS_FRESH())
  {
    ROS_DEBUG_THROTTLE(5.0,
      "[SPaT CAN] MQTT skipped - OBU fresh (last_seen < %.1fs)", spat_stale_timeout_);
    return;
  }
  ROS_INFO_THROTTLE(5.0, "[SPaT CAN] OBU stale -> using MQTT SPaT");
  WRITE_SPAT_CAN(msg, "MQTT");
}

bool SPAT_CAN_WRITER::OBU_IS_FRESH(){
  std::lock_guard<std::mutex> lock(spat_mtx_);
  if (obu_last_seen_.isZero()) return false;
  return (ros::Time::now() - obu_last_seen_).toSec() < spat_stale_timeout_;
}

void SPAT_CAN_WRITER::WRITE_SPAT_CAN(const v2x_msgs::intersection_array_msg& msg,
                                     const char* source_tag)
{
  unsigned char can_data[dlc];
  memset(can_data, 0, sizeof(can_data));

  char* target_msg = (char*)"V2X_SPaT_1";
  unsigned short msg_idx;
  KvaDbMessageHnd mh = 0;
  KvaDbSignalHnd sh = 0;
  unsigned int id_write, flag = 0;
  vector<double> temp_data;

  // ego 링크 정보 스냅샷
  int loc_iid, loc_sig, loc_man;
  {
    std::lock_guard<std::mutex> lock(link_mtx_);
    loc_iid = cur_intersection_id;
    loc_sig = cur_signal_group_id;
    loc_man = cur_manuaver;
  }

  // ego 가 신호 불필요 링크 (IID=0) → CAN 신호 모두 0
  if (loc_iid == 0)
  {
    temp_data = {0.0, 0.0, 0.0, 0.0, 0.0};
  }
  else
  {
    // 다운스트림 매칭: SPaT 안 모든 intersection × movement 중에서
    //   (IID == loc_iid)
    //   AND (loc_sig == 0  OR  SigGrp == loc_sig)
    //   AND (movementName 비어있음  OR  loc_man 에 대응되는 방향)
    // 첫 매칭 항목을 사용. 없으면 CAN 송신 생략.
    const char* target_name = "STR";
    if (loc_man == -1)      target_name = "LEFT";
    else if (loc_man == 1)  target_name = "RIGHT";

    const v2x_msgs::intersection_msg* matched = nullptr;
    for (size_t i = 0; i < msg.data.size(); ++i)
    {
      const auto& d = msg.data[i];
      if ((int)d.IntersectionID != loc_iid) continue;
      if (loc_sig != 0 && (int)d.Movements.SignalGroupID != loc_sig) continue;
      const std::string& mn = d.Movements.MovementStateName;
      if (!mn.empty() && mn != target_name) continue;
      matched = &d;
      break;
    }

    if (matched == nullptr)
    {
      ROS_DEBUG_THROTTLE(2.0,
        "[SPaT CAN/%s] no match for IID=%d SigGrp=%d Move=%s (data=%zu items)",
        source_tag, loc_iid, loc_sig, target_name, msg.data.size());
      return;
    }

    auto& d = *matched;
    temp_data = {0.0,
                 (double)d.Movements.TimeChangeDetails,
                 (double)d.Movements.MovementPhaseStatus,
                 (double)d.Movements.SignalGroupID,
                 (double)d.IntersectionID};

    ROS_INFO_THROTTLE(1.0, "[SPaT CAN/%s] IntID=%d SigGrp=%d Phase=%d minEnd=%.1fs",
             source_tag,
             d.IntersectionID, d.Movements.SignalGroupID,
             d.Movements.MovementPhaseStatus,
             d.Movements.TimeChangeDetails / 10.0);
  }

  msg_idx = FIND_MSG_IDX(target_msg, &msg_list);
  kvaDbGetMsgByName(dh, target_msg, &mh);
  kvaDbGetMsgId(mh, &id_write, &flag);

  for(int j=0; j!=get<1>(msg_list[msg_idx]).size(); j++){
    kvaDbGetSignalByName(mh, get<1>(msg_list[msg_idx])[j], &sh);
    kvaDbStoreSignalValuePhys(sh, &can_data, sizeof(can_data), temp_data[j]);
  }
  canWrite(hCAN, id_write, &can_data, dlc, canMSG_STD);
}

short SPAT_CAN_WRITER::FIND_MSG_IDX(char* target_msg, vector<tuple<char*, vector<char*>>>* msg_list){
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

canStatus SPAT_CAN_WRITER::OPEN_CAN_CHANNEL_AND_READ_DB(int channel_num, char *filename, bool init_access_flag){
  canInitializeLibrary();

  int open_flag;

  if(init_access_flag == true){
      open_flag = canOPEN_REQUIRE_INIT_ACCESS;

    }else{
      open_flag = canOPEN_NO_INIT_ACCESS;
    }

    cout<<"Opening the channel "<<channel_num<<"..."<<endl;
    hCAN = canOpenChannel(channel_num, open_flag);
  
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

void SPAT_CAN_WRITER::LOOP(){
  // ros::Rate rate(20);

  while(ros::ok()){
    // rate.sleep();
  }
}

int main(int argc, char **argv){
  cout<<"Initializing ..."<<endl;
  ros::init(argc, argv, "SPaT_CAN_Writer");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  string relative_path = ros::package::getPath("can");
  char filename[100];

  strcpy(filename, (relative_path + "/dbc/CANdb_IONIQev_PCAN1.dbc").c_str());
  int channel_num = 0;
  bool init_access_flag = false;

  SPAT_CAN_WRITER SPaTCW;
  node.param<double>("spat_stale_timeout", SPaTCW.spat_stale_timeout_, 2.0);

  can_status = SPaTCW.OPEN_CAN_CHANNEL_AND_READ_DB(channel_num, filename, init_access_flag);

  // OBU/MQTT 병합 스트림(/spat_merged) 단일 구독. 교차로 단위 OBU 우선 병합은
  // spat_merge_node 가 처리하므로 여기서는 소스 구분 없이 그대로 CAN write.
  ros::Subscriber sub1 = node.subscribe("/spat_merged", 1, &SPAT_CAN_WRITER::CALLBACK_SPAT, &SPaTCW);
  ros::Subscriber sub2 = node.subscribe("/localization/to_control_team", 1, &SPAT_CAN_WRITER::CALLBACK_LOCAL, &SPaTCW);

  ros::waitForShutdown();
  canBusOff(hCAN);
  canClose(hCAN);
}
