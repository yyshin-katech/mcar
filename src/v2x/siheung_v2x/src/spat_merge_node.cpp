// spat_merge_node
// -----------------------------------------------------------------------------
// OBU SPaT(/siheung_spat) 와 MQTT SPaT(/siheung_v2x/mqtt_spat) 를 교차로(IID)
// 단위로 병합하여 단일 토픽(/spat_merged) 으로 재발행한다.
//
// 병합 정책 (교차로 단위 OBU 우선):
//   - OBU 가 해당 IntersectionID 를 obu_priority_timeout(기본 2s) 내에 실어 보내면
//     그 교차로는 OBU 소스를 사용한다.
//   - OBU 가 싣지 않는 교차로(예: 302) 는 MQTT 소스로 채운다.
//   - 같은 교차로에서 두 소스를 섞지 않는다(발행 시점에 IID별 단일 소스 결정).
//   - entry_ttl(기본 3s) 보다 오래된 movement 는 출력에서 제외(죽은 신호 제거).
//
// 다운스트림(spat_CAN_writer, stat_display, pyqt/web hmi) 은 /siheung_spat 대신
// 이 /spat_merged 를 구독한다. v2x_diagnostic 은 OBU 건전성 확인용으로 raw
// /siheung_spat 를 그대로 둔다.
// -----------------------------------------------------------------------------

#include <ros/ros.h>
#include <v2x_msgs/intersection_array_msg.h>
#include <v2x_msgs/intersection_msg.h>

#include <map>
#include <tuple>
#include <string>
#include <mutex>

class SpatMerger
{
public:
  SpatMerger(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("obu_topic",    obu_topic_,    "/siheung_spat");
    pnh.param<std::string>("mqtt_topic",   mqtt_topic_,   "/siheung_v2x/mqtt_spat");
    pnh.param<std::string>("merged_topic", merged_topic_, "/spat_merged");
    pnh.param<double>("obu_priority_timeout", obu_priority_timeout_, 2.0);
    pnh.param<double>("entry_ttl",            entry_ttl_,            3.0);
    pnh.param<double>("publish_rate",         publish_rate_,         10.0);

    pub_ = nh.advertise<v2x_msgs::intersection_array_msg>(merged_topic_, 1);
    sub_obu_  = nh.subscribe(obu_topic_,  1, &SpatMerger::cbObu,  this);
    sub_mqtt_ = nh.subscribe(mqtt_topic_, 1, &SpatMerger::cbMqtt, this);

    timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_),
                            &SpatMerger::onTimer, this);

    ROS_INFO("[spat_merge] OBU=%s + MQTT=%s -> %s (obu_priority=%.1fs ttl=%.1fs rate=%.1fHz)",
             obu_topic_.c_str(), mqtt_topic_.c_str(), merged_topic_.c_str(),
             obu_priority_timeout_, entry_ttl_, publish_rate_);
  }

private:
  enum Source { SRC_OBU = 0, SRC_MQTT = 1 };

  // movement 단위 키: (IntersectionID, SignalGroupID, MovementStateName)
  typedef std::tuple<uint16_t, uint8_t, std::string> Key;

  struct Entry {
    v2x_msgs::intersection_msg elem;
    Source    source;
    ros::Time stamp;
  };

  static Key makeKey(const v2x_msgs::intersection_msg& d) {
    return std::make_tuple((uint16_t)d.IntersectionID,
                           (uint8_t)d.Movements.SignalGroupID,
                           d.Movements.MovementStateName);
  }

  void cbObu(const v2x_msgs::intersection_array_msg& msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    ros::Time now = ros::Time::now();
    for (const auto& d : msg.data) {
      obu_iid_last_seen_[(uint16_t)d.IntersectionID] = now;
      cache_[makeKey(d)] = Entry{d, SRC_OBU, now};
    }
  }

  void cbMqtt(const v2x_msgs::intersection_array_msg& msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    ros::Time now = ros::Time::now();
    for (const auto& d : msg.data) {
      // OBU 가 이 교차로를 fresh 하게 싣고 있으면 MQTT 는 무시(OBU 우선)
      auto it = obu_iid_last_seen_.find((uint16_t)d.IntersectionID);
      if (it != obu_iid_last_seen_.end() &&
          (now - it->second).toSec() < obu_priority_timeout_) {
        continue;
      }
      cache_[makeKey(d)] = Entry{d, SRC_MQTT, now};
    }
  }

  void onTimer(const ros::TimerEvent&) {
    v2x_msgs::intersection_array_msg out;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      ros::Time now = ros::Time::now();

      // IID 별 승자 소스 결정: OBU 가 fresh 면 OBU, 아니면 MQTT
      std::map<uint16_t, Source> winner;
      for (const auto& kv : obu_iid_last_seen_) {
        if ((now - kv.second).toSec() < obu_priority_timeout_)
          winner[kv.first] = SRC_OBU;
      }

      // ttl 지난 엔트리 제거 + 승자 소스 엔트리만 출력
      for (auto it = cache_.begin(); it != cache_.end(); ) {
        if ((now - it->second.stamp).toSec() >= entry_ttl_) {
          it = cache_.erase(it);
          continue;
        }
        uint16_t iid = std::get<0>(it->first);
        auto w = winner.find(iid);
        Source win = (w != winner.end()) ? w->second : SRC_MQTT;
        if (it->second.source == win)
          out.data.push_back(it->second.elem);
        ++it;
      }
    }
    out.time = ros::Time::now();
    pub_.publish(out);
  }

  ros::Publisher  pub_;
  ros::Subscriber sub_obu_;
  ros::Subscriber sub_mqtt_;
  ros::Timer      timer_;

  std::string obu_topic_, mqtt_topic_, merged_topic_;
  double obu_priority_timeout_, entry_ttl_, publish_rate_;

  std::mutex mtx_;
  std::map<Key, Entry>          cache_;
  std::map<uint16_t, ros::Time> obu_iid_last_seen_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spat_merge_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SpatMerger merger(nh, pnh);
  ros::spin();
  return 0;
}
