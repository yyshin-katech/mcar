// web_hmi_threejs_tracks_node.cpp
//
// C++ port of the `/hmi/threejs/tracks` publisher path inside
// scripts/web_hmi_threejs_bridge.py. Shapefile/map publishing stays in
// Python (option 2 in _bridge_cpp_workspace/01_port_spec.md) — this node
// owns ONLY the percept → tracks JSON aggregation hot path.
//
// Inputs:
//   /percept_topic              perception_ros_msg/RsPerceptionMsg
//   /fusion_lidar_points        sensor_msgs/PointCloud2
//   /localization/to_control_team   mmc_msgs/to_control_team_from_local_msg
// Output:
//   /hmi/threejs/tracks         std_msgs/String  (JSON payload)
//
// Payload keys (kept BYTE-COMPATIBLE with Python):
//   stamp, tracks[{id,tid,type,x,y,vx,vy,size_x,size_y,orientation,
//                  confidence, points?}], ego_at_emit?{east,north,yaw}
//
// Sort/cap order (must match Python):
//   1. confidence < PERCEPT_MIN_CONFIDENCE drop
//   2. distance² ASC
//   3. top TRACKS_MAX_RENDERED
//   4. per-track cloud_indices slice (cap PERCEPT_MAX_POINTS_PER_TRACK)
//
// Single-threaded ros::spin() — no mutex on caches.

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <perception_ros_msg/RsPerceptionMsg.h>
#include <perception_ros_msg/Object.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>

#include <json/json.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

// Constants — must stay identical to web_hmi_threejs_bridge.py.
constexpr std::size_t PERCEPT_MAX_POINTS_PER_TRACK = 256;
constexpr std::size_t TRACKS_MAX_RENDERED         = 6;
constexpr float       PERCEPT_MIN_CONFIDENCE      = 0.9f;

inline const char* percept_type_str(int t) {
  return (t == 1) ? "pedestrian" : "car";
}

inline double round_mm(double v) {
  // np.round(pts, 3) equivalent. Inputs are float32 so rounding-tie
  // differences vs Python's banker's rounding are inside FP noise.
  return std::round(v * 1000.0) / 1000.0;
}

struct EgoSnap {
  double east  = 0.0;
  double north = 0.0;
  double yaw   = 0.0;
};

// Per-percept candidate accumulator. We collect lightweight metadata first,
// then sort & cap, then do the heavy cloud slice for survivors only.
struct Candidate {
  int    id        = 0;
  int    tid       = 0;
  int    type_int  = 0;
  float  x         = 0.f;
  float  y         = 0.f;
  float  vx        = 0.f;
  float  vy        = 0.f;
  float  size_x    = 0.f;
  float  size_y    = 0.f;
  float  orient    = 0.f;
  float  conf      = 0.f;
  float  dist2     = 0.f;
  const perception_ros_msg::Object* obj = nullptr;
};

class WebHmiThreejsTracksCpp {
public:
  explicit WebHmiThreejsTracksCpp(ros::NodeHandle& nh) {
    pub_tracks_ = nh.advertise<std_msgs::String>(
        "/hmi/threejs/tracks", 2, /*latch=*/false);

    sub_local_ = nh.subscribe<mmc_msgs::to_control_team_from_local_msg>(
        "/localization/to_control_team", 4,
        &WebHmiThreejsTracksCpp::onLocal, this,
        ros::TransportHints().tcpNoDelay());

    sub_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>(
        "/fusion_lidar_points", 1,
        &WebHmiThreejsTracksCpp::onCloud, this,
        ros::TransportHints().tcpNoDelay());

    sub_percept_ = nh.subscribe<perception_ros_msg::RsPerceptionMsg>(
        "/percept_topic", 1,
        &WebHmiThreejsTracksCpp::onPercept, this,
        ros::TransportHints().tcpNoDelay());

    log_timer_ = nh.createTimer(
        ros::Duration(5.0),
        &WebHmiThreejsTracksCpp::onLogRate, this);

    // jsoncpp writer config — compact, NaN/Inf is filtered upstream.
    writer_builder_["indentation"]   = "";
    writer_builder_["commentStyle"]  = "None";
  }

private:
  void onLocal(const mmc_msgs::to_control_team_from_local_msg::ConstPtr& msg) {
    EgoSnap snap;
    snap.east  = static_cast<double>(msg->host_east);
    snap.north = static_cast<double>(msg->host_north);
    snap.yaw   = static_cast<double>(msg->host_yaw);
    last_ego_   = snap;
    has_last_ego_ = true;
  }

  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!msg) return;
    const std::size_t n = static_cast<std::size_t>(msg->width) *
                          static_cast<std::size_t>(msg->height);
    if (n == 0 || msg->point_step < 12) {
      cloud_msg_.reset();
      cloud_n_ = 0;
      cloud_stride_floats_ = 0;
      return;
    }
    if (msg->is_bigendian) {
      ROS_WARN_ONCE(
          "web_hmi_threejs_tracks: big-endian PointCloud2 unsupported");
      cloud_msg_.reset();
      cloud_n_ = 0;
      cloud_stride_floats_ = 0;
      return;
    }
    if ((msg->point_step % sizeof(float)) != 0) {
      ROS_WARN_ONCE(
          "web_hmi_threejs_tracks: point_step %u not multiple of 4",
          msg->point_step);
      return;
    }
    if (msg->data.size() != n * static_cast<std::size_t>(msg->point_step)) {
      return;
    }
    // Hold the shared_ptr — refcount bump only, no copy. Lifetime extends
    // until the next /fusion_lidar_points message arrives.
    cloud_msg_ = msg;
    cloud_n_ = n;
    cloud_stride_floats_ =
        static_cast<std::size_t>(msg->point_step) / sizeof(float);
  }

  void onPercept(
      const perception_ros_msg::RsPerceptionMsg::ConstPtr& msg) {
    if (!msg) return;
    const auto& lf = msg->lidarframe;
    const double stamp = lf.timestamp.data;

    const auto& objects = lf.objects.objects;
    candidates_.clear();
    candidates_.reserve(objects.size());

    for (const auto& obj : objects) {
      const auto& ci = obj.coreinfo;
      const float conf = ci.exist_confidence.data;
      if (conf < PERCEPT_MIN_CONFIDENCE) continue;

      Candidate c;
      c.tid      = ci.trakcer_id.data;     // typo preserved (msg field name)
      c.id       = c.tid;
      c.type_int = ci.type.data;
      c.x        = ci.center.x.data;
      c.y        = ci.center.y.data;
      c.vx       = ci.velocity.x.data;
      c.vy       = ci.velocity.y.data;
      c.size_x   = ci.size.x.data;
      c.size_y   = ci.size.y.data;
      c.orient   = std::atan2(
          static_cast<float>(ci.direction.y.data),
          static_cast<float>(ci.direction.x.data));
      c.conf     = conf;
      c.dist2    = c.x * c.x + c.y * c.y;
      c.obj      = &obj;
      candidates_.push_back(c);
    }

    // dist² ASC, top TRACKS_MAX_RENDERED. partial_sort is O(N log K).
    if (candidates_.size() > TRACKS_MAX_RENDERED) {
      std::partial_sort(
          candidates_.begin(),
          candidates_.begin() + TRACKS_MAX_RENDERED,
          candidates_.end(),
          [](const Candidate& a, const Candidate& b) {
            return a.dist2 < b.dist2;
          });
      candidates_.resize(TRACKS_MAX_RENDERED);
    } else {
      std::sort(
          candidates_.begin(), candidates_.end(),
          [](const Candidate& a, const Candidate& b) {
            return a.dist2 < b.dist2;
          });
    }

    Json::Value root(Json::objectValue);
    root["stamp"] = stamp;

    Json::Value tracks_arr(Json::arrayValue);
    std::size_t with_points = 0;
    for (const auto& c : candidates_) {
      Json::Value t(Json::objectValue);
      t["id"]          = c.id;
      t["tid"]         = c.tid;
      t["type"]        = percept_type_str(c.type_int);
      t["x"]           = static_cast<double>(c.x);
      t["y"]           = static_cast<double>(c.y);
      t["vx"]          = static_cast<double>(c.vx);
      t["vy"]          = static_cast<double>(c.vy);
      t["size_x"]      = static_cast<double>(c.size_x);
      t["size_y"]      = static_cast<double>(c.size_y);
      t["orientation"] = static_cast<double>(c.orient);
      t["confidence"]  = static_cast<double>(c.conf);

      if (cloud_msg_ && c.obj && c.obj->hassupplmentinfo.data) {
        Json::Value pts(Json::arrayValue);
        slicePoints(*c.obj, pts);
        if (!pts.empty()) {
          t["points"] = std::move(pts);
          ++with_points;
        }
      }
      tracks_arr.append(std::move(t));
    }
    root["tracks"] = std::move(tracks_arr);

    if (has_last_ego_) {
      Json::Value ego(Json::objectValue);
      ego["east"]  = last_ego_.east;
      ego["north"] = last_ego_.north;
      ego["yaw"]   = last_ego_.yaw;
      root["ego_at_emit"] = std::move(ego);
    }

    std_msgs::String out;
    out.data = Json::writeString(writer_builder_, root);
    pub_tracks_.publish(out);

    tracks_count_      += candidates_.size();
    with_points_count_ += with_points;
  }

  void slicePoints(const perception_ros_msg::Object& obj,
                   Json::Value& out_points) const {
    if (!cloud_msg_ || cloud_n_ == 0 || cloud_stride_floats_ == 0) return;
    const auto& indices = obj.supplementinfo.cloud_indices;
    const std::size_t n_idx = indices.size();
    if (n_idx == 0) return;

    const std::size_t n_take =
        n_idx <= PERCEPT_MAX_POINTS_PER_TRACK
            ? n_idx
            : PERCEPT_MAX_POINTS_PER_TRACK;

    const float* base =
        reinterpret_cast<const float*>(cloud_msg_->data.data());

    for (std::size_t k = 0; k < n_take; ++k) {
      const std::int32_t raw = indices[k].data;
      if (raw < 0) continue;
      const std::size_t idx = static_cast<std::size_t>(raw);
      if (idx >= cloud_n_) continue;
      const std::size_t off = idx * cloud_stride_floats_;
      const float x = base[off + 0];
      const float y = base[off + 1];
      const float z = base[off + 2];
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      out_points.append(round_mm(static_cast<double>(x)));
      out_points.append(round_mm(static_cast<double>(y)));
      out_points.append(round_mm(static_cast<double>(z)));
    }
  }

  void onLogRate(const ros::TimerEvent&) {
    if (tracks_count_ == 0) return;
    const double rate =
        100.0 * static_cast<double>(with_points_count_) /
        static_cast<double>(std::max<std::size_t>(1, tracks_count_));
    ROS_INFO(
        "web_hmi_threejs_tracks: %zu tracks last 5s, %zu (%.0f%%) with points",
        tracks_count_, with_points_count_, rate);
    tracks_count_      = 0;
    with_points_count_ = 0;
  }

  // ROS handles
  ros::Publisher  pub_tracks_;
  ros::Subscriber sub_local_;
  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_percept_;
  ros::Timer      log_timer_;

  // Caches
  sensor_msgs::PointCloud2::ConstPtr cloud_msg_;
  std::size_t cloud_n_             = 0;
  std::size_t cloud_stride_floats_ = 0;

  EgoSnap last_ego_;
  bool    has_last_ego_ = false;

  // Reused per-callback scratch (avoids re-alloc each tick).
  std::vector<Candidate> candidates_;

  // Counters (5s log window).
  std::size_t tracks_count_      = 0;
  std::size_t with_points_count_ = 0;

  // JSON writer config.
  Json::StreamWriterBuilder writer_builder_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "web_hmi_threejs_tracks_cpp");
  ros::NodeHandle nh;
  WebHmiThreejsTracksCpp node(nh);
  ROS_INFO("web_hmi_threejs_tracks_cpp ready");
  ros::spin();
  return 0;
}
