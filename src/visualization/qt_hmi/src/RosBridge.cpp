// qt_hmi/src/RosBridge.cpp
//
// JSON parsing uses Qt's built-in QJsonDocument (no external deps; sufficient
// for ≤200 KB payloads at 10 Hz). The /hmi/threejs/map payload can grow to
// several MB; M1 parses it synchronously on a spinner thread (acceptable for
// a one-shot latched topic). M3 will revisit if startup feels sluggish.
//
// Each callback parses, builds a struct, and emits a Qt signal. The signal
// is queued onto the Qt main thread automatically because RosBridge lives
// there (the AsyncSpinner threads only invoke the ROS callbacks, never the
// signal slots).
#include "qt_hmi/RosBridge.h"

#include <QtCore/QDebug>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonValue>
#include <QtCore/QString>
#include <QtCore/QStringList>

namespace {

// Helper: extract double / int / bool / string with safe defaults.
inline double jdouble(const QJsonValue& v, double def = 0.0) {
  return v.isDouble() ? v.toDouble() : def;
}
inline int jint(const QJsonValue& v, int def = 0) {
  return v.isDouble() ? static_cast<int>(v.toDouble()) : def;
}
inline bool jbool(const QJsonValue& v, bool def = false) {
  return v.isBool() ? v.toBool() : def;
}
inline QString jstr(const QJsonValue& v, const QString& def = QString()) {
  return v.isString() ? v.toString() : def;
}

// Parse the payload field of a std_msgs/String into a QJsonObject.
// Returns std::nullopt-ish via `ok` flag (Qt5 has no std::optional friendly).
QJsonObject parseObject(const std_msgs::String::ConstPtr& msg, bool* ok) {
  QByteArray bytes(msg->data.data(),
                   static_cast<int>(msg->data.size()));
  QJsonParseError err{};
  QJsonDocument doc = QJsonDocument::fromJson(bytes, &err);
  if (err.error != QJsonParseError::NoError || !doc.isObject()) {
    *ok = false;
    return {};
  }
  *ok = true;
  return doc.object();
}

}  // namespace

RosBridge::RosBridge(QObject* parent)
    : QObject(parent), spinner_(2) {
  qRegisterMetaType<HmiState>("HmiState");
  qRegisterMetaType<HmiObject>("HmiObject");
  qRegisterMetaType<QVector<HmiObject>>("QVector<HmiObject>");
  qRegisterMetaType<QHash<QString, int>>("QHash<QString,int>");
  qRegisterMetaType<QHash<QString, double>>("QHash<QString,double>");
  qRegisterMetaType<HmiMap3D>("HmiMap3D");
  qRegisterMetaType<HmiTrack3D>("HmiTrack3D");
  qRegisterMetaType<QVector<HmiTrack3D>>("QVector<HmiTrack3D>");
  qRegisterMetaType<Polyline>("Polyline");
  qRegisterMetaType<QVector<Polyline>>("QVector<Polyline>");

  // Subscribers — 9 total. Latched topics use queue 1, periodic topics 2.
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/state",            2, &RosBridge::onState,       this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/diagnostics",      2, &RosBridge::onDiag,        this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/topic_hz",         2, &RosBridge::onHz,          this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/objects",          2, &RosBridge::onObjects,     this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/popup",            1, &RosBridge::onPopup,       this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/traffic",          1, &RosBridge::onTraffic,     this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/bag",              1, &RosBridge::onBag,         this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/map",              1, &RosBridge::onMap,         this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/threejs/map",      1, &RosBridge::onThreejsMap,  this));
  subs_.push_back(nh_.subscribe<std_msgs::String>(
      "/hmi/threejs/tracks",   2, &RosBridge::onTracks,      this));
  // 50 Hz ego pose — bypasses web_hmi_bridge's 10 Hz throttle for camera
  // follow at rviz cadence. Same source field (host_east/north/yaw) as
  // /hmi/state.ego, just at native rate.
  subs_.push_back(
      nh_.subscribe<mmc_msgs::to_control_team_from_local_msg>(
          "/localization/to_control_team", 4,
          &RosBridge::onLocal, this));

  // Publishers.
  pubModeReq_   = nh_.advertise<std_msgs::Bool>("/hmi/cmd/mode_request", 1);
  pubBagToggle_ = nh_.advertise<std_msgs::Empty>("/hmi/cmd/bag_toggle",  1);

  spinner_.start();
  spinnerRunning_ = true;
}

RosBridge::~RosBridge() {
  // Defensive — main.cpp normally calls stop() via aboutToQuit first.
  stop();
}

void RosBridge::stop() {
  if (!spinnerRunning_) return;
  spinner_.stop();
  spinnerRunning_ = false;
}

// ─── outbound ────────────────────────────────────────────────────────────
void RosBridge::publishModeRequest(bool autonomous) {
  std_msgs::Bool m;
  m.data = autonomous;
  pubModeReq_.publish(m);
}

void RosBridge::publishBagToggle() {
  std_msgs::Empty m;
  pubBagToggle_.publish(m);
}

// ─── inbound (JSON → struct → emit) ──────────────────────────────────────
void RosBridge::onState(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;

  HmiState s;
  s.speed        = jdouble(o.value("speed"));
  s.steering     = jdouble(o.value("steering"));
  s.gear         = jint(o.value("gear"));
  s.mode         = jint(o.value("mode"));
  s.aeb          = jbool(o.value("aeb"));
  s.speedLimit   = jint(o.value("speed_limit"));
  s.linkId       = jint(o.value("link_id"));
  s.onOdd        = jint(o.value("on_odd"));
  s.roadState    = jint(o.value("road_state"));
  s.selectedMode = jint(o.value("selected_mode"));
  s.laneLabel    = jstr(o.value("lane_label"));

  if (o.value("ego").isObject()) {
    QJsonObject e = o.value("ego").toObject();
    s.ego.east  = jdouble(e.value("east"));
    s.ego.north = jdouble(e.value("north"));
    s.ego.yaw   = jdouble(e.value("yaw"));
  }
  if (o.value("gps").isObject()) {
    QJsonObject g = o.value("gps").toObject();
    s.gps.rtk    = jint(g.value("rtk"));
    s.gps.lonStd = jdouble(g.value("lon_std"));
    s.gps.latStd = jdouble(g.value("lat_std"));
  }

  emit stateChanged(s);
}

void RosBridge::onDiag(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;

  QHash<QString, int> status;
  if (o.value("status").isObject()) {
    QJsonObject st = o.value("status").toObject();
    for (auto it = st.constBegin(); it != st.constEnd(); ++it) {
      status.insert(it.key(), jint(it.value()));
    }
  }
  emit diagChanged(status);
}

void RosBridge::onHz(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;

  QHash<QString, double> hz;
  for (auto it = o.constBegin(); it != o.constEnd(); ++it) {
    hz.insert(it.key(), jdouble(it.value()));
  }
  emit hzChanged(hz);
}

void RosBridge::onObjects(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;

  QVector<HmiObject> out;
  if (o.value("data").isArray()) {
    QJsonArray arr = o.value("data").toArray();
    out.reserve(arr.size());
    for (const QJsonValue& v : arr) {
      if (!v.isObject()) continue;
      QJsonObject obj = v.toObject();
      HmiObject ho;
      ho.id          = jint(obj.value("id"));
      ho.type        = jstr(obj.value("type"));
      ho.x           = jdouble(obj.value("x"));
      ho.y           = jdouble(obj.value("y"));
      ho.vx          = jdouble(obj.value("vx"));
      ho.vy          = jdouble(obj.value("vy"));
      ho.width       = jdouble(obj.value("width"));
      ho.length      = jdouble(obj.value("length"));
      ho.orientation = jdouble(obj.value("orientation"));
      out.push_back(ho);
    }
  }
  emit objectsChanged(out);
}

void RosBridge::onPopup(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;
  emit popupChanged(jstr(o.value("text")), jstr(o.value("severity")));
}

void RosBridge::onTraffic(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;

  int color = jint(o.value("color"));
  int td    = jint(o.value("time_decisec"));
  int interId = 0, sgId = 0;
  if (o.value("look_at").isObject()) {
    QJsonObject la = o.value("look_at").toObject();
    interId = jint(la.value("intersection_id"));
    sgId    = jint(la.value("signal_group_id"));
  }
  emit trafficChanged(color, td, interId, sgId);
}

void RosBridge::onBag(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;
  emit bagChanged(jbool(o.value("recording")), jstr(o.value("info")));
}

void RosBridge::onMap(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;

  QVector<Polyline> out;
  if (o.value("polylines").isArray()) {
    QJsonArray arr = o.value("polylines").toArray();
    out.reserve(arr.size());
    for (const QJsonValue& v : arr) {
      if (!v.isArray()) continue;
      QJsonArray pts = v.toArray();
      Polyline pl;
      pl.pts.reserve(pts.size());
      for (const QJsonValue& pv : pts) {
        if (!pv.isArray()) continue;
        QJsonArray pa = pv.toArray();
        if (pa.size() < 2) continue;
        pl.pts.append(QPointF(jdouble(pa.at(0)), jdouble(pa.at(1))));
      }
      out.push_back(std::move(pl));
    }
  }
  emit mapPolylinesReceived(out);
}

void RosBridge::onThreejsMap(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;

  HmiMap3D out;
  out.epsg = jint(o.value("epsg"), 5179);
  if (o.value("origin").isArray()) {
    QJsonArray oa = o.value("origin").toArray();
    if (oa.size() >= 2) {
      out.origin = QPointF(jdouble(oa.at(0)), jdouble(oa.at(1)));
    }
  }
  if (o.value("layers").isObject()) {
    QJsonObject layers = o.value("layers").toObject();
    for (auto it = layers.constBegin(); it != layers.constEnd(); ++it) {
      if (!it.value().isObject()) continue;
      QJsonObject lo = it.value().toObject();
      HmiLayer3D L;
      L.kind = jstr(lo.value("kind"));
      const QJsonValue dv = lo.value("data");
      if (dv.isArray()) {
        QJsonArray arr = dv.toArray();
        if (L.kind == "point") {
          // [[e,n], [e,n], ...] → single feature
          QVector<QPointF> single;
          single.reserve(arr.size());
          for (const QJsonValue& pv : arr) {
            if (!pv.isArray()) continue;
            QJsonArray pa = pv.toArray();
            if (pa.size() < 2) continue;
            single.append(QPointF(jdouble(pa.at(0)), jdouble(pa.at(1))));
          }
          L.features.append(std::move(single));
        } else {
          // polyline / polygon: [[ [e,n],... ], [ [e,n],... ], ...]
          for (const QJsonValue& fv : arr) {
            if (!fv.isArray()) continue;
            QJsonArray fa = fv.toArray();
            QVector<QPointF> verts;
            verts.reserve(fa.size());
            for (const QJsonValue& pv : fa) {
              if (!pv.isArray()) continue;
              QJsonArray pa = pv.toArray();
              if (pa.size() < 2) continue;
              verts.append(QPointF(jdouble(pa.at(0)), jdouble(pa.at(1))));
            }
            L.features.append(std::move(verts));
          }
        }
      }
      out.layers.insert(it.key(), std::move(L));
    }
  }
  emit threejsMapReceived(out);
}

void RosBridge::onTracks(const std_msgs::String::ConstPtr& msg) {
  bool ok = false;
  QJsonObject o = parseObject(msg, &ok);
  if (!ok) return;

  QVector<HmiTrack3D> out;
  if (o.value("tracks").isArray()) {
    QJsonArray arr = o.value("tracks").toArray();
    out.reserve(arr.size());
    for (const QJsonValue& v : arr) {
      if (!v.isObject()) continue;
      QJsonObject t = v.toObject();
      HmiTrack3D ht;
      ht.id          = jint(t.value("id"));
      ht.type        = jstr(t.value("type"));
      ht.x           = jdouble(t.value("x"));
      ht.y           = jdouble(t.value("y"));
      ht.vx          = jdouble(t.value("vx"));
      ht.vy          = jdouble(t.value("vy"));
      ht.sizeX       = jdouble(t.value("size_x"));
      ht.sizeY       = jdouble(t.value("size_y"));
      ht.orientation = jdouble(t.value("orientation"));
      ht.confidence  = jdouble(t.value("confidence"));
      if (t.value("points").isArray()) {
        QJsonArray pa = t.value("points").toArray();
        ht.points.reserve(pa.size());
        for (const QJsonValue& pv : pa) {
          ht.points.append(static_cast<float>(jdouble(pv)));
        }
      }
      out.push_back(std::move(ht));
    }
  }
  emit tracksChanged(out);
}

void RosBridge::onLocal(
    const mmc_msgs::to_control_team_from_local_msg::ConstPtr& msg) {
  emit egoPoseChanged(msg->host_east, msg->host_north, msg->host_yaw);
}
