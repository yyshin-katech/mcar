// qt_hmi/RosBridge.h
//
// QObject wrapper around 9 std_msgs/String JSON subscribers + 2 publishers
// for the /hmi/* topic surface published by web_hmi bridge.
//
// Threading: ros::AsyncSpinner(2) drives subscriber callbacks on background
// threads. Each callback parses JSON and emits a Qt signal; main-thread
// widget slots receive them via Qt::QueuedConnection (auto cross-thread).
//
// See claude_work_list/hmi_design/01_design.md §2.B.
#pragma once

#include <memory>
#include <vector>

#include <QtCore/QHash>
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QVector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <mmc_msgs/to_control_team_from_local_msg.h>

#include "qt_hmi/HmiTypes.h"

class RosBridge : public QObject {
  Q_OBJECT

 public:
  explicit RosBridge(QObject* parent = nullptr);
  ~RosBridge() override;

  // Stop the AsyncSpinner before ros::shutdown() / process exit.
  // Safe to call multiple times. Should be invoked from QApplication::aboutToQuit
  // to avoid race with destructors.
  void stop();

 public slots:
  // Outbound publishers — must be called from the main (Qt) thread.
  void publishModeRequest(bool autonomous);
  void publishBagToggle();

 signals:
  void stateChanged(const HmiState& s);
  void diagChanged(const QHash<QString, int>& status);
  void hzChanged(const QHash<QString, double>& hz);
  void objectsChanged(const QVector<HmiObject>& objs);
  void popupChanged(const QString& text, const QString& severity);
  void trafficChanged(int color, int timeDecisec,
                      int intersectionId, int signalGroupId);
  void bagChanged(bool recording, const QString& info);
  void mapPolylinesReceived(const QVector<Polyline>& polylines);
  void threejsMapReceived(const HmiMap3D& map);
  void tracksChanged(const QVector<HmiTrack3D>& tracks);
  // 50 Hz ego pose direct from /localization/to_control_team
  // (web_hmi_bridge throttles ego to 10 Hz inside /hmi/state — for rviz-grade
  // camera smoothness we take the unthrottled stream directly).
  void egoPoseChanged(double east, double north, double yaw);

 private:
  // ROS callbacks (bound via subscribe<std_msgs::String>).
  void onState(const std_msgs::String::ConstPtr& msg);
  void onDiag(const std_msgs::String::ConstPtr& msg);
  void onHz(const std_msgs::String::ConstPtr& msg);
  void onObjects(const std_msgs::String::ConstPtr& msg);
  void onPopup(const std_msgs::String::ConstPtr& msg);
  void onTraffic(const std_msgs::String::ConstPtr& msg);
  void onBag(const std_msgs::String::ConstPtr& msg);
  void onMap(const std_msgs::String::ConstPtr& msg);
  void onThreejsMap(const std_msgs::String::ConstPtr& msg);
  void onTracks(const std_msgs::String::ConstPtr& msg);
  void onLocal(const mmc_msgs::to_control_team_from_local_msg::ConstPtr& msg);

  ros::NodeHandle              nh_;
  ros::AsyncSpinner            spinner_;          // 2 threads
  std::vector<ros::Subscriber> subs_;
  ros::Publisher               pubModeReq_;
  ros::Publisher               pubBagToggle_;
  bool                         spinnerRunning_ = false;
};
