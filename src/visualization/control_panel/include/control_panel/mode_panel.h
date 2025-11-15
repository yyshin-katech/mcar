#ifndef MODE_PANEL_H
#define MODE_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/UInt8.h>
#include <mmc_msgs/chassis_msg.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>

namespace control_panel
{

class ModePanel: public rviz::Panel
{
Q_OBJECT
public:
  ModePanel(QWidget* parent = 0);
  virtual ~ModePanel();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected Q_SLOTS:
  void onModeButtonClicked();
  void updateDisplay();

protected:
  void chassisCallback(const mmc_msgs::chassis_msg::ConstPtr& msg);
  
  QPushButton* mode_button_;
  QLabel* status_label_;
  QTimer* update_timer_;
  
  ros::Publisher mode_pub_;
  ros::Subscriber chassis_sub_;
  
  uint8_t current_eps_status_;
  bool is_auto_mode_;
  
  ros::NodeHandle nh_;
};

} // end namespace control_panel

#endif // MODE_PANEL_H