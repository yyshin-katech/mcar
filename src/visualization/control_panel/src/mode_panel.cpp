#include "control_panel/mode_panel.h"
#include <pluginlib/class_list_macros.h>

namespace control_panel
{

ModePanel::ModePanel(QWidget* parent)
  : rviz::Panel(parent)
  , current_eps_status_(0)
  , is_auto_mode_(false)
{
  // Publisher: EPS 모드 명령 토픽 (실제 토픽 이름으로 변경 필요)
  mode_pub_ = nh_.advertise<std_msgs::UInt8>("/vehicle/mode_command", 1);
  
  // Subscriber: Chassis 메시지
  chassis_sub_ = nh_.subscribe("/sensors/chassis", 1, 
                               &ModePanel::chassisCallback, this);

  // 메인 레이아웃
  QVBoxLayout* main_layout = new QVBoxLayout;
  
  // 타이틀
  QLabel* title = new QLabel("Driving Mode Control");
  title->setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;");
  title->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(title);
  
  // 모드 전환 버튼
  mode_button_ = new QPushButton("MANUAL");
  mode_button_->setMinimumHeight(80);
  mode_button_->setMinimumWidth(200);
  mode_button_->setStyleSheet(
    "QPushButton {"
    "  background-color: #808080;"  // 회색
    "  color: white;"
    "  font-size: 24px;"
    "  font-weight: bold;"
    "  border: 2px solid #555555;"
    "  border-radius: 10px;"
    "}"
    "QPushButton:hover {"
    "  background-color: #606060;"
    "}"
    "QPushButton:pressed {"
    "  background-color: #404040;"
    "}"
  );
  connect(mode_button_, SIGNAL(clicked()), this, SLOT(onModeButtonClicked()));
  main_layout->addWidget(mode_button_, 0, Qt::AlignCenter);
  
  // 상태 표시 레이블
  status_label_ = new QLabel("Current: MANUAL (EPS Status: 0)");
  status_label_->setStyleSheet(
    "font-size: 14px; "
    "padding: 10px; "
    "background-color: #f0f0f0; "
    "border-radius: 5px;"
  );
  status_label_->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(status_label_);
  
  // 스페이서 추가
  main_layout->addStretch();
  
  setLayout(main_layout);
  
  // 타이머로 주기적 업데이트 (UI 반영용)
  update_timer_ = new QTimer(this);
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(updateDisplay()));
  update_timer_->start(100);  // 100ms마다 업데이트
  
  ROS_INFO("Mode Panel initialized");
}

ModePanel::~ModePanel()
{
}

void ModePanel::chassisCallback(const mmc_msgs::chassis_msg::ConstPtr& msg)
{
  current_eps_status_ = msg->vcu_EPS_Status;
  
  // vcu_EPS_Status가 2이면 AUTO 모드
  is_auto_mode_ = (current_eps_status_ == 2);
}

void ModePanel::updateDisplay()
{
  // 현재 상태에 따라 버튼 업데이트
  if(is_auto_mode_)
  {
    mode_button_->setText("AUTO");
    mode_button_->setStyleSheet(
      "QPushButton {"
      "  background-color: #28a745;"  // 초록색
      "  color: white;"
      "  font-size: 24px;"
      "  font-weight: bold;"
      "  border: 2px solid #1e7e34;"
      "  border-radius: 10px;"
      "}"
      "QPushButton:hover {"
      "  background-color: #218838;"
      "}"
      "QPushButton:pressed {"
      "  background-color: #1e7e34;"
      "}"
    );
  }
  else
  {
    mode_button_->setText("MANUAL");
    mode_button_->setStyleSheet(
      "QPushButton {"
      "  background-color: #808080;"  // 회색
      "  color: white;"
      "  font-size: 24px;"
      "  font-weight: bold;"
      "  border: 2px solid #555555;"
      "  border-radius: 10px;"
      "}"
      "QPushButton:hover {"
      "  background-color: #606060;"
      "}"
      "QPushButton:pressed {"
      "  background-color: #404040;"
      "}"
    );
  }
  
  // 상태 레이블 업데이트
  QString status_text = QString("Current: %1 (EPS Status: %2)")
                        .arg(is_auto_mode_ ? "AUTO" : "MANUAL")
                        .arg(current_eps_status_);
  status_label_->setText(status_text);
}

void ModePanel::onModeButtonClicked()
{
  // 모드 전환 명령 발행
  std_msgs::UInt8 msg;
  
  if(is_auto_mode_)
  {
    // AUTO → MANUAL로 전환 요청
    msg.data = 0;  // MANUAL 모드 값 (실제 값으로 변경 필요)
    ROS_INFO("Mode change requested: AUTO → MANUAL");
  }
  else
  {
    // MANUAL → AUTO로 전환 요청
    msg.data = 2;  // AUTO 모드 값 (실제 값으로 변경 필요)
    ROS_INFO("Mode change requested: MANUAL → AUTO");
  }
  
  mode_pub_.publish(msg);
}

void ModePanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

void ModePanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

} // end namespace control_panel

// RViz 플러그인으로 등록
PLUGINLIB_EXPORT_CLASS(control_panel::ModePanel, rviz::Panel)