// qt_hmi/widgets/F1Dashboard.h
//
// 1600×900 dashboard chrome, direct port of `F1HMIShell` in
// `web_hmi/web/f1/F1HMI.jsx`.
//
// Composition:
//   ┌─────────────────────────────── TOP BAR (44px) ───────────────────────┐
//   │ KATECH logo · UTC · KST · TICK · NET · ROS badge                      │
//   ├──────────── 320px ────────────┬───────────── main area (flex) ───────┤
//   │ 01 VELOCITY · STEER           │  06 ENVIRONMENT (M2 placeholder, M3) │
//   │   SpeedHalf  SteerDial        │                                       │
//   │ 02 DRIVE MODE  │  03 V2X      │                                       │
//   │   MANUAL/AUTO  │  TrafficLight│                                       │
//   │ 04 LOCALIZATION               │                                       │
//   │   8 stats grid + RTK label    │                                       │
//   │ 05 SYSTEM HEALTH              │                                       │
//   │   6 HealthRow + 4 Dot summary │                                       │
//   ├───────────────────────────────┴ BOTTOM TELEMETRY (56px) ──────────────┤
//   │ EGO-VEL · Δ-LIM · LATERAL · JERK · LEAD-D · LEAD-Δv · PLAN-H · CPU    │
//   └────────────────────────────────────────────────────────────────────────┘
//
// All values arrive via RosBridge signals — wired by MainWindow when it
// constructs the dashboard. The dashboard does not subscribe to ROS; it
// only owns the visual state machine.
//
// See claude_work_list/hmi_design/01_design.md §5/M2.
#pragma once

#include <QtCore/QHash>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QElapsedTimer>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

#include "qt_hmi/HmiTypes.h"

namespace qt_hmi_widgets { class MapScene; }

namespace f1widgets {

class SpeedHalf;
class SteerDial;
class TrafficLight;
class Section;
class Stat;
class Dot;
class HealthRow;

class F1Dashboard : public QWidget {
  Q_OBJECT

 public:
  explicit F1Dashboard(QWidget* parent = nullptr);
  ~F1Dashboard() override;

  // Inject the MapScene widget into the main-area stage. Called by
  // MainWindow (which also wires its slots to RosBridge). The dashboard
  // takes ownership via Qt parent-child once installed. Replaces the
  // M2 placeholder QLabel.
  void installMapScene(qt_hmi_widgets::MapScene* scene);
  qt_hmi_widgets::MapScene* mapScene() const { return mapScene_; }

 public slots:
  // From RosBridge::stateChanged
  void onState(const HmiState& s);
  // From RosBridge::diagChanged + hzChanged (re-renders both rows when either
  // arrives, latest hz cached internally).
  void onDiag(const QHash<QString, int>& status);
  void onHz(const QHash<QString, double>& hz);
  // From RosBridge::trafficChanged
  void onTraffic(int color, int timeDecisec,
                 int intersectionId, int signalGroupId);
  // From RosBridge::popupChanged — drives ODD overlay text
  void onPopup(const QString& text, const QString& severity);
  // From RosBridge::bagChanged — drives BAG strip visuals (M4 will add click).
  void onBag(bool recording, const QString& info);
  // Connection liveness from main.cpp's clock + RosBridge state
  void onConnectionChanged(bool connected, qint64 lastMsgAgeMs);

 private slots:
  void tick();    // 4 Hz local timer for clock / TICK / NET text

 private:
  // Layout helpers
  QWidget* buildTopBar();
  QWidget* buildLeftPanel();
  QWidget* buildMainArea();
  QWidget* buildBottomStrip();

  // Repaint helpers (cheap — they re-set label text from cached state)
  void refreshLocalization();
  void refreshDriveMode();
  void refreshHealth();
  void refreshBottom();

  // ── Top bar ────────────────────────────────────────────
  Stat*   utcStat_   = nullptr;
  Stat*   kstStat_   = nullptr;
  Stat*   tickStat_  = nullptr;
  Stat*   netStat_   = nullptr;
  QLabel* rosBadge_  = nullptr;

  // ── 01 VELOCITY · STEER ────────────────────────────────
  SpeedHalf* speedHalf_ = nullptr;
  SteerDial* steerDial_ = nullptr;
  QLabel*    steerNumLabel_ = nullptr;       // "+12.3°"
  QLabel*    steerDirLabel_ = nullptr;       // "← LEFT" / "RIGHT →"
  QLabel*    thrLabel_ = nullptr;
  QLabel*    brkLabel_ = nullptr;
  QLabel*    accelLabel_ = nullptr;

  // ── 02 DRIVE MODE ──────────────────────────────────────
  QLabel*    manualLabel_ = nullptr;
  QLabel*    autoLabel_   = nullptr;
  QLabel*    engagedLabel_ = nullptr;
  QLabel*    vMaxLabel_    = nullptr;
  QLabel*    oddLabel_     = nullptr;

  // ── 03 V2X ─────────────────────────────────────────────
  TrafficLight* trafficLight_ = nullptr;

  // ── 04 LOCALIZATION ────────────────────────────────────
  Section*   localSection_ = nullptr;
  QLabel*    laneLabel_ = nullptr;
  QLabel*    linkLabel_ = nullptr;
  QLabel*    sigELabel_ = nullptr;
  QLabel*    sigNLabel_ = nullptr;
  QLabel*    headingLabel_ = nullptr;

  // ── 05 SYSTEM HEALTH ───────────────────────────────────
  // Visual order (matches F1HMIScreen.HEALTH_ROWS)
  // {GPS-RTK, K-ADCU, LIDAR, RADAR, CAMERA, V2X}
  HealthRow* healthRows_[6] = {nullptr};
  Dot*       summaryDots_[4] = {nullptr};

  // ── 06 ENVIRONMENT — MapScene injected by MainWindow ─────────
  QFrame*    mainStage_ = nullptr;        // host for placeholder/MapScene
  QLabel*    mainPlaceholder_ = nullptr;  // hidden once MapScene installed
  qt_hmi_widgets::MapScene* mapScene_ = nullptr;
  QFrame*    oddBanner_ = nullptr;        // amber strip overlay
  QLabel*    oddBannerLabel_ = nullptr;
  QFrame*    bagOverlay_ = nullptr;
  QLabel*    bagInfoLabel_ = nullptr;
  QLabel*    bagStateLabel_ = nullptr;

  // ── Bottom strip ────────────────────────────────────────
  static constexpr int kBottomCells = 8;
  struct BottomCell {
    QLabel* key;
    QLabel* value;
  };
  BottomCell bottom_[kBottomCells];

  // ── Cached state ────────────────────────────────────────
  HmiState lastState_;
  QHash<QString, int>    lastDiag_;
  QHash<QString, double> lastHz_;
  QString  rtkLabel_  = QStringLiteral("NO RTK");
  QColor   rtkColor_;
  bool     rosConnected_ = false;
  qint64   lastMsgAgeMs_ = -1;

  // 4 Hz clock for top bar
  QTimer*        clockTimer_ = nullptr;
  QElapsedTimer  uptime_;
};

}  // namespace f1widgets
