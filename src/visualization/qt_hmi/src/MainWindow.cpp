// qt_hmi/src/MainWindow.cpp
//
// M4: F1Dashboard hosts a MapScene QOpenGLWidget in its main area; a right
// QDockWidget hosts the ControlPanel (V2X TrafficLightWidget + display /
// camera / 13 layer toggles). Wires RosBridge → dashboard, scene, and
// traffic-light slots.
#include "qt_hmi/MainWindow.h"

#include <QtWidgets/QDockWidget>
#include <QtWidgets/QWidget>

#include "qt_hmi/widgets/ControlPanel.h"
#include "qt_hmi/widgets/F1Dashboard.h"
#include "qt_hmi/widgets/MapScene.h"
#include "qt_hmi/widgets/TrafficLightWidget.h"
#include "qt_hmi/style/F1Tokens.h"

MainWindow::MainWindow(RosBridge* bridge, QWidget* parent)
    : QMainWindow(parent), bridge_(bridge) {
  setWindowTitle(QStringLiteral(
      "qt_hmi (M4: F1 Dashboard + MapScene + ControlPanel)"));
  // Default size matches JSX F1HMIShell viewport (1600×900) but resizable.
  // ControlPanel adds ~320 px on the right; expand the default footprint
  // to keep the F1 chrome visible at startup.
  resize(1920, 900);

  // Central widget = F1Dashboard
  dashboard_ = new f1widgets::F1Dashboard(this);
  setCentralWidget(dashboard_);

  // Construct MapScene and inject into dashboard's main stage. The dashboard
  // takes ownership via Qt parent reparenting inside installMapScene().
  qt_hmi_widgets::MapScene* scene = new qt_hmi_widgets::MapScene(dashboard_);
  dashboard_->installMapScene(scene);

  // ── M4: ControlPanel right dock ───────────────────────────────────
  // Hosts the V2X TrafficLightWidget + Display / Camera / Map-layers
  // groups. ControlPanel signals → MapScene slots. RosBridge::trafficChanged
  // → ControlPanel::trafficLight()->setTraffic.
  controlPanel_ = new qt_hmi_widgets::ControlPanel(this);
  QDockWidget* dock = new QDockWidget(QStringLiteral("CONTROL"), this);
  dock->setObjectName(QStringLiteral("controlPanelDock"));
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock->setFeatures(QDockWidget::DockWidgetMovable |
                    QDockWidget::DockWidgetFloatable);
  dock->setWidget(controlPanel_);
  dock->setStyleSheet(QStringLiteral(
      "QDockWidget { color:%1; font-family:'JetBrains Mono',monospace; "
      "  font-size:10px; letter-spacing:2px; } "
      "QDockWidget::title { background:%2; padding:4px 8px; }")
      .arg(f1tokens::cyan.name(), f1tokens::bg1.name()));
  addDockWidget(Qt::RightDockWidgetArea, dock);

  // Background colour: bg0
  setStyleSheet(QStringLiteral("QMainWindow{background:%1;}")
                    .arg(f1tokens::bg0.name()));

  // Wire RosBridge signals → F1Dashboard slots.
  // AutoConnection (default) decays to QueuedConnection because RosBridge
  // signals are emitted from spinner threads while dashboard slots live on
  // the GUI thread. (See design §2.A.)
  connect(bridge_, &RosBridge::stateChanged,
          dashboard_, &f1widgets::F1Dashboard::onState);
  connect(bridge_, &RosBridge::diagChanged,
          dashboard_, &f1widgets::F1Dashboard::onDiag);
  connect(bridge_, &RosBridge::hzChanged,
          dashboard_, &f1widgets::F1Dashboard::onHz);
  connect(bridge_, &RosBridge::trafficChanged,
          dashboard_, &f1widgets::F1Dashboard::onTraffic);
  connect(bridge_, &RosBridge::popupChanged,
          dashboard_, &f1widgets::F1Dashboard::onPopup);
  connect(bridge_, &RosBridge::bagChanged,
          dashboard_, &f1widgets::F1Dashboard::onBag);

  // M3: wire RosBridge → MapScene. AutoConnection across thread boundary
  // decays to QueuedConnection (RosBridge signals fire on spinner threads).
  connect(bridge_, &RosBridge::threejsMapReceived,
          scene, &qt_hmi_widgets::MapScene::onMapReceived);
  connect(bridge_, &RosBridge::tracksChanged,
          scene, &qt_hmi_widgets::MapScene::onTracksChanged);
  connect(bridge_, &RosBridge::stateChanged,
          scene, &qt_hmi_widgets::MapScene::onStateChanged);
  // 50 Hz ego pose direct from /localization/to_control_team for rviz-grade
  // camera follow (web_hmi_bridge throttles ego inside /hmi/state to 10 Hz).
  connect(bridge_, &RosBridge::egoPoseChanged,
          scene, &qt_hmi_widgets::MapScene::onEgoPoseChanged);

  // ── M4: ControlPanel ↔ MapScene ──────────────────────────────────
  connect(controlPanel_,
          &qt_hmi_widgets::ControlPanel::layerVisibilityChanged,
          scene, &qt_hmi_widgets::MapScene::setLayerVisibility);
  connect(controlPanel_,
          &qt_hmi_widgets::ControlPanel::showBoxesChanged,
          scene, &qt_hmi_widgets::MapScene::setShowBoxes);
  connect(controlPanel_,
          &qt_hmi_widgets::ControlPanel::showHeadingChanged,
          scene, &qt_hmi_widgets::MapScene::setShowHeading);
  connect(controlPanel_,
          &qt_hmi_widgets::ControlPanel::showCloudsChanged,
          scene, &qt_hmi_widgets::MapScene::setShowClouds);
  connect(controlPanel_,
          &qt_hmi_widgets::ControlPanel::pointSizeChanged,
          scene, &qt_hmi_widgets::MapScene::setPointSize);
  connect(controlPanel_,
          &qt_hmi_widgets::ControlPanel::cameraModeChanged,
          scene, &qt_hmi_widgets::MapScene::setCameraMode);

  // ── M4: RosBridge::trafficChanged → ControlPanel TrafficLightWidget ──
  if (auto* tl = controlPanel_->trafficLight()) {
    connect(bridge_, &RosBridge::trafficChanged,
            tl, &qt_hmi_widgets::TrafficLightWidget::setTraffic);
  }

  // Push initial defaults to MapScene so DEFAULT_LAYER_VIS / iso camera /
  // 0.08 m point size match between the panel UI and the scene without
  // requiring the operator to touch any widget.
  controlPanel_->emitInitialState();

  // Bootstrap: assume connected at startup. RosBridge could grow a
  // rosConnectionChanged signal in a follow-up M-step, but for M2 the
  // dashboard's own tick() distinguishes ONLINE/WAITING via Hz table alone.
  dashboard_->onConnectionChanged(true, 0);
}

MainWindow::~MainWindow() = default;
