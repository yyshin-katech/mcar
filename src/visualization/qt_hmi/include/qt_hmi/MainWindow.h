// qt_hmi/MainWindow.h
//
// M4: hosts the F1Dashboard (M2/M3 chrome + MapScene) as central widget and
// the ControlPanel (V2X TrafficLightWidget + Display/Camera/Layer controls)
// as a right QDockWidget. Wires RosBridge signals to dashboard, scene, and
// traffic-light widgets.
#pragma once

#include <memory>

#include <QtWidgets/QMainWindow>

#include "qt_hmi/HmiTypes.h"
#include "qt_hmi/RosBridge.h"

namespace f1widgets { class F1Dashboard; }
namespace qt_hmi_widgets { class ControlPanel; }

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  // Takes ownership of `bridge` (must outlive MainWindow).
  // Passing the bridge in (instead of constructing inside) lets main.cpp
  // arrange ordering of QApplication::aboutToQuit -> bridge->stop().
  explicit MainWindow(RosBridge* bridge, QWidget* parent = nullptr);
  ~MainWindow() override;

 private:
  RosBridge* bridge_;          // not owned (lifetime managed in main.cpp)
  f1widgets::F1Dashboard*       dashboard_    = nullptr;
  qt_hmi_widgets::ControlPanel* controlPanel_ = nullptr;
};
