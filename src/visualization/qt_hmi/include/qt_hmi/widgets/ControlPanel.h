// qt_hmi/widgets/ControlPanel.h
//
// M4 right-edge control dock. Hosts (top to bottom):
//
//   1. V2X TrafficLightWidget — current `/siheung_spat` signal phase + countdown.
//   2. Display group           — bbox / heading / clouds toggles + point size.
//   3. Camera group            — iso / top radio.
//   4. Map layers group        — 13 layer checkboxes (11 MOLIT + 2 TB_senario).
//                                Default visibility mirrors `DEFAULT_LAYER_VIS`.
//
// Direct port of `web_hmi/web/threejs/ControlPanel.jsx` (which itself owns
// only DOM state — Three.js scene reads via props). Each user toggle emits a
// Qt signal that MainWindow connects to the matching MapScene slot. The
// TrafficLightWidget at the top is wired by MainWindow as well — RosBridge
// → controlPanel->trafficLight()->setTraffic(...).
//
// Sizing: targets the 800×900 design footprint with a fixed-width dock
// (320 px) that does not collide with the left F1Dashboard panel (also 320).
// The widget is its own QWidget so it can be hosted inside a QDockWidget at
// the QMainWindow right edge — no internal QDockWidget dependency.
//
// See claude_work_list/hmi_design/01_design.md §5/M4.
#pragma once

#include <QtCore/QHash>
#include <QtCore/QString>
#include <QtWidgets/QWidget>

class QButtonGroup;
class QCheckBox;
class QRadioButton;
class QSlider;
class QLabel;

namespace qt_hmi_widgets {

class TrafficLightWidget;

class ControlPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ControlPanel(QWidget* parent = nullptr);
  ~ControlPanel() override;

  // Exposed so MainWindow can wire the RosBridge::trafficChanged signal.
  TrafficLightWidget* trafficLight() const { return traffic_; }

  // Emit current state once on demand (called by MainWindow after wiring so
  // MapScene receives initial visibility / camera mode without a user click).
  void emitInitialState();

 signals:
  // Map layer visibility changes — emitted as a snapshot (full QHash) every
  // time any single checkbox flips. MapScene's setLayerVisibility() takes
  // the same shape, so the receiver can ignore the diff.
  void layerVisibilityChanged(const QHash<QString, bool>& vis);

  // Display options
  void showBoxesChanged(bool on);
  void showHeadingChanged(bool on);
  void showCloudsChanged(bool on);
  void pointSizeChanged(float size);

  // Camera mode — "iso" | "top"
  void cameraModeChanged(const QString& mode);

 private slots:
  void onLayerToggled();
  void onPointSliderMoved(int v);
  void onCameraIso(bool checked);
  void onCameraTop(bool checked);

 private:
  QWidget* buildTrafficSection();
  QWidget* buildDisplayGroup();
  QWidget* buildCameraGroup();
  QWidget* buildLayersGroup();

  // Owned children
  TrafficLightWidget* traffic_       = nullptr;

  QCheckBox*          chkBoxes_      = nullptr;
  QCheckBox*          chkHeading_    = nullptr;
  QCheckBox*          chkClouds_     = nullptr;
  QSlider*            pointSlider_   = nullptr;
  QLabel*             pointValueLbl_ = nullptr;

  QRadioButton*       camIso_        = nullptr;
  QRadioButton*       camTop_        = nullptr;
  QButtonGroup*       camGroup_      = nullptr;

  // 13 layer checkboxes keyed by layer name (LayerStyle names).
  QHash<QString, QCheckBox*> layerCheckboxes_;
};

}  // namespace qt_hmi_widgets
