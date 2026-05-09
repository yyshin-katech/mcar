// qt_hmi/widgets/TrafficLightWidget.h
//
// M4 V2X traffic-light card — three round LEDs (R / Y / G) stacked vertically
// with a numeric countdown beneath them. Mirrors the layout of the f1widgets
// `TrafficLight` already used in F1Dashboard's section 03, but is sized for
// the right-edge ControlPanel header (the ControlPanel hosts it at the top
// of the dock so the operator can watch the signal while toggling layers).
//
// Data source: `RosBridge::trafficChanged(int color, int timeDecisec, ...)`.
// Bridge encodes color as 0=none/off, 1=green, 2=amber, 3=red; matching the
// /siheung_spat MovementPhaseStatus mapping (3=red, 6=green, 8=amber).
//
// All paint logic uses QPainter + QColor — no OpenGL / no external assets.
//
// See claude_work_list/hmi_design/01_design.md §5/M4.
#pragma once

#include <QtCore/QString>
#include <QtGui/QColor>
#include <QtWidgets/QWidget>

namespace qt_hmi_widgets {

class TrafficLightWidget : public QWidget {
  Q_OBJECT

 public:
  enum Phase { OFF = 0, GREEN = 1, AMBER = 2, RED = 3 };

  explicit TrafficLightWidget(QWidget* parent = nullptr);

 public slots:
  // Wired to RosBridge::trafficChanged. Last two args (intersectionId,
  // signalGroupId) carry only context, displayed beneath the countdown.
  void setTraffic(int color, int timeDecisec,
                  int intersectionId, int signalGroupId);

  // Lower-level setters retained for parity with f1widgets::TrafficLight
  // (callable directly from non-RosBridge code if ever needed).
  void setPhaseColor(int color);
  void setRemainSeconds(int sec);

 protected:
  void paintEvent(QPaintEvent* e) override;
  QSize sizeHint() const override        { return QSize(220, 130); }
  QSize minimumSizeHint() const override { return QSize(180, 110); }

 private:
  // Returns the lamp colour; off-state lamps render dim (10% alpha).
  static QColor lampOnColor(Phase p);

  Phase phase_           = OFF;
  int   remainSeconds_   = 0;
  int   intersectionId_  = 0;
  int   signalGroupId_   = 0;
};

}  // namespace qt_hmi_widgets
