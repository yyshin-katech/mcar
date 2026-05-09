// qt_hmi/widgets/F1Sections.h
//
// Seven small QPainter widgets that mirror the JSX components in
// `web_hmi/web/f1/F1HMI.jsx`:
//
//   1. SpeedHalf   — half-circle speedometer with limit marker.
//   2. SteerDial   — round mini steering-wheel + GEAR letter.
//   3. TrafficLight — V2X 3-LED stack + PHASE / CHANGE-IN block.
//   4. Section     — header strip ("01 VELOCITY · STEER  LIVE").
//   5. Stat        — small "KEY value" pair used in the top bar.
//   6. Dot         — colored 7px dot used in HEALTH summary.
//   7. HealthRow   — single row in the SYSTEM HEALTH list.
//
// All widgets are stateless on construction; setters update the cached
// values and call update() to schedule paintEvent. None of them subscribe
// to ROS directly — F1Dashboard wires RosBridge signals to setters.
//
// See claude_work_list/hmi_design/01_design.md §5/M2.
#pragma once

#include <QtCore/QString>
#include <QtGui/QColor>
#include <QtWidgets/QWidget>

namespace f1widgets {

// ── 1. Half-circle speed gauge with numeric center ─────────────────────────
class SpeedHalf : public QWidget {
  Q_OBJECT
 public:
  explicit SpeedHalf(QWidget* parent = nullptr);

 public slots:
  void setValues(double speedKmh, double limitKmh);

 protected:
  void paintEvent(QPaintEvent* e) override;
  QSize sizeHint() const override        { return QSize(200, 130); }
  QSize minimumSizeHint() const override { return QSize(160, 110); }

 private:
  double speed_ = 0.0;
  double limit_ = 0.0;
  double max_   = 120.0;   // km/h, fixed (matches F1HMI.jsx)
};

// ── 2. Mini steering wheel + GEAR letter ───────────────────────────────────
class SteerDial : public QWidget {
  Q_OBJECT
 public:
  explicit SteerDial(QWidget* parent = nullptr);

 public slots:
  void setAngle(double deg);
  void setGear(const QString& letter);

 protected:
  void paintEvent(QPaintEvent* e) override;
  QSize sizeHint() const override        { return QSize(80, 88); }
  QSize minimumSizeHint() const override { return QSize(64, 72); }

 private:
  double  angleDeg_ = 0.0;
  QString gear_     = QStringLiteral("—");
};

// ── 3. V2X traffic light (R/A/G + PHASE + CHANGE-IN) ───────────────────────
class TrafficLight : public QWidget {
  Q_OBJECT
 public:
  enum Phase { OFF = 0, GREEN = 1, AMBER = 2, RED = 3 };

  explicit TrafficLight(QWidget* parent = nullptr);

 public slots:
  // bridge color encoding: 0=none, 1=green, 2=amber, 3=red
  void setPhaseColor(int color);
  void setRemainSeconds(int sec);

 protected:
  void paintEvent(QPaintEvent* e) override;
  QSize sizeHint() const override        { return QSize(150, 95); }
  QSize minimumSizeHint() const override { return QSize(120, 80); }

 private:
  Phase phase_  = OFF;
  int   remain_ = 0;
};

// ── 4. Section header strip ───────────────────────────────────────────────
//   "01  VELOCITY · STEER         LIVE"
//   The right-side text accepts an optional QColor that recolours just the
//   right slot (used by LOCALIZATION → RTK FIX/FLOAT/NO RTK).
class Section : public QWidget {
  Q_OBJECT
 public:
  explicit Section(const QString& number,
                   const QString& title,
                   QWidget* parent = nullptr);

 public slots:
  void setRight(const QString& text);
  void setRightColor(const QColor& c);

 protected:
  void paintEvent(QPaintEvent* e) override;
  QSize sizeHint() const override        { return QSize(280, 26); }
  QSize minimumSizeHint() const override { return QSize(160, 20); }

 private:
  QString number_;
  QString title_;
  QString right_;
  QColor  rightColor_;     // empty/invalid → default text3
};

// ── 5. Stat (top-bar key/value) ───────────────────────────────────────────
class Stat : public QWidget {
  Q_OBJECT
 public:
  explicit Stat(const QString& key, QWidget* parent = nullptr);

 public slots:
  void setValue(const QString& v);

 protected:
  void paintEvent(QPaintEvent* e) override;
  QSize sizeHint() const override        { return QSize(80, 20); }
  QSize minimumSizeHint() const override { return QSize(60, 16); }

 private:
  QString key_;
  QString value_ = QStringLiteral("—");
};

// ── 6. Dot (small status indicator) ───────────────────────────────────────
class Dot : public QWidget {
  Q_OBJECT
 public:
  explicit Dot(QWidget* parent = nullptr);

 public slots:
  void setColor(const QColor& c);

 protected:
  void paintEvent(QPaintEvent* e) override;
  QSize sizeHint() const override        { return QSize(11, 11); }
  QSize minimumSizeHint() const override { return QSize(9, 9); }

 private:
  QColor color_;
};

// ── 7. HealthRow (single SYSTEM HEALTH list entry) ────────────────────────
//   [Dot] [LABEL] [info text — fills middle] [STATUS]
class HealthRow : public QWidget {
  Q_OBJECT
 public:
  explicit HealthRow(const QString& label, QWidget* parent = nullptr);

 public slots:
  // status code: 0=OK, 1=WARN, 2=ERR (drives both dot/status colour and text)
  void setStatus(int code);
  void setInfo(const QString& info);

 protected:
  void paintEvent(QPaintEvent* e) override;
  QSize sizeHint() const override        { return QSize(280, 28); }
  QSize minimumSizeHint() const override { return QSize(220, 24); }

 private:
  QString label_;
  QString info_;
  int     code_ = 0;       // OK by default
};

}  // namespace f1widgets
