// qt_hmi/src/widgets/TrafficLightWidget.cpp
//
// Three-lamp traffic light card with countdown. Lamps are drawn as filled
// circles on a dark vertical strip — matches the visual language used by
// `web_hmi/web/f1/F1HMI.jsx` TrafficLight component (which also drives the
// f1widgets::TrafficLight at left-panel section 03). This M4 widget is the
// scene-side V2X status card that lives at the top of the right ControlPanel.
#include "qt_hmi/widgets/TrafficLightWidget.h"

#include <algorithm>

#include <QtCore/QString>
#include <QtGui/QFont>
#include <QtGui/QFontMetrics>
#include <QtGui/QPainter>
#include <QtGui/QPainterPath>

#include "qt_hmi/style/F1Tokens.h"

namespace qt_hmi_widgets {

TrafficLightWidget::TrafficLightWidget(QWidget* parent) : QWidget(parent) {
  setAutoFillBackground(false);
  setAttribute(Qt::WA_OpaquePaintEvent, false);
}

void TrafficLightWidget::setTraffic(int color, int timeDecisec,
                                    int intersectionId, int signalGroupId) {
  bool changed = false;

  Phase newPhase;
  switch (color) {
    case 1:  newPhase = GREEN; break;
    case 2:  newPhase = AMBER; break;
    case 3:  newPhase = RED;   break;
    default: newPhase = OFF;   break;
  }
  if (newPhase != phase_) { phase_ = newPhase; changed = true; }

  int sec = (timeDecisec >= 0)
                ? static_cast<int>((timeDecisec + 5) / 10)   // round
                : 0;
  if (sec != remainSeconds_) { remainSeconds_ = sec; changed = true; }

  if (intersectionId != intersectionId_) {
    intersectionId_ = intersectionId; changed = true;
  }
  if (signalGroupId != signalGroupId_) {
    signalGroupId_ = signalGroupId; changed = true;
  }
  if (changed) update();
}

void TrafficLightWidget::setPhaseColor(int color) {
  Phase p;
  switch (color) {
    case 1:  p = GREEN; break;
    case 2:  p = AMBER; break;
    case 3:  p = RED;   break;
    default: p = OFF;   break;
  }
  if (p != phase_) { phase_ = p; update(); }
}

void TrafficLightWidget::setRemainSeconds(int sec) {
  if (sec < 0) sec = 0;
  if (sec != remainSeconds_) { remainSeconds_ = sec; update(); }
}

QColor TrafficLightWidget::lampOnColor(Phase p) {
  using namespace f1tokens;
  switch (p) {
    case GREEN: return green;
    case AMBER: return amber;
    case RED:   return red;
    default:    return text3;
  }
}

void TrafficLightWidget::paintEvent(QPaintEvent* /*e*/) {
  using namespace f1tokens;

  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  const int W = width();
  const int H = height();

  // Card background — bg2 with hairline border (ControlPanel sections style).
  p.setPen(QPen(line, 1));
  p.setBrush(bg2);
  p.drawRect(0, 0, W - 1, H - 1);

  // Header strip ("V2X · TRAFFIC").
  const int headerH = 18;
  p.setPen(Qt::NoPen);
  p.setBrush(bg1);
  p.drawRect(0, 0, W, headerH);
  p.setPen(text2);
  QFont hf;
  hf.setFamily(monoFamily());
  hf.setPixelSize(9);
  hf.setLetterSpacing(QFont::AbsoluteSpacing, 1.5);
  p.setFont(hf);
  p.drawText(QRect(8, 0, W - 16, headerH),
             Qt::AlignVCenter | Qt::AlignLeft,
             QStringLiteral("V2X · TRAFFIC"));

  // Phase tag in header right (RED / AMBER / GREEN / —).
  QString phaseTag;
  QColor  phaseColor = text3;
  switch (phase_) {
    case GREEN: phaseTag = QStringLiteral("GREEN"); phaseColor = green; break;
    case AMBER: phaseTag = QStringLiteral("AMBER"); phaseColor = amber; break;
    case RED:   phaseTag = QStringLiteral("RED");   phaseColor = red;   break;
    default:    phaseTag = QStringLiteral("—");     phaseColor = text3; break;
  }
  p.setPen(phaseColor);
  p.drawText(QRect(8, 0, W - 16, headerH),
             Qt::AlignVCenter | Qt::AlignRight, phaseTag);

  // Lamp column geometry — three circles stacked vertically on the left.
  const int padTop  = headerH + 10;
  const int padLeft = 16;
  const int lampD   = std::max(14, std::min((H - padTop - 14) / 3 - 6, 26));
  const int gap     = 6;

  struct Lamp { Phase p; QColor c; int yOff; };
  Lamp lamps[3] = {
      { RED,   red,   0                  },
      { AMBER, amber, lampD + gap        },
      { GREEN, green, 2 * (lampD + gap)  },
  };

  for (int i = 0; i < 3; ++i) {
    int cx = padLeft + lampD / 2;
    int cy = padTop + lamps[i].yOff + lampD / 2;
    bool on = (lamps[i].p == phase_);

    // Outer dim disk (always shown).
    p.setPen(QPen(line, 1));
    p.setBrush(with_alpha(lamps[i].c, on ? 230 : 38));
    p.drawEllipse(QPoint(cx, cy), lampD / 2, lampD / 2);

    // Inner highlight when on (tiny offset to give it a "lit" look).
    if (on) {
      p.setPen(Qt::NoPen);
      QColor hi = lamps[i].c;
      hi.setAlpha(70);
      int r2 = std::max(3, lampD / 4);
      p.setBrush(hi);
      p.drawEllipse(QPoint(cx - 1, cy - 1), r2, r2);
    }
  }

  // Right-side block: countdown number + "CHANGE IN s".
  int rightX = padLeft + lampD + 18;
  int rightW = W - rightX - 12;

  if (rightW > 32) {
    QFont labelF;
    labelF.setFamily(monoFamily());
    labelF.setPixelSize(9);
    labelF.setLetterSpacing(QFont::AbsoluteSpacing, 1.5);
    p.setFont(labelF);
    p.setPen(text3);
    p.drawText(QRect(rightX, padTop, rightW, 12),
               Qt::AlignLeft | Qt::AlignVCenter,
               QStringLiteral("CHANGE IN"));

    QFont numF;
    numF.setFamily(monoFamily());
    numF.setPixelSize(34);
    numF.setBold(true);
    p.setFont(numF);
    QString numStr;
    if (phase_ == OFF || remainSeconds_ <= 0) {
      numStr = QStringLiteral("—");
      p.setPen(text3);
    } else {
      numStr = QString::number(remainSeconds_);
      p.setPen(text0);
    }
    p.drawText(QRect(rightX, padTop + 12, rightW, 40),
               Qt::AlignLeft | Qt::AlignTop, numStr);

    // Suffix "s" right next to the number (small).
    if (phase_ != OFF && remainSeconds_ > 0) {
      QFontMetrics fm(numF);
      int numW = fm.horizontalAdvance(numStr);
      QFont sufF;
      sufF.setFamily(monoFamily());
      sufF.setPixelSize(12);
      p.setFont(sufF);
      p.setPen(text2);
      p.drawText(rightX + numW + 4, padTop + 12 + 30,
                 QStringLiteral("s"));
    }

    // Footer line: intersection / signal group identifiers.
    QFont footF;
    footF.setFamily(monoFamily());
    footF.setPixelSize(9);
    footF.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
    p.setFont(footF);
    p.setPen(text3);
    QString footStr;
    if (intersectionId_ > 0 || signalGroupId_ > 0) {
      footStr = QStringLiteral("INT %1 · SG %2")
                    .arg(intersectionId_).arg(signalGroupId_);
    } else {
      footStr = QStringLiteral("INT — · SG —");
    }
    p.drawText(QRect(rightX, H - 16, rightW, 12),
               Qt::AlignLeft | Qt::AlignVCenter, footStr);
  }
}

}  // namespace qt_hmi_widgets
