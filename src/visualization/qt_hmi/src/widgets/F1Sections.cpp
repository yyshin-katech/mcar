// qt_hmi/src/widgets/F1Sections.cpp
//
// QPainter implementations for the seven F1HMI widgets declared in
// `include/qt_hmi/widgets/F1Sections.h`. Geometry constants mirror the SVG
// math from `web_hmi/web/f1/F1HMI.jsx` so visual parity is byte-aligned with
// the existing web HMI.
#include "qt_hmi/widgets/F1Sections.h"

#include <algorithm>
#include <cmath>

#include <QtCore/QtMath>
#include <QtGui/QFont>
#include <QtGui/QFontMetrics>
#include <QtGui/QPainter>
#include <QtGui/QPainterPath>
#include <QtGui/QPaintEvent>
#include <QtGui/QPen>

#include "qt_hmi/style/F1Tokens.h"

namespace f1widgets {

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

QFont monoFont(int pxSize, int weight = QFont::Normal) {
  QFont f;
  // Qt 5.12 lacks setFamilies(); use comma-separated single-string fallback
  // chain. setStyleHint(Monospace) ensures a generic mono is used if none of
  // the listed families resolves.
  f.setFamily(QStringLiteral("JetBrains Mono, DejaVu Sans Mono, "
                              "Liberation Mono, Monospace"));
  f.setStyleHint(QFont::Monospace, QFont::PreferDefault);
  f.setPixelSize(pxSize);
  f.setWeight(weight);
  return f;
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────
// 1. SpeedHalf
// ────────────────────────────────────────────────────────────────────────

SpeedHalf::SpeedHalf(QWidget* parent) : QWidget(parent) {
  setAttribute(Qt::WA_OpaquePaintEvent, false);
  setMinimumSize(minimumSizeHint());
}

void SpeedHalf::setValues(double speedKmh, double limitKmh) {
  // Coerce to non-negative; limit==0 means "no limit known".
  double s = std::max(0.0, speedKmh);
  double l = std::max(0.0, limitKmh);
  if (qFuzzyCompare(s + 1.0, speed_ + 1.0) &&
      qFuzzyCompare(l + 1.0, limit_ + 1.0)) {
    return;
  }
  speed_ = s;
  limit_ = l;
  update();
}

void SpeedHalf::paintEvent(QPaintEvent*) {
  using namespace f1tokens;

  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setRenderHint(QPainter::TextAntialiasing, true);

  // Logical canvas matches JSX viewBox 200×130 — scale to fit while keeping
  // aspect ratio.
  const double designW = 200.0, designH = 130.0;
  const double scale   = std::min(width()  / designW,
                                  height() / designH);
  if (scale <= 0.0) return;
  const double offX = (width()  - designW * scale) * 0.5;
  const double offY = (height() - designH * scale) * 0.5;
  p.translate(offX, offY);
  p.scale(scale, scale);

  const double cx = designW / 2.0;
  const double cy = 110.0;
  const double r  = 80.0;

  const double t = std::clamp(speed_ / max_, 0.0, 1.0);

  // Background arc (180° → 360°)
  QRectF arcRect(cx - r, cy - r, 2 * r, 2 * r);
  p.setPen(QPen(line, 2.0, Qt::SolidLine, Qt::FlatCap));
  // Qt drawArc uses 1/16th of degrees, with 0° at 3 o'clock and counter-CW.
  // SVG has 0° at 3 o'clock and CW, so the JSX 180→360 (i.e. CW from 9
  // o'clock to 3 o'clock through bottom — wait, JSX large-arc-flag=0,
  // sweep-flag=1, sweeping CW from 180° to 360° means it traces the BOTTOM
  // semicircle. But the JSX text values fall on the TOP half (cy=110, ticks
  // at 180+(180*i/max) which is bottom half mathematically). SVG y axis
  // points DOWN, so y=cy + r*sin(180°)=cy+0 (left), sin(270°)=cy-r (top),
  // sin(360°)=cy (right). Thus the arc actually traces the TOP half of the
  // circle through y < cy.) For Qt, with y-down and 0°=East CCW=positive,
  // we want 0..180° (angle 0 → 180 CCW = top half from East to West).
  p.drawArc(arcRect, 0 * 16, 180 * 16);

  // Active arc — same 180° span, scaled by t. Sweep CCW from 180° → 180-180*t.
  if (t > 0.0) {
    QPen activePen(cyan, 2.5, Qt::SolidLine, Qt::FlatCap);
    p.setPen(activePen);
    // CCW sweep span = 180*t degrees from start angle 180° going negative.
    p.drawArc(arcRect, 180 * 16, -static_cast<int>(180.0 * t * 16.0));
  }

  // Ticks every 10 km/h, major every 20.
  const QPen majorTickPast(cyan,    1.4);
  const QPen majorTickPend(line,    1.4);
  const QPen minorTickPast(cyan,    0.8);
  const QPen minorTickPend(line,    0.8);
  QFont labelFont = monoFont(8);
  p.setFont(labelFont);
  for (int i = 0; i <= static_cast<int>(max_); i += 10) {
    // Match JSX angle: a = 180 + (180*i/max), with SVG axes (y-down, 0°=East
    // CW). On Qt (also y-down) that maps to standard polar with CCW positive
    // by negating the angle. Use radians directly with the JSX formula —
    // the math is identical because we use cos/sin of the angle in degrees.
    double aDeg = 180.0 + (180.0 * i) / max_;
    double aRad = aDeg * kDeg2Rad;
    bool major = (i % 20 == 0);
    double inner = r - (major ? 12.0 : 6.0);
    double x1 = cx + inner * std::cos(aRad);
    double y1 = cy + inner * std::sin(aRad);
    double x2 = cx + r     * std::cos(aRad);
    double y2 = cy + r     * std::sin(aRad);
    bool past = (static_cast<double>(i) / max_) <= t;
    p.setPen(major ? (past ? majorTickPast : majorTickPend)
                   : (past ? minorTickPast : minorTickPend));
    p.drawLine(QPointF(x1, y1), QPointF(x2, y2));

    if (major) {
      double tx = cx + (r - 22.0) * std::cos(aRad);
      double ty = cy + (r - 22.0) * std::sin(aRad) + 3.0;
      p.setPen(past ? cyan : text3);
      QString lbl = QString::number(i);
      QFontMetricsF fm(labelFont);
      double tw = fm.horizontalAdvance(lbl);
      p.drawText(QPointF(tx - tw / 2.0, ty), lbl);
    }
  }

  // Limit marker (amber dot) — only when 0 < limit ≤ max.
  if (limit_ > 0.0 && limit_ <= max_) {
    double la = (180.0 + (180.0 * limit_) / max_) * kDeg2Rad;
    double lx = cx + (r + 6.0) * std::cos(la);
    double ly = cy + (r + 6.0) * std::sin(la);
    p.setPen(Qt::NoPen);
    p.setBrush(amber);
    p.drawEllipse(QPointF(lx, ly), 3.0, 3.0);
  }

  // Center numeric (big, tabular-aligned)
  QFont bigFont = monoFont(40, QFont::Medium);
  p.setFont(bigFont);
  p.setPen(text0);
  QString val = QString::number(static_cast<int>(std::round(speed_)));
  QFontMetricsF fmBig(bigFont);
  double bw = fmBig.horizontalAdvance(val);
  p.drawText(QPointF(cx - bw / 2.0, 92.0), val);

  // "KM / H" label
  QFont smallFont = monoFont(9);
  p.setFont(smallFont);
  p.setPen(text2);
  QString unit = QStringLiteral("KM / H");
  QFontMetricsF fmS(smallFont);
  double uw = fmS.horizontalAdvance(unit);
  p.drawText(QPointF(cx - uw / 2.0, 108.0), unit);

  // "LIMIT NN" amber subline
  QFont limFont = monoFont(8);
  p.setFont(limFont);
  p.setPen(amber);
  QString lim = limit_ > 0.0
      ? QStringLiteral("LIMIT %1").arg(static_cast<int>(std::round(limit_)))
      : QStringLiteral("LIMIT —");
  QFontMetricsF fmL(limFont);
  double lw = fmL.horizontalAdvance(lim);
  p.drawText(QPointF(cx - lw / 2.0, 125.0), lim);
}

// ────────────────────────────────────────────────────────────────────────
// 2. SteerDial
// ────────────────────────────────────────────────────────────────────────

SteerDial::SteerDial(QWidget* parent) : QWidget(parent) {
  setMinimumSize(minimumSizeHint());
}

void SteerDial::setAngle(double deg) {
  if (qFuzzyCompare(deg + 1.0, angleDeg_ + 1.0)) return;
  angleDeg_ = deg;
  update();
}

void SteerDial::setGear(const QString& letter) {
  if (letter == gear_) return;
  gear_ = letter;
  update();
}

void SteerDial::paintEvent(QPaintEvent*) {
  using namespace f1tokens;
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  const double designW = 80.0, designH = 88.0;
  const double scale = std::min(width() / designW, height() / designH);
  if (scale <= 0.0) return;
  const double offX = (width()  - designW * scale) * 0.5;
  const double offY = (height() - designH * scale) * 0.5;
  p.translate(offX, offY);
  p.scale(scale, scale);

  const double cx = designW / 2.0;
  const double cy = 40.0;
  const double r  = 28.0;

  // Outer ring
  p.setPen(QPen(line, 1.2));
  p.setBrush(Qt::NoBrush);
  p.drawEllipse(QPointF(cx, cy), r, r);
  p.setPen(QPen(line, 0.7));
  p.drawEllipse(QPointF(cx, cy), r - 5.0, r - 5.0);

  // Rotated indicator group (cyan needle + horizontal cross + center hub)
  p.save();
  p.translate(cx, cy);
  p.rotate(angleDeg_);  // Qt rotation is CW for positive angles, matches SVG.
  p.setPen(QPen(cyan, 1.6, Qt::SolidLine, Qt::RoundCap));
  p.drawLine(QPointF(0, -r + 3), QPointF(0, -r - 5));
  p.setPen(Qt::NoPen);
  p.setBrush(cyan);
  p.drawEllipse(QPointF(0, 0), 2.4, 2.4);
  QPen crossPen(cyan, 1.0);
  // Half-opacity cross arms (web JSX opacity 0.5)
  QColor cyanHalf = cyan;
  cyanHalf.setAlphaF(0.5);
  crossPen.setColor(cyanHalf);
  p.setPen(crossPen);
  p.drawLine(QPointF(-r + 3, 0), QPointF(r - 3, 0));
  p.restore();

  // GEAR <letter>
  QFont gFont = monoFont(8);
  p.setFont(gFont);
  QFontMetricsF fm(gFont);
  QString prefix = QStringLiteral("GEAR ");
  double pw = fm.horizontalAdvance(prefix);
  double gw = fm.horizontalAdvance(gear_);
  double total = pw + gw;
  double startX = cx - total / 2.0;
  p.setPen(text3);
  p.drawText(QPointF(startX, 82.0), prefix);
  p.setPen(text0);
  p.drawText(QPointF(startX + pw, 82.0), gear_);
}

// ────────────────────────────────────────────────────────────────────────
// 3. TrafficLight
// ────────────────────────────────────────────────────────────────────────

TrafficLight::TrafficLight(QWidget* parent) : QWidget(parent) {
  setMinimumSize(minimumSizeHint());
}

void TrafficLight::setPhaseColor(int color) {
  Phase np;
  switch (color) {
    case 1:  np = GREEN; break;
    case 2:  np = AMBER; break;
    case 3:  np = RED;   break;
    default: np = OFF;   break;
  }
  if (np == phase_) return;
  phase_ = np;
  update();
}

void TrafficLight::setRemainSeconds(int sec) {
  if (sec == remain_) return;
  remain_ = sec;
  update();
}

void TrafficLight::paintEvent(QPaintEvent*) {
  using namespace f1tokens;
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  // Layout: left LED stack 38×90, right text block.
  const double designW = 150.0, designH = 95.0;
  const double scale = std::min(width() / designW, height() / designH);
  if (scale <= 0.0) return;
  const double offX = (width()  - designW * scale) * 0.5;
  const double offY = (height() - designH * scale) * 0.5;
  p.translate(offX, offY);
  p.scale(scale, scale);

  // LED column
  const double colW = 38.0, colH = 90.0;
  const double colX = 0.0,  colY = 2.0;
  QRectF colRect(colX, colY, colW, colH);
  p.setPen(QPen(line, 1.0));
  p.setBrush(bg0);
  p.drawRoundedRect(colRect, 5.0, 5.0);

  const QColor litColor = (phase_ == RED)   ? red
                        : (phase_ == AMBER) ? amber
                        : (phase_ == GREEN) ? green
                        : QColor();
  const Phase order[3] = {RED, AMBER, GREEN};
  for (int i = 0; i < 3; ++i) {
    double diaPx = 24.0;
    double cx = colX + colW / 2.0;
    double cy = colY + 4.0 + diaPx / 2.0 + i * (diaPx + 3.0);
    bool on = (order[i] == phase_) && phase_ != OFF;
    QColor body = on ? (order[i] == RED   ? red
                      : order[i] == AMBER ? amber
                      : green)
                     : bg2;
    QColor border = on ? body : line;
    p.setBrush(body);
    p.setPen(QPen(border, 1.0));
    p.drawEllipse(QPointF(cx, cy), diaPx / 2.0, diaPx / 2.0);
    if (on) {
      // Glow halo (drop-shadow approximation)
      QRadialGradient grad(QPointF(cx, cy), diaPx);
      QColor halo = body;
      halo.setAlpha(120);
      grad.setColorAt(0.0, halo);
      halo.setAlpha(0);
      grad.setColorAt(1.0, halo);
      p.save();
      p.setBrush(grad);
      p.setPen(Qt::NoPen);
      p.drawEllipse(QPointF(cx, cy), diaPx, diaPx);
      p.restore();
    }
  }

  // Text block
  const double tx = colW + 8.0;
  QFont labelFont = monoFont(9);
  p.setFont(labelFont);
  p.setPen(text3);
  p.drawText(QPointF(tx, 14.0), QStringLiteral("PHASE"));

  QFont phaseFont = monoFont(13);
  p.setFont(phaseFont);
  QString phaseTxt = (phase_ == RED)   ? QStringLiteral("RED")
                   : (phase_ == AMBER) ? QStringLiteral("AMBER")
                   : (phase_ == GREEN) ? QStringLiteral("GREEN")
                   : QStringLiteral("OFF");
  QColor phaseTxtColor = (phase_ == RED)   ? red
                       : (phase_ == AMBER) ? amber
                       : (phase_ == GREEN) ? green
                       : text3;
  p.setPen(phaseTxtColor);
  p.drawText(QPointF(tx, 30.0), phaseTxt);

  p.setFont(labelFont);
  p.setPen(text3);
  p.drawText(QPointF(tx, 50.0), QStringLiteral("CHANGE IN"));

  QFont remFont = monoFont(18);
  p.setFont(remFont);
  p.setPen(text0);
  QString remStr = QString::number(std::max(0, remain_));
  p.drawText(QPointF(tx, 72.0), remStr);
  QFontMetricsF fmR(remFont);
  double rw = fmR.horizontalAdvance(remStr);
  p.setFont(monoFont(9));
  p.setPen(text3);
  p.drawText(QPointF(tx + rw + 4.0, 72.0), QStringLiteral("SEC"));
}

// ────────────────────────────────────────────────────────────────────────
// 4. Section
// ────────────────────────────────────────────────────────────────────────

Section::Section(const QString& number, const QString& title, QWidget* parent)
    : QWidget(parent), number_(number), title_(title) {
  setMinimumHeight(26);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void Section::setRight(const QString& text) {
  if (text == right_) return;
  right_ = text;
  update();
}

void Section::setRightColor(const QColor& c) {
  if (c == rightColor_) return;
  rightColor_ = c;
  update();
}

void Section::paintEvent(QPaintEvent*) {
  using namespace f1tokens;
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, false);
  p.fillRect(rect(), bg1);
  // Bottom hairline
  p.setPen(QPen(line, 1.0));
  p.drawLine(0, height() - 1, width(), height() - 1);

  QFont f = monoFont(10);
  // letter-spacing 2 ≈ extra spacing 2 px between glyphs. Qt's setLetterSpacing
  // uses absolute spacing for Qt::AbsoluteSpacing.
  f.setLetterSpacing(QFont::AbsoluteSpacing, 2.0);
  p.setFont(f);

  const int padX = 14;
  int y = height() - 8;  // baseline like JSX padding "8px 14px"

  // Number (cyan, bold)
  QFont fNum = f;
  fNum.setWeight(QFont::DemiBold);
  p.setFont(fNum);
  p.setPen(cyan);
  p.drawText(QPoint(padX, y), number_);
  QFontMetricsF fmNum(fNum);
  double nw = fmNum.horizontalAdvance(number_);

  // Title (text2)
  p.setFont(f);
  p.setPen(text2);
  p.drawText(QPoint(padX + static_cast<int>(nw) + 10, y), title_);

  // Right-aligned info
  if (!right_.isEmpty()) {
    QColor c = rightColor_.isValid() ? rightColor_ : text3;
    p.setPen(c);
    QFontMetricsF fmR(f);
    double rw = fmR.horizontalAdvance(right_);
    p.drawText(QPoint(width() - padX - static_cast<int>(rw), y), right_);
  }
}

// ────────────────────────────────────────────────────────────────────────
// 5. Stat
// ────────────────────────────────────────────────────────────────────────

Stat::Stat(const QString& key, QWidget* parent)
    : QWidget(parent), key_(key) {
  setMinimumHeight(20);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

void Stat::setValue(const QString& v) {
  if (v == value_) return;
  value_ = v;
  update();
  updateGeometry();
}

void Stat::paintEvent(QPaintEvent*) {
  using namespace f1tokens;
  QPainter p(this);
  p.setRenderHint(QPainter::TextAntialiasing, true);

  // Key (small, dim, letter-spaced)
  QFont kFont = monoFont(9);
  kFont.setLetterSpacing(QFont::AbsoluteSpacing, 2.0);
  p.setFont(kFont);
  QFontMetricsF kfm(kFont);
  p.setPen(text3);
  int y = height() - 5;
  p.drawText(QPoint(0, y), key_);
  double kw = kfm.horizontalAdvance(key_) + 6.0;

  // Value
  QFont vFont = monoFont(12);
  p.setFont(vFont);
  p.setPen(text0);
  p.drawText(QPoint(static_cast<int>(kw), y), value_);
}

// ────────────────────────────────────────────────────────────────────────
// 6. Dot
// ────────────────────────────────────────────────────────────────────────

Dot::Dot(QWidget* parent) : QWidget(parent) {
  setMinimumSize(minimumSizeHint());
  color_ = f1tokens::text3;
}

void Dot::setColor(const QColor& c) {
  if (c == color_) return;
  color_ = c;
  update();
}

void Dot::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  const double dia = std::min(width(), height()) - 2.0;
  if (dia <= 0.0) return;
  const QPointF c(width() / 2.0, height() / 2.0);
  // Outer glow
  QRadialGradient g(c, dia);
  QColor halo = color_;
  halo.setAlpha(140);
  g.setColorAt(0.0, halo);
  halo.setAlpha(0);
  g.setColorAt(1.0, halo);
  p.setBrush(g);
  p.setPen(Qt::NoPen);
  p.drawEllipse(c, dia, dia);
  // Solid dot
  p.setBrush(color_);
  p.setPen(Qt::NoPen);
  p.drawEllipse(c, dia / 2.0, dia / 2.0);
}

// ────────────────────────────────────────────────────────────────────────
// 7. HealthRow
// ────────────────────────────────────────────────────────────────────────

HealthRow::HealthRow(const QString& label, QWidget* parent)
    : QWidget(parent), label_(label) {
  setMinimumHeight(28);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void HealthRow::setStatus(int code) {
  if (code == code_) return;
  code_ = code;
  update();
}

void HealthRow::setInfo(const QString& info) {
  if (info == info_) return;
  info_ = info;
  update();
}

void HealthRow::paintEvent(QPaintEvent*) {
  using namespace f1tokens;
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setRenderHint(QPainter::TextAntialiasing, true);

  // Bottom hairline divider
  p.setPen(QPen(line, 1.0));
  p.drawLine(0, height() - 1, width(), height() - 1);

  // Status colour & text
  QColor statusColor;
  QString statusText;
  if (code_ >= 2)      { statusColor = red;   statusText = QStringLiteral("ERR"); }
  else if (code_ == 1) { statusColor = amber; statusText = QStringLiteral("WARN"); }
  else                 { statusColor = green; statusText = QStringLiteral("OK"); }

  // Layout: 14px pad | 7px dot | 10px gap | 80px label | gap | info (flex) | status | 14px pad
  const int padX = 14;
  const int dotDia = 7;
  const int labelW = 80;
  const int dotX = padX;
  const int dotY = (height() - dotDia) / 2;

  // Dot with glow
  QPointF dotC(dotX + dotDia / 2.0, dotY + dotDia / 2.0);
  QRadialGradient g(dotC, dotDia * 1.6);
  QColor halo = statusColor;
  halo.setAlpha(140);
  g.setColorAt(0.0, halo);
  halo.setAlpha(0);
  g.setColorAt(1.0, halo);
  p.setBrush(g);
  p.setPen(Qt::NoPen);
  p.drawEllipse(dotC, dotDia * 1.6, dotDia * 1.6);
  p.setBrush(statusColor);
  p.drawEllipse(dotC, dotDia / 2.0, dotDia / 2.0);

  QFont f = monoFont(11);
  f.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
  p.setFont(f);
  QFontMetricsF fm(f);

  int textY = static_cast<int>(height() / 2.0 + fm.ascent() / 2.0 - 2);

  // LABEL
  p.setPen(text1);
  int labelX = dotX + dotDia + 10;
  p.drawText(QPoint(labelX, textY), label_);

  // INFO (left-aligned in middle column)
  p.setPen(text3);
  int infoX = labelX + labelW + 10;
  // Elide if needed
  int rightMargin = padX + static_cast<int>(fm.horizontalAdvance(statusText)) + 10;
  int infoMaxW = std::max(0, width() - infoX - rightMargin);
  QString info = fm.elidedText(info_, Qt::ElideRight, infoMaxW);
  p.drawText(QPoint(infoX, textY), info);

  // STATUS
  p.setPen(statusColor);
  int statusX = width() - padX - static_cast<int>(fm.horizontalAdvance(statusText));
  p.drawText(QPoint(statusX, textY), statusText);
}

}  // namespace f1widgets
