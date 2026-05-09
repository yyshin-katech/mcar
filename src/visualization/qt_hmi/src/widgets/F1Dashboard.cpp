// qt_hmi/src/widgets/F1Dashboard.cpp
//
// 1600×900 dashboard composition. Layout structure must mirror the JSX
// `F1HMIShell` so the visual parity is byte-aligned:
//
//   gridTemplateColumns: "320px 1fr"
//   gridTemplateRows:    "44px 1fr 56px"
//   gridTemplateAreas:   "top top" / "left main" / "left bottom"
//
// Every value is driven by a setter slot; nothing here subscribes to ROS.
#include "qt_hmi/widgets/F1Dashboard.h"

#include <algorithm>
#include <cmath>

#include <QtCore/QDateTime>
#include <QtCore/QEvent>
#include <QtCore/QTimeZone>
#include <QtCore/QTimer>
#include <QtCore/QtMath>
#include <QtGui/QPainter>
#include <QtGui/QPalette>
#include <QtGui/QResizeEvent>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSizePolicy>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

#include "qt_hmi/style/F1Tokens.h"
#include "qt_hmi/widgets/F1Sections.h"
#include "qt_hmi/widgets/MapScene.h"

namespace f1widgets {

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;

QString css(const QString& fmt) { return fmt; }

// JSX gear table 1=P, 2=R, 3=N, 4=D
QString gearLetter(int gear) {
  switch (gear) {
    case 1: return QStringLiteral("P");
    case 2: return QStringLiteral("R");
    case 3: return QStringLiteral("N");
    case 4: return QStringLiteral("D");
    default: return QStringLiteral("—");
  }
}

QString fmtKmh(double v) {
  return QString::number(v, 'f', 1) + QStringLiteral(" km/h");
}

QString fmtSigmaCm(double m) {
  if (m <= 0.0) return QStringLiteral("—");
  double cm = m * 100.0;
  // "01.84 cm" — pad integer part to 2 digits.
  return QString::number(cm, 'f', 2).rightJustified(5, QLatin1Char('0')) +
         QStringLiteral(" cm");
}

QString fmtUtc(const QDateTime& dt) {
  QDateTime u = dt.toUTC();
  return u.toString(QStringLiteral("HH:mm:ss"));
}

QString fmtKst(const QDateTime& dt) {
  // KST = UTC+9, fixed offset (no DST in Korea)
  QDateTime k = dt.toUTC().addSecs(9 * 3600);
  return k.toString(QStringLiteral("HH:mm:ss"));
}

// Applies a CSS-like style string to a QLabel — small helper to keep the
// constructor compact. Falls back gracefully when QStyleSheet isn't set.
void styleLabel(QLabel* l, const QString& sheet) {
  if (l) l->setStyleSheet(sheet);
}

}  // namespace

// ───────────────────────────────────────────────────────────────────────
// Construction
// ───────────────────────────────────────────────────────────────────────

F1Dashboard::F1Dashboard(QWidget* parent) : QWidget(parent) {
  using namespace f1tokens;
  rtkColor_ = text3;

  setAutoFillBackground(true);
  QPalette pal = palette();
  pal.setColor(QPalette::Window, bg0);
  pal.setColor(QPalette::WindowText, text0);
  setPalette(pal);

  // Set base font so all child QLabels inherit a sensible default.
  // Qt 5.12 lacks setFamilies(); use comma-separated single-family fallback.
  QFont base;
  base.setFamily(QStringLiteral("Inter, Pretendard, Noto Sans CJK KR, "
                                 "DejaVu Sans, sans-serif"));
  base.setPixelSize(11);
  setFont(base);

  QGridLayout* grid = new QGridLayout(this);
  grid->setContentsMargins(0, 0, 0, 0);
  grid->setHorizontalSpacing(0);
  grid->setVerticalSpacing(0);
  // Columns: 320 fixed, 1 expanding
  grid->setColumnMinimumWidth(0, 320);
  grid->setColumnStretch(0, 0);
  grid->setColumnStretch(1, 1);
  // Rows: 44 fixed (top), expanding (middle), 56 fixed (bottom)
  grid->setRowMinimumHeight(0, 44);
  grid->setRowStretch(0, 0);
  grid->setRowStretch(1, 1);
  grid->setRowMinimumHeight(2, 56);
  grid->setRowStretch(2, 0);

  QWidget* topBar = buildTopBar();
  QWidget* left   = buildLeftPanel();
  QWidget* main   = buildMainArea();
  QWidget* bottom = buildBottomStrip();

  grid->addWidget(topBar, 0, 0, 1, 2);
  grid->addWidget(left,   1, 0, 2, 1);   // spans middle + bottom rows
  grid->addWidget(main,   1, 1);
  grid->addWidget(bottom, 2, 1);

  setMinimumSize(1280, 720);

  // 4 Hz clock for UTC/KST/TICK/NET.
  uptime_.start();
  clockTimer_ = new QTimer(this);
  clockTimer_->setInterval(250);
  connect(clockTimer_, &QTimer::timeout, this, &F1Dashboard::tick);
  clockTimer_->start();
  tick();    // initial paint
}

F1Dashboard::~F1Dashboard() = default;

// ───────────────────────────────────────────────────────────────────────
// MapScene installation (M3) — replaces placeholder QLabel inside main
// stage. Caller (MainWindow) wires RosBridge signals to the scene's slots.
// Overlays (oddBanner_, bagOverlay_) are re-raised so they stay visible
// on top of the OpenGL widget.
// ───────────────────────────────────────────────────────────────────────

void F1Dashboard::installMapScene(qt_hmi_widgets::MapScene* scene) {
  if (!scene || !mainStage_) return;
  // Replace placeholder inside the existing stage layout.
  if (mainPlaceholder_) {
    mainPlaceholder_->hide();
    if (auto* lay = mainStage_->layout()) {
      lay->removeWidget(mainPlaceholder_);
    }
    delete mainPlaceholder_;
    mainPlaceholder_ = nullptr;
  }
  mapScene_ = scene;
  scene->setParent(mainStage_);
  if (auto* lay = qobject_cast<QVBoxLayout*>(mainStage_->layout())) {
    lay->addWidget(scene);
  }
  // Overlays must remain on top of the GL widget.
  if (oddBanner_) oddBanner_->raise();
  if (bagOverlay_) bagOverlay_->raise();
}

// ───────────────────────────────────────────────────────────────────────
// Top bar
// ───────────────────────────────────────────────────────────────────────

QWidget* F1Dashboard::buildTopBar() {
  using namespace f1tokens;

  QFrame* bar = new QFrame(this);
  bar->setFixedHeight(44);
  bar->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; border-bottom:1px solid %2; }")
      .arg(bg1.name(), line.name()));

  QHBoxLayout* row = new QHBoxLayout(bar);
  row->setContentsMargins(18, 0, 18, 0);
  row->setSpacing(22);

  // KATECH logo block (chevron square + text)
  QFrame* logoBox = new QFrame(bar);
  QHBoxLayout* logoRow = new QHBoxLayout(logoBox);
  logoRow->setContentsMargins(0, 0, 0, 0);
  logoRow->setSpacing(10);
  QLabel* k = new QLabel(QStringLiteral("K"), logoBox);
  k->setFixedSize(22, 22);
  k->setAlignment(Qt::AlignCenter);
  k->setStyleSheet(QStringLiteral(
      "background:%1; color:#000; "
      "font-family:'JetBrains Mono','DejaVu Sans Mono',monospace; "
      "font-weight:700; font-size:11px;")
      .arg(cyan.name()));
  logoRow->addWidget(k);
  QLabel* title = new QLabel(QStringLiteral("KATECH · IONIQ EV"), logoBox);
  title->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:12px; letter-spacing:3px;")
      .arg(text1.name()));
  logoRow->addWidget(title);
  QLabel* sub = new QLabel(QStringLiteral("AD/HMI v3.2 · NODE qt_hmi_node"),
                           logoBox);
  sub->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:9px; letter-spacing:1px;")
      .arg(text3.name()));
  logoRow->addWidget(sub);
  row->addWidget(logoBox);

  row->addStretch(1);

  utcStat_  = new Stat(QStringLiteral("UTC"),  bar);
  kstStat_  = new Stat(QStringLiteral("KST"),  bar);
  tickStat_ = new Stat(QStringLiteral("TICK"), bar);
  netStat_  = new Stat(QStringLiteral("NET"),  bar);
  for (Stat* s : {utcStat_, kstStat_, tickStat_, netStat_}) {
    s->setMinimumWidth(110);
    row->addWidget(s);
  }

  rosBadge_ = new QLabel(QStringLiteral("● ROS · OFFLINE"), bar);
  rosBadge_->setStyleSheet(QStringLiteral(
      "color:%1; background:%2; "
      "border:1px solid %3; padding:5px 12px; "
      "font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:2px;")
      .arg(red.name(),
           with_alpha(red, 26).name(QColor::HexArgb),
           with_alpha(red, 102).name(QColor::HexArgb)));
  row->addWidget(rosBadge_);

  return bar;
}

// ───────────────────────────────────────────────────────────────────────
// Left panel — sections 01..05
// ───────────────────────────────────────────────────────────────────────

QWidget* F1Dashboard::buildLeftPanel() {
  using namespace f1tokens;

  QFrame* panel = new QFrame(this);
  panel->setFixedWidth(320);
  panel->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; border-right:1px solid %2; }")
      .arg(bg0.name(), line.name()));

  QVBoxLayout* col = new QVBoxLayout(panel);
  col->setContentsMargins(0, 0, 0, 0);
  col->setSpacing(0);

  // ── 01 VELOCITY · STEER ───────────────────────
  Section* sec01 = new Section(QStringLiteral("01"),
                               QStringLiteral("VELOCITY · STEER"), panel);
  sec01->setRight(QStringLiteral("LIVE"));
  col->addWidget(sec01);

  QFrame* row01 = new QFrame(panel);
  QGridLayout* g01 = new QGridLayout(row01);
  g01->setContentsMargins(14, 12, 14, 12);
  g01->setHorizontalSpacing(10);
  g01->setVerticalSpacing(6);
  speedHalf_ = new SpeedHalf(row01);
  speedHalf_->setMinimumSize(140, 110);
  g01->addWidget(speedHalf_, 0, 0, 2, 1);

  // Right column: STEER label/value + LEFT/RIGHT marker + SteerDial
  QLabel* steerKey = new QLabel(QStringLiteral("STEER"), row01);
  steerKey->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:9px; letter-spacing:1px;")
      .arg(text3.name()));
  g01->addWidget(steerKey, 0, 1, Qt::AlignLeft | Qt::AlignTop);

  steerNumLabel_ = new QLabel(QStringLiteral("—"), row01);
  steerNumLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:14px; font-variant-numeric:tabular-nums;")
      .arg(text0.name()));
  g01->addWidget(steerNumLabel_, 0, 1, Qt::AlignRight | Qt::AlignTop);

  steerDirLabel_ = new QLabel(QStringLiteral("RIGHT →"), row01);
  steerDirLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:8px; letter-spacing:1px;")
      .arg(text3.name()));
  g01->addWidget(steerDirLabel_, 1, 1, Qt::AlignRight | Qt::AlignTop);

  steerDial_ = new SteerDial(row01);
  steerDial_->setFixedSize(80, 88);
  g01->addWidget(steerDial_, 2, 1, Qt::AlignCenter);
  g01->setRowStretch(0, 0);
  g01->setRowStretch(1, 0);
  g01->setRowStretch(2, 1);
  col->addWidget(row01);

  // THR / BRK / ACCEL — placeholder strip
  QFrame* triple = new QFrame(panel);
  QHBoxLayout* tripLay = new QHBoxLayout(triple);
  tripLay->setContentsMargins(0, 0, 0, 0);
  tripLay->setSpacing(1);
  triple->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; border-top:1px solid %1; "
      "         border-bottom:1px solid %1; }")
      .arg(line.name()));
  auto makeCell = [&](const QString& k, QLabel** out) {
    QFrame* f = new QFrame(triple);
    f->setStyleSheet(QStringLiteral("QFrame { background:%1; }").arg(bg1.name()));
    QVBoxLayout* fl = new QVBoxLayout(f);
    fl->setContentsMargins(12, 6, 12, 6);
    fl->setSpacing(2);
    QLabel* keyL = new QLabel(k, f);
    keyL->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:9px; letter-spacing:2px;")
        .arg(text3.name()));
    QLabel* valL = new QLabel(QStringLiteral("—"), f);
    valL->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:14px; font-variant-numeric:tabular-nums;")
        .arg(text0.name()));
    fl->addWidget(keyL);
    fl->addWidget(valL);
    tripLay->addWidget(f);
    *out = valL;
  };
  makeCell(QStringLiteral("THR"),   &thrLabel_);
  makeCell(QStringLiteral("BRK"),   &brkLabel_);
  makeCell(QStringLiteral("ACCEL"), &accelLabel_);
  col->addWidget(triple);

  // ── 02 DRIVE MODE  +  03 V2X (side by side) ───
  QFrame* dm03 = new QFrame(panel);
  QHBoxLayout* dmRow = new QHBoxLayout(dm03);
  dmRow->setContentsMargins(0, 0, 0, 0);
  dmRow->setSpacing(0);

  QFrame* dmCol = new QFrame(dm03);
  dmCol->setStyleSheet(QStringLiteral("QFrame { border-right:1px solid %1; }")
                           .arg(line.name()));
  QVBoxLayout* dmL = new QVBoxLayout(dmCol);
  dmL->setContentsMargins(0, 0, 0, 0);
  dmL->setSpacing(0);
  Section* sec02 = new Section(QStringLiteral("02"),
                               QStringLiteral("DRIVE MODE"), dmCol);
  dmL->addWidget(sec02);
  QFrame* dmBody = new QFrame(dmCol);
  QVBoxLayout* dmBodyL = new QVBoxLayout(dmBody);
  dmBodyL->setContentsMargins(14, 12, 14, 12);
  dmBodyL->setSpacing(8);

  QLabel* dmKey = new QLabel(QStringLiteral("DRIVE MODE"), dmBody);
  dmKey->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:9px; letter-spacing:2px;")
      .arg(text3.name()));
  dmBodyL->addWidget(dmKey);

  // MANUAL/AUTO toggle (visual only — M4 will add click)
  QFrame* toggle = new QFrame(dmBody);
  toggle->setStyleSheet(QStringLiteral(
      "QFrame { border:1px solid %1; }").arg(line.name()));
  QHBoxLayout* tL = new QHBoxLayout(toggle);
  tL->setContentsMargins(0, 0, 0, 0);
  tL->setSpacing(0);
  manualLabel_ = new QLabel(QStringLiteral("MANUAL"), toggle);
  manualLabel_->setAlignment(Qt::AlignCenter);
  autoLabel_   = new QLabel(QStringLiteral("AUTONOMOUS"), toggle);
  autoLabel_->setAlignment(Qt::AlignCenter);
  for (QLabel* l : {manualLabel_, autoLabel_}) {
    l->setStyleSheet(QStringLiteral(
        "padding:10px 0; font-family:'JetBrains Mono',monospace; "
        "font-size:13px; letter-spacing:1px; color:%1; background:transparent;")
        .arg(text2.name()));
    tL->addWidget(l);
  }
  dmBodyL->addWidget(toggle);

  engagedLabel_ = new QLabel(QStringLiteral("● STANDBY"), dmBody);
  vMaxLabel_    = new QLabel(QStringLiteral("● v_max —"), dmBody);
  oddLabel_     = new QLabel(QStringLiteral("● ODD —"), dmBody);
  for (QLabel* l : {engagedLabel_, vMaxLabel_, oddLabel_}) {
    l->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:10px; letter-spacing:1px;")
        .arg(amber.name()));
    dmBodyL->addWidget(l);
  }
  dmL->addWidget(dmBody);
  dmL->addStretch(1);
  dmRow->addWidget(dmCol, 14);

  QFrame* v2xCol = new QFrame(dm03);
  QVBoxLayout* v2xL = new QVBoxLayout(v2xCol);
  v2xL->setContentsMargins(0, 0, 0, 0);
  v2xL->setSpacing(0);
  Section* sec03 = new Section(QStringLiteral("03"),
                               QStringLiteral("V2X"), v2xCol);
  v2xL->addWidget(sec03);
  trafficLight_ = new TrafficLight(v2xCol);
  trafficLight_->setMinimumSize(140, 100);
  QFrame* tlWrap = new QFrame(v2xCol);
  QHBoxLayout* tlWrapL = new QHBoxLayout(tlWrap);
  tlWrapL->setContentsMargins(8, 12, 14, 12);
  tlWrapL->addStretch(1);
  tlWrapL->addWidget(trafficLight_);
  v2xL->addWidget(tlWrap);
  v2xL->addStretch(1);
  dmRow->addWidget(v2xCol, 10);

  col->addWidget(dm03);

  // ── 04 LOCALIZATION ──────────────────────────
  localSection_ = new Section(QStringLiteral("04"),
                              QStringLiteral("LOCALIZATION"), panel);
  localSection_->setRight(QStringLiteral("NO RTK"));
  localSection_->setRightColor(text3);
  col->addWidget(localSection_);

  QFrame* localBody = new QFrame(panel);
  QGridLayout* lg = new QGridLayout(localBody);
  lg->setContentsMargins(14, 10, 14, 10);
  lg->setHorizontalSpacing(10);
  lg->setVerticalSpacing(6);
  // 4 rows × 4 cols (key val key val) — match JSX order
  auto addPair = [&](int row, int col0, const QString& kText,
                     QLabel** outVal) {
    QLabel* k = new QLabel(kText, localBody);
    k->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:11px; letter-spacing:1px;")
        .arg(text3.name()));
    QLabel* v = new QLabel(QStringLiteral("—"), localBody);
    v->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:11px; font-variant-numeric:tabular-nums;")
        .arg(text0.name()));
    lg->addWidget(k, row, col0);
    lg->addWidget(v, row, col0 + 1);
    *outVal = v;
  };
  // Only the 4 fields backed by /hmi/state get live values; static rows
  // with sigU/HDOP/SATS keep "—" until upstream provides them.
  addPair(0, 0, QStringLiteral("LANE ID"), &laneLabel_);
  QLabel* dummy = nullptr;
  Q_UNUSED(dummy);
  addPair(0, 2, QStringLiteral("LINK"),    &linkLabel_);
  addPair(1, 0, QStringLiteral("σ-EAST"),  &sigELabel_);
  addPair(1, 2, QStringLiteral("σ-NORTH"), &sigNLabel_);

  QLabel* sigUKey = new QLabel(QStringLiteral("σ-UP"), localBody);
  QLabel* sigUVal = new QLabel(QStringLiteral("—"), localBody);
  QLabel* hdopKey = new QLabel(QStringLiteral("HDOP"), localBody);
  QLabel* hdopVal = new QLabel(QStringLiteral("—"), localBody);
  QLabel* satsKey = new QLabel(QStringLiteral("SATS"), localBody);
  QLabel* satsVal = new QLabel(QStringLiteral("—"), localBody);
  QLabel* hdgKey  = new QLabel(QStringLiteral("HEADING"), localBody);
  for (QLabel* k : {sigUKey, hdopKey, satsKey, hdgKey}) {
    k->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:11px; letter-spacing:1px;")
        .arg(text3.name()));
  }
  for (QLabel* v : {sigUVal, hdopVal, satsVal}) {
    v->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:11px; font-variant-numeric:tabular-nums;")
        .arg(text0.name()));
  }
  lg->addWidget(sigUKey, 2, 0);
  lg->addWidget(sigUVal, 2, 1);
  lg->addWidget(hdopKey, 2, 2);
  lg->addWidget(hdopVal, 2, 3);
  lg->addWidget(satsKey, 3, 0);
  lg->addWidget(satsVal, 3, 1);
  lg->addWidget(hdgKey,  3, 2);
  headingLabel_ = new QLabel(QStringLiteral("—"), localBody);
  headingLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:11px; font-variant-numeric:tabular-nums;")
      .arg(text0.name()));
  lg->addWidget(headingLabel_, 3, 3);
  col->addWidget(localBody);

  // ── 05 SYSTEM HEALTH ─────────────────────────
  Section* sec05 = new Section(QStringLiteral("05"),
                               QStringLiteral("SYSTEM HEALTH"), panel);
  // Right side = 4 dot summary
  QFrame* sumWrap = new QFrame(sec05);
  QHBoxLayout* sumLay = new QHBoxLayout(sumWrap);
  sumLay->setContentsMargins(0, 0, 0, 0);
  sumLay->setSpacing(8);
  for (int i = 0; i < 4; ++i) {
    summaryDots_[i] = new Dot(sumWrap);
    summaryDots_[i]->setFixedSize(11, 11);
    summaryDots_[i]->setColor(text3);
    sumLay->addWidget(summaryDots_[i]);
  }
  // Embed sumWrap in section header right slot via composite: Section paints
  // its own header text but doesn't accept a child widget. Instead, position
  // sumWrap as an overlay on top of the Section header.
  sumWrap->setParent(sec05);
  sumWrap->setStyleSheet(QStringLiteral("QFrame { background:transparent; }"));
  sumWrap->show();
  // Defer absolute positioning to a resize-time hook: Section is fixed-height
  // so we can place at right=14 px.
  sumWrap->move(sec05->width() - 60, 7);
  // When section is resized later, reposition. Connect via lambda in showEvent
  // approximation: use installEventFilter — too invasive for M2. The Section
  // also stretches with QHBoxLayout so the dock width is stable in this build
  // (left panel fixed at 320). Re-position once after first layout.
  QTimer::singleShot(0, sumWrap, [sumWrap, sec05]() {
    sumWrap->move(sec05->width() - 60, 7);
  });
  col->addWidget(sec05);

  static const QString labels[6] = {
      QStringLiteral("GPS-RTK"), QStringLiteral("K-ADCU"),
      QStringLiteral("LIDAR"),   QStringLiteral("RADAR"),
      QStringLiteral("CAMERA"),  QStringLiteral("V2X"),
  };
  for (int i = 0; i < 6; ++i) {
    healthRows_[i] = new HealthRow(labels[i], panel);
    healthRows_[i]->setStatus(0);
    col->addWidget(healthRows_[i]);
  }
  col->addStretch(1);

  return panel;
}

// ───────────────────────────────────────────────────────────────────────
// Main area (06 ENVIRONMENT placeholder)
// ───────────────────────────────────────────────────────────────────────

QWidget* F1Dashboard::buildMainArea() {
  using namespace f1tokens;

  QFrame* main = new QFrame(this);
  main->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; }").arg(bg0.name()));

  QVBoxLayout* mainV = new QVBoxLayout(main);
  mainV->setContentsMargins(0, 0, 0, 0);
  mainV->setSpacing(0);

  // Section header for 06
  QFrame* head = new QFrame(main);
  head->setFixedHeight(32);
  head->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; border-bottom:1px solid %2; }")
      .arg(bg1.name(), line.name()));
  QHBoxLayout* hl = new QHBoxLayout(head);
  hl->setContentsMargins(18, 0, 18, 0);
  hl->setSpacing(10);
  QLabel* num = new QLabel(QStringLiteral("06"), head);
  num->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:2px; font-weight:600;")
      .arg(cyan.name()));
  QLabel* title = new QLabel(QStringLiteral("ENVIRONMENT · TOP-DOWN"), head);
  title->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:2px;")
      .arg(text2.name()));
  hl->addWidget(num);
  hl->addWidget(title);
  hl->addStretch(1);
  QLabel* meta = new QLabel(QStringLiteral("SCALE 1:8     GRID 6     ● IDLE"),
                            head);
  meta->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:2px;")
      .arg(text3.name()));
  hl->addWidget(meta);
  mainV->addWidget(head);

  // Center stage — hosts MapScene in M3 (placeholder QLabel until installed).
  mainStage_ = new QFrame(main);
  mainStage_->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; }").arg(bg0.name()));
  QVBoxLayout* sv = new QVBoxLayout(mainStage_);
  sv->setContentsMargins(0, 0, 0, 0);
  sv->setSpacing(0);
  mainPlaceholder_ = new QLabel(QStringLiteral(
      "MapScene loading — waiting for /hmi/threejs/map …"),
      mainStage_);
  mainPlaceholder_->setAlignment(Qt::AlignCenter);
  mainPlaceholder_->setStyleSheet(QStringLiteral(
      "color:%1; font-style:italic; font-family:'JetBrains Mono',monospace; "
      "font-size:11px;").arg(text3.name()));
  sv->addWidget(mainPlaceholder_);
  // Re-bind original local "stage" variable name for downstream references
  // (the rest of buildMainArea references `stage` for overlays).
  QFrame* stage = mainStage_;

  // ODD banner overlay — initially hidden
  oddBanner_ = new QFrame(stage);
  oddBanner_->setStyleSheet(QStringLiteral(
      "QFrame { background:rgba(255,181,71,0.08); "
      "         border:1px solid %1; border-radius:3px; }")
      .arg(amber.name()));
  oddBanner_->setVisible(false);
  oddBanner_->setFixedHeight(36);
  oddBanner_->setMinimumWidth(440);
  QHBoxLayout* obL = new QHBoxLayout(oddBanner_);
  obL->setContentsMargins(20, 6, 20, 6);
  obL->setSpacing(12);
  QLabel* warnLbl = new QLabel(QStringLiteral("⚠ WARN"), oddBanner_);
  warnLbl->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:11px; letter-spacing:1px;").arg(amber.name()));
  oddBannerLabel_ = new QLabel(QString(), oddBanner_);
  oddBannerLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-size:13px;").arg(text0.name()));
  obL->addWidget(warnLbl);
  obL->addWidget(oddBannerLabel_, 1);

  // BAG strip overlay — top-right, initially "IDLE"
  bagOverlay_ = new QFrame(stage);
  bagOverlay_->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; border:1px solid %2; }")
      .arg(bg1.name(), line.name()));
  bagOverlay_->setFixedSize(220, 80);
  QVBoxLayout* bagL = new QVBoxLayout(bagOverlay_);
  bagL->setContentsMargins(10, 10, 10, 10);
  bagL->setSpacing(6);
  bagStateLabel_ = new QLabel(QStringLiteral("● IDLE   ROSBAG"), bagOverlay_);
  bagStateLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:11px; letter-spacing:2px;").arg(text3.name()));
  bagInfoLabel_ = new QLabel(QStringLiteral("START"), bagOverlay_);
  bagInfoLabel_->setAlignment(Qt::AlignCenter);
  bagInfoLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:11px; letter-spacing:3px; font-weight:700; "
      "background:%2; padding:6px;")
      .arg(text1.name(), bg2.name()));
  bagL->addWidget(bagStateLabel_);
  bagL->addWidget(bagInfoLabel_);

  // Position banner & bag overlay via custom resize handler.
  // We attach an event filter that re-positions on stage resize.
  class StageFilter : public QObject {
   public:
    StageFilter(QWidget* parent, QFrame* banner, QFrame* bag)
        : QObject(parent), banner_(banner), bag_(bag) {}
    bool eventFilter(QObject* o, QEvent* e) override {
      if (e->type() == QEvent::Resize) {
        QWidget* w = qobject_cast<QWidget*>(o);
        if (w) {
          // Banner: centered horizontally near top (y=28 in JSX)
          int bw = std::min(w->width() - 40, 600);
          banner_->setGeometry((w->width() - bw) / 2, 28,
                               bw, banner_->height());
          // Bag overlay: top-right (top=4 since main area already has 32px header)
          bag_->setGeometry(w->width() - bag_->width() - 18, 8,
                            bag_->width(), bag_->height());
        }
      }
      return QObject::eventFilter(o, e);
    }
   private:
    QFrame* banner_;
    QFrame* bag_;
  };
  stage->installEventFilter(new StageFilter(stage, oddBanner_, bagOverlay_));

  mainV->addWidget(stage, 1);
  return main;
}

// ───────────────────────────────────────────────────────────────────────
// Bottom strip (8 cells)
// ───────────────────────────────────────────────────────────────────────

QWidget* F1Dashboard::buildBottomStrip() {
  using namespace f1tokens;

  QFrame* strip = new QFrame(this);
  strip->setFixedHeight(56);
  strip->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; border-top:1px solid %2; }")
      .arg(bg1.name(), line.name()));

  QHBoxLayout* row = new QHBoxLayout(strip);
  row->setContentsMargins(0, 0, 0, 0);
  row->setSpacing(0);

  static const QString keys[kBottomCells] = {
      QStringLiteral("EGO-VEL"), QStringLiteral("Δ-LIM"),
      QStringLiteral("LATERAL"), QStringLiteral("JERK"),
      QStringLiteral("LEAD-D"),  QStringLiteral("LEAD-Δv"),
      QStringLiteral("PLAN-H"),  QStringLiteral("CPU"),
  };
  for (int i = 0; i < kBottomCells; ++i) {
    QFrame* cell = new QFrame(strip);
    cell->setStyleSheet(QStringLiteral(
        "QFrame { background:transparent; %1 }")
        .arg(i + 1 < kBottomCells
             ? QStringLiteral("border-right:1px solid %1;").arg(line.name())
             : QString()));
    QVBoxLayout* cl = new QVBoxLayout(cell);
    cl->setContentsMargins(14, 10, 14, 10);
    cl->setSpacing(4);
    bottom_[i].key = new QLabel(keys[i], cell);
    bottom_[i].key->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:9px; letter-spacing:2px;").arg(text3.name()));
    bottom_[i].value = new QLabel(QStringLiteral("—"), cell);
    bottom_[i].value->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:16px; font-variant-numeric:tabular-nums;").arg(text0.name()));
    cl->addWidget(bottom_[i].key);
    cl->addWidget(bottom_[i].value);
    row->addWidget(cell, 1);
  }
  return strip;
}

// ───────────────────────────────────────────────────────────────────────
// Slot implementations
// ───────────────────────────────────────────────────────────────────────

void F1Dashboard::onState(const HmiState& s) {
  using namespace f1tokens;
  lastState_ = s;

  // 01 VELOCITY · STEER
  speedHalf_->setValues(s.speed, static_cast<double>(s.speedLimit));
  steerDial_->setAngle(s.steering);
  steerDial_->setGear(gearLetter(s.gear));

  QString steerStr = QStringLiteral("%1%2°")
                         .arg(s.steering >= 0 ? QStringLiteral("+")
                                              : QStringLiteral("−"))
                         .arg(QString::number(std::fabs(s.steering), 'f', 1));
  steerNumLabel_->setText(steerStr);
  steerDirLabel_->setText(s.steering < 0 ? QStringLiteral("← LEFT")
                                         : QStringLiteral("RIGHT →"));

  // 02 DRIVE MODE
  refreshDriveMode();

  // 04 LOCALIZATION
  // RTK label
  if (s.gps.rtk == 2)      { rtkLabel_ = QStringLiteral("RTK FIX");   rtkColor_ = green; }
  else if (s.gps.rtk == 1) { rtkLabel_ = QStringLiteral("RTK FLOAT"); rtkColor_ = amber; }
  else                     { rtkLabel_ = QStringLiteral("NO RTK");    rtkColor_ = text3; }
  localSection_->setRight(rtkLabel_);
  localSection_->setRightColor(rtkColor_);
  refreshLocalization();

  // 05 health row 0 (GPS-RTK) info uses RTK label + Hz
  refreshHealth();

  // Bottom EGO-VEL & Δ-LIM
  refreshBottom();
}

void F1Dashboard::onDiag(const QHash<QString, int>& status) {
  lastDiag_ = status;
  refreshHealth();
}

void F1Dashboard::onHz(const QHash<QString, double>& hz) {
  lastHz_ = hz;
  refreshHealth();
}

void F1Dashboard::onTraffic(int color, int timeDecisec, int, int) {
  trafficLight_->setPhaseColor(color);
  trafficLight_->setRemainSeconds(static_cast<int>(
      std::round(timeDecisec / 10.0)));
}

void F1Dashboard::onPopup(const QString& text, const QString& severity) {
  if (severity != QStringLiteral("info") && !text.isEmpty()) {
    oddBannerLabel_->setText(text);
    oddBanner_->setVisible(true);
    oddBanner_->raise();
  } else {
    oddBanner_->setVisible(false);
  }
}

void F1Dashboard::onBag(bool recording, const QString& info) {
  using namespace f1tokens;
  if (recording) {
    bagOverlay_->setStyleSheet(QStringLiteral(
        "QFrame { background:rgba(255,59,59,0.06); "
        "         border:1px solid %1; }")
        .arg(with_alpha(red, 102).name(QColor::HexArgb)));
    bagStateLabel_->setText(QStringLiteral("● REC   ROSBAG"));
    bagStateLabel_->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:11px; letter-spacing:2px;").arg(red.name()));
    bagInfoLabel_->setText(info.isEmpty() ? QStringLiteral("STOP") : info);
    bagInfoLabel_->setStyleSheet(QStringLiteral(
        "color:%1; background:%2; "
        "font-family:'JetBrains Mono',monospace; "
        "font-size:11px; letter-spacing:3px; font-weight:700; padding:6px;")
        .arg(text0.name(), red.name()));
  } else {
    bagOverlay_->setStyleSheet(QStringLiteral(
        "QFrame { background:%1; border:1px solid %2; }")
        .arg(bg1.name(), line.name()));
    bagStateLabel_->setText(QStringLiteral("● IDLE   ROSBAG"));
    bagStateLabel_->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:11px; letter-spacing:2px;").arg(text3.name()));
    bagInfoLabel_->setText(QStringLiteral("START"));
    bagInfoLabel_->setStyleSheet(QStringLiteral(
        "color:%1; background:%2; "
        "font-family:'JetBrains Mono',monospace; "
        "font-size:11px; letter-spacing:3px; font-weight:700; padding:6px;")
        .arg(text1.name(), bg2.name()));
  }
}

void F1Dashboard::onConnectionChanged(bool connected, qint64 lastMsgAgeMs) {
  rosConnected_ = connected;
  lastMsgAgeMs_ = lastMsgAgeMs;
  // ROS badge text is updated in tick() because it also depends on hz table.
}

// ───────────────────────────────────────────────────────────────────────
// Refresh helpers
// ───────────────────────────────────────────────────────────────────────

void F1Dashboard::refreshLocalization() {
  using namespace f1tokens;
  laneLabel_->setText(lastState_.laneLabel.isEmpty() ? QStringLiteral("—")
                                                     : lastState_.laneLabel);
  linkLabel_->setText(lastState_.linkId > 0
                          ? QString::number(lastState_.linkId)
                          : QStringLiteral("—"));
  sigELabel_->setText(fmtSigmaCm(lastState_.gps.lonStd));
  sigNLabel_->setText(fmtSigmaCm(lastState_.gps.latStd));
  double yawDeg = lastState_.ego.yaw * kRad2Deg;
  yawDeg = std::fmod(yawDeg, 360.0);
  if (yawDeg < 0.0) yawDeg += 360.0;
  headingLabel_->setText(QString::number(yawDeg, 'f', 1) +
                         QStringLiteral("°"));
}

void F1Dashboard::refreshDriveMode() {
  using namespace f1tokens;
  bool engaged = lastState_.mode == 1;

  // Toggle highlight
  manualLabel_->setStyleSheet(QStringLiteral(
      "padding:10px 0; font-family:'JetBrains Mono',monospace; "
      "font-size:13px; letter-spacing:1px; color:%1; background:%2; "
      "font-weight:%3;")
      .arg(engaged ? text2.name() : bg0.name(),
           engaged ? QStringLiteral("transparent") : cyan.name(),
           engaged ? QStringLiteral("400") : QStringLiteral("600")));
  autoLabel_->setStyleSheet(QStringLiteral(
      "padding:10px 0; font-family:'JetBrains Mono',monospace; "
      "font-size:13px; letter-spacing:1px; color:%1; background:%2; "
      "font-weight:%3;")
      .arg(engaged ? bg0.name() : text2.name(),
           engaged ? cyan.name() : QStringLiteral("transparent"),
           engaged ? QStringLiteral("600") : QStringLiteral("400")));

  // Engaged / vMax / ODD lines
  engagedLabel_->setText(engaged ? QStringLiteral("● ENGAGED")
                                 : QStringLiteral("● STANDBY"));
  engagedLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:1px;")
      .arg((engaged ? green : amber).name()));

  QString vMaxStr = (lastState_.speedLimit > 0)
                        ? QString::number(lastState_.speedLimit)
                        : QStringLiteral("—");
  vMaxLabel_->setText(QStringLiteral("● v_max ") + vMaxStr);
  vMaxLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:1px;").arg(cyan.name()));

  bool oddNominal = (lastState_.onOdd == 0);
  QString oddText = oddNominal ? QStringLiteral("nominal")
                                : QStringLiteral("ODD 이탈");
  oddLabel_->setText(QStringLiteral("● ODD ") + oddText);
  oddLabel_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:1px;")
      .arg((oddNominal ? green : amber).name()));
}

void F1Dashboard::refreshHealth() {
  using namespace f1tokens;
  // Bridge keys: gps, adcu, lidar, radar, cam, v2x.  Same visual order.
  static const QString keys[6]   = {QStringLiteral("gps"),  QStringLiteral("adcu"),
                                    QStringLiteral("lidar"), QStringLiteral("radar"),
                                    QStringLiteral("cam"),   QStringLiteral("v2x")};
  for (int i = 0; i < 6; ++i) {
    int code = lastDiag_.value(keys[i], 0);
    healthRows_[i]->setStatus(code);
    double hz = lastHz_.value(keys[i], 0.0);
    QString hzStr = QString::number(hz, 'f', 1) + QStringLiteral(" Hz");
    if (i == 0) {
      // GPS-RTK row carries RTK label
      healthRows_[i]->setInfo(rtkLabel_ + QStringLiteral(" · ") + hzStr);
    } else {
      healthRows_[i]->setInfo(hzStr);
    }
  }
  // 4-dot summary uses first 4 codes
  for (int i = 0; i < 4; ++i) {
    int code = lastDiag_.value(keys[i], 0);
    QColor c = (code >= 2) ? red : (code == 1) ? amber : green;
    summaryDots_[i]->setColor(c);
  }
}

void F1Dashboard::refreshBottom() {
  using namespace f1tokens;
  bottom_[0].value->setText(fmtKmh(lastState_.speed));
  bottom_[0].value->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:16px; font-variant-numeric:tabular-nums;").arg(cyan.name()));

  if (lastState_.speedLimit > 0) {
    double dLim = lastState_.speed - lastState_.speedLimit;
    QString sgn = dLim >= 0 ? QStringLiteral("+") : QStringLiteral("−");
    QString s = sgn + QString::number(std::fabs(dLim), 'f', 1) +
                QStringLiteral(" km/h");
    bottom_[1].value->setText(s);
    bottom_[1].value->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:16px; font-variant-numeric:tabular-nums;")
        .arg(dLim > 0 ? amber.name() : green.name()));
  } else {
    bottom_[1].value->setText(QStringLiteral("—"));
    bottom_[1].value->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; "
        "font-size:16px;").arg(text3.name()));
  }
}

// ───────────────────────────────────────────────────────────────────────
// Clock / TICK / NET tick
// ───────────────────────────────────────────────────────────────────────

void F1Dashboard::tick() {
  using namespace f1tokens;
  QDateTime now = QDateTime::currentDateTimeUtc();
  utcStat_->setValue(fmtUtc(now));
  kstStat_->setValue(fmtKst(now));

  qint64 ms = uptime_.elapsed();
  tickStat_->setValue(QString::number(ms / 1000.0, 'f', 3) +
                      QStringLiteral("s"));

  // NET text: "—" when not connected, else "<ms>ms" age
  QString netStr = QStringLiteral("—");
  if (rosConnected_ && lastMsgAgeMs_ >= 0) {
    netStr = QString::number(lastMsgAgeMs_) + QStringLiteral("ms");
  }
  netStat_->setValue(netStr);

  // ROS badge — connected + any topic > 0.5 Hz → ONLINE; connected + no Hz →
  // WAITING; not connected → OFFLINE.
  bool anyHz = false;
  for (auto it = lastHz_.cbegin(); it != lastHz_.cend(); ++it) {
    if (it.value() > 0.5) { anyHz = true; break; }
  }
  QString rosLabel; QColor rosColor;
  if (!rosConnected_)   { rosLabel = QStringLiteral("● ROS · OFFLINE"); rosColor = red;   }
  else if (anyHz)       { rosLabel = QStringLiteral("● ROS · ONLINE");  rosColor = green; }
  else                  { rosLabel = QStringLiteral("● ROS · WAITING"); rosColor = amber; }
  rosBadge_->setText(rosLabel);
  rosBadge_->setStyleSheet(QStringLiteral(
      "color:%1; background:%2; border:1px solid %3; padding:5px 12px; "
      "font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:2px;")
      .arg(rosColor.name(),
           with_alpha(rosColor, 26).name(QColor::HexArgb),
           with_alpha(rosColor, 102).name(QColor::HexArgb)));
}

}  // namespace f1widgets
