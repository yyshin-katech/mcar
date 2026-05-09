// qt_hmi/src/widgets/ControlPanel.cpp
//
// M4 ControlPanel implementation. Pure Qt Widgets — every group is a
// QGroupBox + QVBoxLayout + QCheckBox/QRadioButton/QSlider; no custom paint.
// Color swatches next to layer checkboxes are rendered as small QLabel
// pixmaps so we get the same cyan-on-dark cue as web_hmi/threejs/ControlPanel.jsx.
//
// Defaults and labels are sourced from:
//   - LayerStyle.{h,cpp}      (palette + layer name list)
//   - LayerStyle.cpp          (DEFAULT_LAYER_VIS — 6 layers visible at start)
//   - F1Tokens.h              (bg/text colours, mono font family)
#include "qt_hmi/widgets/ControlPanel.h"

#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtGui/QColor>
#include <QtGui/QFont>
#include <QtGui/QPainter>
#include <QtGui/QPalette>
#include <QtGui/QPixmap>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QSizePolicy>

#include "qt_hmi/style/F1Tokens.h"
#include "qt_hmi/style/LayerStyle.h"
#include "qt_hmi/widgets/TrafficLightWidget.h"

namespace qt_hmi_widgets {

namespace {

// Same long-form labels as web_hmi/web/threejs/ControlPanel.jsx LAYER_LABELS,
// extended to cover the two TB_senario_* siheung_dev shp_map root layers.
const QHash<QString, QString>& layerLabels() {
  static const QHash<QString, QString> kLabels = {
      {QStringLiteral("A1_NODE"),                    QStringLiteral("A1 Nodes")},
      {QStringLiteral("A2_LINK"),                    QStringLiteral("A2 Lane lines")},
      {QStringLiteral("A3_DRIVEWAYSECTION"),         QStringLiteral("A3 Driveway")},
      {QStringLiteral("A4_SUBSIDIARYSECTION"),       QStringLiteral("A4 Subsidiary")},
      {QStringLiteral("A5_PARKINGLOT"),              QStringLiteral("A5 Parking")},
      {QStringLiteral("B1_SAFETYSIGN"),              QStringLiteral("B1 Safety signs")},
      {QStringLiteral("B2_SURFACELINEMARK"),         QStringLiteral("B2 Surface lines")},
      {QStringLiteral("B3_SURFACEMARK"),             QStringLiteral("B3 Surface marks")},
      {QStringLiteral("C1_TRAFFICLIGHT"),            QStringLiteral("C1 Traffic light")},
      {QStringLiteral("C3_VEHICLEPROTECTIONSAFETY"), QStringLiteral("C3 Protection")},
      {QStringLiteral("C4_SPEEDBUMP"),               QStringLiteral("C4 Speed bumps")},
      {QStringLiteral("C5_HEIGHTBARRIER"),           QStringLiteral("C5 Height barrier")},
      {QStringLiteral("C6_POSTPOINT"),               QStringLiteral("C6 Post points")},
      {QStringLiteral("TB_senario_map"),             QStringLiteral("TB road network")},
      {QStringLiteral("TB_senario_surfaceMARK"),     QStringLiteral("TB surface marks")},
  };
  return kLabels;
}

// Render a flat 9×9 color swatch next to each layer checkbox.
QPixmap makeSwatch(const QColor& c) {
  QPixmap pm(9, 9);
  pm.fill(Qt::transparent);
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing, false);
  p.setPen(QPen(QColor(0, 0, 0, 80), 1));
  p.setBrush(c);
  p.drawRect(0, 0, 8, 8);
  return pm;
}

QString groupTitleStyle() {
  using namespace f1tokens;
  return QStringLiteral(
      "QGroupBox { "
      "  color:%1; font-family:'JetBrains Mono',monospace; "
      "  font-size:10px; letter-spacing:2px; "
      "  border:1px solid %2; border-radius:0; "
      "  margin-top:14px; padding:8px 8px 8px 8px; "
      "  background:%3; "
      "} "
      "QGroupBox::title { "
      "  subcontrol-origin:margin; subcontrol-position:top left; "
      "  padding:0 6px; color:%1; "
      "}").arg(f1tokens::text2.name(),
                f1tokens::line.name(),
                f1tokens::bg2.name());
}

QString checkboxStyle() {
  using namespace f1tokens;
  return QStringLiteral(
      "QCheckBox { "
      "  color:%1; font-family:'JetBrains Mono',monospace; "
      "  font-size:10px; spacing:6px; padding:2px 0; "
      "} "
      "QCheckBox::indicator { width:13px; height:13px; } "
      "QCheckBox::indicator:unchecked { "
      "  background:%2; border:1px solid %3; "
      "} "
      "QCheckBox::indicator:checked { "
      "  background:%4; border:1px solid %4; "
      "} "
      "QCheckBox:hover { color:%5; }")
      .arg(text1.name(), bg1.name(), line.name(),
           cyan.name(), text0.name());
}

QString radioStyle() {
  using namespace f1tokens;
  return QStringLiteral(
      "QRadioButton { "
      "  color:%1; font-family:'JetBrains Mono',monospace; "
      "  font-size:10px; spacing:6px; padding:2px 0; "
      "} "
      "QRadioButton::indicator { width:13px; height:13px; } "
      "QRadioButton::indicator:unchecked { "
      "  background:%2; border:1px solid %3; border-radius:7px; "
      "} "
      "QRadioButton::indicator:checked { "
      "  background:%4; border:1px solid %4; border-radius:7px; "
      "}").arg(text1.name(), bg1.name(), line.name(), cyan.name());
}

}  // namespace

// ───────────────────────────────────────────────────────────────────────
// Construction
// ───────────────────────────────────────────────────────────────────────

ControlPanel::ControlPanel(QWidget* parent) : QWidget(parent) {
  using namespace f1tokens;

  setAutoFillBackground(true);
  QPalette pal = palette();
  pal.setColor(QPalette::Window, bg0);
  pal.setColor(QPalette::WindowText, text1);
  setPalette(pal);

  setMinimumWidth(280);
  setMaximumWidth(360);

  // Outer scroll area so the dock survives 720 px height environments.
  QVBoxLayout* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);

  QScrollArea* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  QWidget* page = new QWidget(scroll);
  page->setStyleSheet(QStringLiteral("background:%1;").arg(bg0.name()));
  QVBoxLayout* col = new QVBoxLayout(page);
  col->setContentsMargins(10, 10, 10, 10);
  col->setSpacing(10);

  // Header band — "CONTROL PANEL" title strip in the dock so the user can
  // tell it apart from the dashboard's left panel sections.
  QFrame* head = new QFrame(page);
  head->setFixedHeight(28);
  head->setStyleSheet(QStringLiteral(
      "QFrame { background:%1; border-bottom:1px solid %2; }")
      .arg(bg1.name(), line.name()));
  QHBoxLayout* hh = new QHBoxLayout(head);
  hh->setContentsMargins(8, 0, 8, 0);
  QLabel* hl = new QLabel(QStringLiteral("CONTROL · MAP / DISPLAY"), head);
  hl->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; "
      "font-size:10px; letter-spacing:2px;").arg(cyan.name()));
  hh->addWidget(hl);
  hh->addStretch(1);
  col->addWidget(head);

  // Sections (top → bottom)
  col->addWidget(buildTrafficSection());
  col->addWidget(buildDisplayGroup());
  col->addWidget(buildCameraGroup());
  col->addWidget(buildLayersGroup());
  col->addStretch(1);

  scroll->setWidget(page);
  outer->addWidget(scroll);
}

ControlPanel::~ControlPanel() = default;

// ───────────────────────────────────────────────────────────────────────
// Sections
// ───────────────────────────────────────────────────────────────────────

QWidget* ControlPanel::buildTrafficSection() {
  // Hosts the V2X TrafficLightWidget. No QGroupBox so the card paints its
  // own header strip flush to the panel edge.
  QFrame* frame = new QFrame(this);
  frame->setStyleSheet(QStringLiteral("background:transparent;"));
  QVBoxLayout* lay = new QVBoxLayout(frame);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(0);
  traffic_ = new TrafficLightWidget(frame);
  traffic_->setMinimumHeight(120);
  traffic_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  lay->addWidget(traffic_);
  return frame;
}

QWidget* ControlPanel::buildDisplayGroup() {
  using namespace f1tokens;

  QGroupBox* gb = new QGroupBox(QStringLiteral("DISPLAY"), this);
  gb->setStyleSheet(groupTitleStyle());
  QVBoxLayout* v = new QVBoxLayout(gb);
  v->setContentsMargins(8, 14, 8, 8);
  v->setSpacing(4);

  chkBoxes_   = new QCheckBox(QStringLiteral("Bounding boxes"),  gb);
  chkHeading_ = new QCheckBox(QStringLiteral("Heading arrows"),  gb);
  chkClouds_  = new QCheckBox(QStringLiteral("Point clouds"),    gb);
  for (QCheckBox* c : {chkBoxes_, chkHeading_, chkClouds_}) {
    c->setStyleSheet(checkboxStyle());
    c->setChecked(true);            // matches MapScene default
  }
  v->addWidget(chkBoxes_);
  v->addWidget(chkHeading_);
  v->addWidget(chkClouds_);

  // Point size slider row.
  QFrame* sliderRow = new QFrame(gb);
  QHBoxLayout* sh = new QHBoxLayout(sliderRow);
  sh->setContentsMargins(0, 4, 0, 0);
  sh->setSpacing(6);

  QLabel* sLbl = new QLabel(QStringLiteral("Point size"), sliderRow);
  sLbl->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; font-size:10px;")
      .arg(text2.name()));
  sh->addWidget(sLbl);

  pointSlider_ = new QSlider(Qt::Horizontal, sliderRow);
  // Range: 0.02..0.50 → integer 2..50 (×0.01)
  pointSlider_->setRange(2, 50);
  pointSlider_->setValue(8);          // 0.08 m default (mirrors MapScene)
  pointSlider_->setFixedWidth(110);
  pointSlider_->setStyleSheet(QStringLiteral(
      "QSlider::groove:horizontal { height:4px; background:%1; } "
      "QSlider::sub-page:horizontal { background:%2; } "
      "QSlider::handle:horizontal { background:%3; "
      "  width:10px; margin:-4px 0; border:1px solid %4; }")
      .arg(line.name(), cyanDim.name(), cyan.name(), cyanDim.name()));
  sh->addWidget(pointSlider_);

  pointValueLbl_ = new QLabel(QStringLiteral("0.08"), sliderRow);
  pointValueLbl_->setStyleSheet(QStringLiteral(
      "color:%1; font-family:'JetBrains Mono',monospace; font-size:10px;")
      .arg(text2.name()));
  pointValueLbl_->setMinimumWidth(38);
  sh->addWidget(pointValueLbl_);
  sh->addStretch(1);

  v->addWidget(sliderRow);

  // Wiring
  connect(chkBoxes_, &QCheckBox::toggled,
          this, &ControlPanel::showBoxesChanged);
  connect(chkHeading_, &QCheckBox::toggled,
          this, &ControlPanel::showHeadingChanged);
  connect(chkClouds_, &QCheckBox::toggled,
          this, &ControlPanel::showCloudsChanged);
  connect(pointSlider_,
          QOverload<int>::of(&QSlider::valueChanged),
          this, &ControlPanel::onPointSliderMoved);

  return gb;
}

QWidget* ControlPanel::buildCameraGroup() {
  using namespace f1tokens;

  QGroupBox* gb = new QGroupBox(QStringLiteral("CAMERA"), this);
  gb->setStyleSheet(groupTitleStyle());
  QVBoxLayout* v = new QVBoxLayout(gb);
  v->setContentsMargins(8, 14, 8, 8);
  v->setSpacing(4);

  camIso_ = new QRadioButton(QStringLiteral("Iso (chase)"), gb);
  camTop_ = new QRadioButton(QStringLiteral("Top (bird's-eye)"), gb);
  camIso_->setChecked(true);          // matches MapScene default
  camIso_->setStyleSheet(radioStyle());
  camTop_->setStyleSheet(radioStyle());

  camGroup_ = new QButtonGroup(this);
  camGroup_->setExclusive(true);
  camGroup_->addButton(camIso_, 0);
  camGroup_->addButton(camTop_, 1);

  v->addWidget(camIso_);
  v->addWidget(camTop_);

  connect(camIso_, &QRadioButton::toggled,
          this, &ControlPanel::onCameraIso);
  connect(camTop_, &QRadioButton::toggled,
          this, &ControlPanel::onCameraTop);

  return gb;
}

QWidget* ControlPanel::buildLayersGroup() {
  using namespace f1tokens;

  QGroupBox* gb = new QGroupBox(QStringLiteral("MAP LAYERS"), this);
  gb->setStyleSheet(groupTitleStyle());
  QVBoxLayout* v = new QVBoxLayout(gb);
  v->setContentsMargins(8, 14, 8, 8);
  v->setSpacing(2);

  const QHash<QString, bool>& defVis = qt_hmi_style::defaultLayerVis();
  const int n = qt_hmi_style::layerStyleCount();
  const auto* table = qt_hmi_style::layerStyleTable();

  for (int i = 0; i < n; ++i) {
    const auto& s = table[i];
    QString name = QString::fromLatin1(s.name);

    QFrame* row = new QFrame(gb);
    QHBoxLayout* rh = new QHBoxLayout(row);
    rh->setContentsMargins(0, 0, 0, 0);
    rh->setSpacing(6);

    QCheckBox* cb = new QCheckBox(row);
    cb->setStyleSheet(checkboxStyle());
    cb->setChecked(defVis.value(name, true));
    layerCheckboxes_.insert(name, cb);
    rh->addWidget(cb);

    QLabel* swatch = new QLabel(row);
    swatch->setPixmap(makeSwatch(s.color));
    swatch->setFixedSize(11, 11);
    rh->addWidget(swatch);

    QLabel* lbl = new QLabel(layerLabels().value(name, name), row);
    lbl->setStyleSheet(QStringLiteral(
        "color:%1; font-family:'JetBrains Mono',monospace; font-size:10px;")
        .arg(text1.name()));
    rh->addWidget(lbl, 1);

    v->addWidget(row);

    connect(cb, &QCheckBox::toggled,
            this, &ControlPanel::onLayerToggled);
  }

  return gb;
}

// ───────────────────────────────────────────────────────────────────────
// Slots — collect snapshot + emit
// ───────────────────────────────────────────────────────────────────────

void ControlPanel::onLayerToggled() {
  QHash<QString, bool> vis;
  for (auto it = layerCheckboxes_.cbegin();
       it != layerCheckboxes_.cend(); ++it) {
    vis.insert(it.key(), it.value()->isChecked());
  }
  emit layerVisibilityChanged(vis);
}

void ControlPanel::onPointSliderMoved(int v) {
  float sz = static_cast<float>(v) * 0.01f;
  if (pointValueLbl_) {
    pointValueLbl_->setText(QString::number(sz, 'f', 2));
  }
  emit pointSizeChanged(sz);
}

void ControlPanel::onCameraIso(bool checked) {
  if (checked) emit cameraModeChanged(QStringLiteral("iso"));
}

void ControlPanel::onCameraTop(bool checked) {
  if (checked) emit cameraModeChanged(QStringLiteral("top"));
}

// ───────────────────────────────────────────────────────────────────────
// emitInitialState — push current widget state out without user click. Used
// by MainWindow right after wiring so MapScene receives matching defaults.
// ───────────────────────────────────────────────────────────────────────

void ControlPanel::emitInitialState() {
  // Layer visibility snapshot
  QHash<QString, bool> vis;
  for (auto it = layerCheckboxes_.cbegin();
       it != layerCheckboxes_.cend(); ++it) {
    vis.insert(it.key(), it.value()->isChecked());
  }
  emit layerVisibilityChanged(vis);
  emit showBoxesChanged(chkBoxes_ ? chkBoxes_->isChecked() : true);
  emit showHeadingChanged(chkHeading_ ? chkHeading_->isChecked() : true);
  emit showCloudsChanged(chkClouds_ ? chkClouds_->isChecked() : true);
  if (pointSlider_) {
    emit pointSizeChanged(static_cast<float>(pointSlider_->value()) * 0.01f);
  }
  emit cameraModeChanged(camTop_ && camTop_->isChecked()
                             ? QStringLiteral("top")
                             : QStringLiteral("iso"));
}

}  // namespace qt_hmi_widgets
