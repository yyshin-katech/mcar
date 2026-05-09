// qt_hmi/src/style/LayerStyle.cpp
//
// Static layer style table — hex strings parsed once via QColor("#...").
// Identical values to web_hmi types.js LAYER_STYLE so a side-by-side render
// shows no colour drift.
#include "qt_hmi/style/LayerStyle.h"

#include <array>

namespace qt_hmi_style {

namespace {

// Helper to keep the table compact while QColor cannot be constexpr.
LayerStyle make(const char* name, const char* hex,
                float width, float size, float alpha, LayerKind kind) {
  LayerStyle s{};
  s.name  = name;
  s.color = QColor(QString::fromLatin1(hex));
  s.width = width;
  s.size  = size;
  s.alpha = alpha;
  s.kind  = kind;
  return s;
}

const std::array<LayerStyle, 15>& tableImpl() {
  // 13 keys; std::array sized 15 just to keep one slot of head-room would be
  // wasteful — use exactly 13. (Const reference to a function-local static).
  static const std::array<LayerStyle, 15> kTable = {{
      // ── 11 MOLIT NGII layers ────────────────────────────────────────────
      make("A1_NODE",                    "#5cf2ff", 1.0f, 0.4f, 1.00f, LayerKind::Point),
      make("A2_LINK",                    "#00e5ff", 1.0f, 0.0f, 1.00f, LayerKind::Polyline),
      make("A3_DRIVEWAYSECTION",         "#4d7fa3", 1.0f, 0.0f, 0.18f, LayerKind::Polygon),
      make("A4_SUBSIDIARYSECTION",       "#3a5f7d", 1.0f, 0.0f, 0.16f, LayerKind::Polygon),
      make("A5_PARKINGLOT",              "#5a8fb4", 1.0f, 0.0f, 0.20f, LayerKind::Polygon),
      make("B1_SAFETYSIGN",              "#fff352", 1.0f, 0.0f, 0.40f, LayerKind::Polygon),
      make("B2_SURFACELINEMARK",         "#cfd8e3", 1.0f, 0.0f, 1.00f, LayerKind::Polyline),
      make("B3_SURFACEMARK",             "#94a3b8", 1.0f, 0.0f, 0.28f, LayerKind::Polygon),
      make("C1_TRAFFICLIGHT",            "#ff5252", 1.0f, 0.6f, 1.00f, LayerKind::Point),
      make("C3_VEHICLEPROTECTIONSAFETY", "#ef4444", 1.0f, 0.0f, 1.00f, LayerKind::Polyline),
      make("C4_SPEEDBUMP",               "#fb923c", 1.0f, 0.0f, 0.45f, LayerKind::Polygon),
      make("C5_HEIGHTBARRIER",           "#a78bfa", 1.0f, 0.0f, 1.00f, LayerKind::Polyline),
      make("C6_POSTPOINT",               "#94a3b8", 1.0f, 0.3f, 1.00f, LayerKind::Point),
      // ── 2 siheung_dev shp_map root layers ───────────────────────────────
      make("TB_senario_map",             "#00e5ff", 1.0f, 0.0f, 1.00f, LayerKind::Polyline),
      make("TB_senario_surfaceMARK",     "#94a3b8", 1.0f, 0.0f, 0.28f, LayerKind::Polygon),
  }};
  return kTable;
}

}  // namespace

const LayerStyle* layerStyleTable() {
  return tableImpl().data();
}

int layerStyleCount() {
  return static_cast<int>(tableImpl().size());
}

const LayerStyle* findLayerStyle(const QString& name) {
  const auto& t = tableImpl();
  for (const auto& s : t) {
    if (name == QLatin1String(s.name)) return &s;
  }
  return nullptr;
}

const QHash<QString, bool>& defaultLayerVis() {
  static const QHash<QString, bool> kVis = {
      {QStringLiteral("A1_NODE"),                    false},
      {QStringLiteral("A2_LINK"),                    true},
      {QStringLiteral("A3_DRIVEWAYSECTION"),         true},
      {QStringLiteral("A4_SUBSIDIARYSECTION"),       false},
      {QStringLiteral("A5_PARKINGLOT"),              false},
      {QStringLiteral("B1_SAFETYSIGN"),              false},
      {QStringLiteral("B2_SURFACELINEMARK"),         true},
      {QStringLiteral("B3_SURFACEMARK"),             true},
      {QStringLiteral("C1_TRAFFICLIGHT"),            false},
      {QStringLiteral("C3_VEHICLEPROTECTIONSAFETY"), false},
      {QStringLiteral("C4_SPEEDBUMP"),               false},
      {QStringLiteral("C5_HEIGHTBARRIER"),           false},
      {QStringLiteral("C6_POSTPOINT"),               false},
      {QStringLiteral("TB_senario_map"),             true},
      {QStringLiteral("TB_senario_surfaceMARK"),     true},
  };
  return kVis;
}

}  // namespace qt_hmi_style
