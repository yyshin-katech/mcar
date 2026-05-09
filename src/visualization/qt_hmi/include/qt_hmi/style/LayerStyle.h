// qt_hmi/style/LayerStyle.h
//
// HD map layer palette + default visibility — direct port of
// `web_hmi/web/threejs/types.js` LAYER_STYLE / DEFAULT_LAYER_VIS.
//
// 13 keys total: 11 NGII MOLIT layers (A1_NODE … C6_POSTPOINT) + 2 siheung_dev
// shp_map root layers (TB_senario_map, TB_senario_surfaceMARK).
//
// **Kind matching policy** — payload `layer.kind` is authoritative.
// types.js encodes LAYER_STYLE.kind only as a hint (web_hmi MapLayers.jsx
// branches on payload.kind, not on style.kind). qt_hmi mirrors that — the
// LayerKind field here is informational; MapScene.cpp dispatches off
// HmiLayer3D::kind from /hmi/threejs/map.
//
// See claude_work_list/hmi_design/01_design.md §5/M3.
#pragma once

#include <QtCore/QHash>
#include <QtCore/QString>
#include <QtGui/QColor>

namespace qt_hmi_style {

enum class LayerKind {
  Point,
  Polyline,
  Polygon,
};

// One entry per layer; fields mirror types.js shape (color/width/size/alpha).
struct LayerStyle {
  const char* name;
  QColor      color;
  // Polyline / polygon-outline line width (ignored for points).
  float       width;
  // Point primitive size (ignored for line/polygon — see kind).
  float       size;
  // Polygon outline alpha (ignored for points / polylines that have no alpha
  // entry in types.js — fallback to 1.0 in those cases).
  float       alpha;
  LayerKind   kind;
};

// 13 layer table. Order matches types.js (and DEFAULT_LAYER_VIS hash below).
// Inline to avoid needing a .cpp file for static data — Qt5 disallows
// constexpr QColor so we use anonymous-namespace inline functions wrapped
// in an accessor. Defined in LayerStyle.cpp to keep includes minimal.
const LayerStyle* layerStyleTable();
int               layerStyleCount();
const LayerStyle* findLayerStyle(const QString& name);

// Default layer visibility map (mirrors types.js DEFAULT_LAYER_VIS).
const QHash<QString, bool>& defaultLayerVis();

}  // namespace qt_hmi_style
