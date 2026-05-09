// qt_hmi/HmiTypes.h
//
// Plain-old-data structs that mirror the JSON payloads published by
// web_hmi bridge (`/hmi/*`). Used as Qt signal arguments via
// Q_DECLARE_METATYPE + qRegisterMetaType.
//
// See claude_work_list/hmi_design/01_design.md §2.C.
#pragma once

#include <QtCore/QHash>
#include <QtCore/QMetaType>
#include <QtCore/QPointF>
#include <QtCore/QString>
#include <QtCore/QVector>

struct HmiEgo {
  double east  = 0.0;
  double north = 0.0;
  double yaw   = 0.0;   // radians, ENU
};

struct HmiGps {
  int    rtk    = 0;    // 0 = no fix, 1 = float, 2 = fixed
  double lonStd = 0.0;
  double latStd = 0.0;
};

struct HmiState {
  double  speed         = 0.0;   // km/h
  double  steering      = 0.0;   // deg, +left
  int     gear          = 0;     // 0=N,5=D,6=R,7=P (web_hmi 매핑)
  int     mode          = 0;     // autonomous_mode (0..3)
  bool    aeb           = false;
  HmiEgo  ego;
  HmiGps  gps;
  int     speedLimit    = 0;     // km/h
  int     linkId        = 0;
  int     onOdd         = 0;     // 0/1
  int     roadState     = 0;
  int     selectedMode  = 0;     // mode_request pulse mirror
  QString laneLabel;
};

struct HmiObject {
  int     id          = 0;
  QString type;                    // "car","pedestrian","truck",...
  double  x           = 0.0;       // ego frame meters (forward)
  double  y           = 0.0;       // ego frame meters (left)
  double  vx          = 0.0;
  double  vy          = 0.0;
  double  width       = 0.0;
  double  length      = 0.0;
  double  orientation = 0.0;       // rad
};

struct Polyline {
  // EPSG:5179 absolute coordinates (east, north).
  QVector<QPointF> pts;
};

// /hmi/threejs/map layer. `kind` ∈ {"point","polyline","polygon"}.
//   - polyline / polygon: features[i] = vertices of the i-th feature
//     (delta_east, delta_north relative to map.origin).
//   - point: features[0] holds all points (single flat feature).
struct HmiLayer3D {
  QString kind;
  QVector<QVector<QPointF>> features;
};

struct HmiMap3D {
  int     epsg = 5179;
  QPointF origin;                  // (east0, north0) used to shift to delta coords
  QHash<QString, HmiLayer3D> layers;
};

struct HmiTrack3D {
  int     id          = 0;
  QString type;
  double  x           = 0.0;       // EPSG:5179 absolute east (bridge expands relative→abs)
  double  y           = 0.0;       // EPSG:5179 absolute north
  double  vx          = 0.0;
  double  vy          = 0.0;
  double  sizeX       = 0.0;
  double  sizeY       = 0.0;
  double  orientation = 0.0;       // rad
  double  confidence  = 0.0;
  // Optional point cloud (track-attached): flat [x0,y0,z0, x1,y1,z1, ...] in
  // EPSG:5179 absolute (or world) coordinates as published by bridge.
  QVector<float> points;
};

Q_DECLARE_METATYPE(HmiState)
Q_DECLARE_METATYPE(HmiObject)
Q_DECLARE_METATYPE(QVector<HmiObject>)
Q_DECLARE_METATYPE(Polyline)
Q_DECLARE_METATYPE(QVector<Polyline>)
Q_DECLARE_METATYPE(HmiMap3D)
Q_DECLARE_METATYPE(HmiTrack3D)
Q_DECLARE_METATYPE(QVector<HmiTrack3D>)
