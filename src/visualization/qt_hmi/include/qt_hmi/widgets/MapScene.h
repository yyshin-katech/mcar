// qt_hmi/widgets/MapScene.h
//
// 3D top-down / iso scene rendered with QOpenGLWidget + Core 3.3 direct GL.
// Replaces the F1Dashboard main-area placeholder when /hmi/threejs/map +
// /hmi/threejs/tracks data arrives.
//
// World axes match three.js code: +X = east_delta (m), +Z = -north_delta,
// +Y = up. Heading yaw rotates around +Y (CCW when looking down).
//
// Threading: all GL calls happen on the GUI thread (QOpenGLWidget invariant).
// Slots invoked from Qt::QueuedConnection — RosBridge signals from spinner
// threads automatically marshal to the GUI thread.
//
// Shaders are inline C-string constants in MapScene.cpp (no .qrc / no .glsl
// files — keeps the M3 footprint small and avoids resource-compilation
// indirection on Qt5.12.8).
//
// See claude_work_list/hmi_design/01_design.md §4 + §5/M3.
#pragma once

#include <QtCore/QHash>
#include <QtCore/QPointF>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtGui/QMatrix4x4>
#include <QtWidgets/QOpenGLWidget>

#include <QtGui/QOpenGLFunctions_3_3_Core>

#include "qt_hmi/HmiTypes.h"
#include "qt_hmi/style/LayerStyle.h"

namespace qt_hmi_widgets {

class MapScene : public QOpenGLWidget,
                 protected QOpenGLFunctions_3_3_Core {
  Q_OBJECT

 public:
  explicit MapScene(QWidget* parent = nullptr);
  ~MapScene() override;

 public slots:
  // RosBridge::threejsMapReceived
  void onMapReceived(const HmiMap3D& m);
  // RosBridge::tracksChanged
  void onTracksChanged(const QVector<HmiTrack3D>& tracks);
  // RosBridge::stateChanged — used for ego pose + camera follow only.
  void onStateChanged(const HmiState& s);
  // RosBridge::egoPoseChanged — 50 Hz ego pose direct from
  // /localization/to_control_team. Overrides the 10 Hz ego field of
  // onStateChanged so camera follow runs at rviz cadence.
  void onEgoPoseChanged(double east, double north, double yaw);

  // ControlPanel hooks (M4 will wire these, M3 keeps DEFAULT_LAYER_VIS).
  void setLayerVisibility(const QHash<QString, bool>& vis);
  void setShowBoxes(bool on);
  void setShowHeading(bool on);
  void setShowClouds(bool on);
  void setPointSize(float sz);
  void setCameraMode(const QString& mode);   // "iso" | "top"

 protected:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void wheelEvent(QWheelEvent* e) override;

 private:
  // GL helpers
  GLuint compileShader(GLenum type, const char* src, const char* tag);
  GLuint linkProgram(GLuint vs, GLuint fs, const char* tag);

  void rebuildMapLayers();
  void destroyMapLayers();

  void updateOrCreateTrackBox(const HmiTrack3D& t);
  void updateOrCreateTrackCloud(const HmiTrack3D& t);
  void pruneTrackSlots(const QVector<HmiTrack3D>& tracks);
  void destroyTrackSlots();

  void buildEgoBox();
  void destroyEgoBox();

  void buildGrid();
  void destroyGrid();

  // Compute view & projection from current state. Called every paintGL.
  QMatrix4x4 viewMatrix()       const;
  QMatrix4x4 projectionMatrix() const;

  // Color for a track type, mirrors web_hmi types.js OBJ_PALETTE.
  QColor trackColor(const QString& type) const;
  // Fallback height per track type (web_hmi OBJ_DIMS.h).
  float  trackHeight(const QString& type) const;

  // ── GL programs ─────────────────────────────────────────────────
  // line.{vert,frag}: per-vertex position transform + uniform color/alpha.
  GLuint progLine_   = 0;
  GLint  uLineMvp_   = -1;
  GLint  uLineColor_ = -1;

  // point.{vert,frag}: same VS, FS gives a small disk (round point).
  GLuint progPoint_      = 0;
  GLint  uPointMvp_      = -1;
  GLint  uPointColor_    = -1;
  GLint  uPointSize_     = -1;   // gl_PointSize uniform

  // ── Map layer GL slots ──────────────────────────────────────────
  struct LayerGL {
    GLuint vao   = 0;
    GLuint vbo   = 0;
    int    count = 0;             // vertex count
    GLenum primitive = 0;         // GL_LINES / GL_POINTS
    QColor color;
    float  alpha   = 1.0f;
    float  pointSz = 1.0f;
    bool   visible = true;
  };
  QHash<QString, LayerGL> mapLayers_;

  // ── Per-track slots ─────────────────────────────────────────────
  struct TrackBoxSlot {
    GLuint vao = 0;
    GLuint vbo = 0;
    int    count = 0;             // 24 verts (12 lines)
    QColor color;
    double cx = 0.0, cz = 0.0;    // ego-frame: forward (m), -lateral (m)
    double yaw = 0.0;             // rad
    bool   alive = true;
  };
  QHash<int, TrackBoxSlot> trackBoxes_;

  struct TrackCloudSlot {
    GLuint vao = 0;
    GLuint vbo = 0;
    int    capacity = 0;          // floats slots (numPoints*3)
    int    count    = 0;          // numPoints currently uploaded
    QColor color;
    bool   alive = true;
  };
  QHash<int, TrackCloudSlot> trackClouds_;

  // ── Ego ─────────────────────────────────────────────────────────
  GLuint egoVao_   = 0;
  GLuint egoVbo_   = 0;
  int    egoCount_ = 0;            // 24 verts (12 lines for unit box)

  // ── Grid (top-down ground reference) ───────────────────────────
  GLuint gridVao_   = 0;
  GLuint gridVbo_   = 0;
  int    gridCount_ = 0;

  // ── Map / scene state ───────────────────────────────────────────
  QPointF mapOrigin_;        // (east0, north0) — bridge applied
  bool    mapReady_ = false;
  bool    mapDirty_ = false; // true between onMapReceived and rebuildMapLayers in initializeGL/paintGL
  HmiMap3D pendingMap_;      // queued before initializeGL has run

  // ── Camera state ────────────────────────────────────────────────
  HmiState lastState_;
  QString  camMode_   = QStringLiteral("iso");
  float    zoom_      = 1.0f;

  // ── Display flags ───────────────────────────────────────────────
  bool   showBoxes_   = true;
  bool   showHeading_ = true;
  bool   showClouds_  = true;
  float  pointSize_   = 0.08f;
  QHash<QString, bool> layerVis_;   // applied on rebuild + per-frame

  // ── 60 fps render tick — mirrors three.js requestAnimationFrame loop
  // so paintGL runs at display refresh even when /hmi/state arrives at 10 Hz.
  QTimer* renderTimer_ = nullptr;

  // ── Ego pose smoothing (exponential decay between 10 Hz samples) ──
  // /hmi/state arrives at 10 Hz so the camera/ego position would jump every
  // 100 ms. Per render tick (16 ms) we ease the displayed pose toward the
  // latest sample, mirroring rviz's TF-interpolated follow camera. yaw is
  // unwrapped via atan2(sin Δ, cos Δ) so a wrap from +π → -π doesn't spin.
  double smEast_  = 0.0;
  double smNorth_ = 0.0;
  double smYaw_   = 0.0;
  bool   smInit_  = false;
};

}  // namespace qt_hmi_widgets
