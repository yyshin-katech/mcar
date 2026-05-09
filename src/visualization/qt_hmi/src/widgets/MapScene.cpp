// qt_hmi/src/widgets/MapScene.cpp
//
// QOpenGLWidget + Core 3.3 direct GL implementation.
//
// Camera math (design §4.D):
//   delta_east  = state.ego.east  - map.origin.x
//   delta_north = state.ego.north - map.origin.y
//   ex = delta_east
//   ez = -delta_north             // reverse-Z (avoids three.js scene.scale.z trick)
//
//   iso (chase):
//     back   = 60 / zoom
//     height = 60 / zoom
//     cam.pos = (ex - back*cos(yaw), height, ez + back*sin(yaw))
//     cam.up  = (0, 1, 0)
//     cam.lookAt(ex, 0, ez)
//
//   top (bird's-eye):
//     cam.up  = (0, 0, -1)
//     cam.pos = (ex, 100/zoom, ez)
//     cam.lookAt(ex, 0, ez)
//
// All vertex coordinates are uploaded with this z=-north_delta sign convention
// already baked in — no scene-wide scale.z=-1 trick.
#include "qt_hmi/widgets/MapScene.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <QtCore/QDebug>
#include <QtCore/QtMath>
#include <QtGui/QWheelEvent>

namespace qt_hmi_widgets {

namespace {

// ── Shaders (inline) ────────────────────────────────────────────────────

// Lines / triangles. Pos in meters; uniform MVP transforms to clip.
const char* kVertLine =
    "#version 330 core\n"
    "layout(location=0) in vec3 aPos;\n"
    "uniform mat4 uMvp;\n"
    "void main() { gl_Position = uMvp * vec4(aPos, 1.0); }\n";

const char* kFragLine =
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "uniform vec4 uColor;\n"
    "void main() { FragColor = uColor; }\n";

// Points: same VS+FS but FS makes a soft circular disk (avoids square points).
const char* kVertPoint =
    "#version 330 core\n"
    "layout(location=0) in vec3 aPos;\n"
    "uniform mat4 uMvp;\n"
    "uniform float uPointSize;\n"
    "void main() {\n"
    "  gl_Position = uMvp * vec4(aPos, 1.0);\n"
    "  gl_PointSize = uPointSize;\n"
    "}\n";

const char* kFragPoint =
    "#version 330 core\n"
    "out vec4 FragColor;\n"
    "uniform vec4 uColor;\n"
    "void main() {\n"
    "  vec2 q = gl_PointCoord - vec2(0.5);\n"
    "  if (dot(q, q) > 0.25) discard;\n"
    "  FragColor = uColor;\n"
    "}\n";

// IONIQ 5 dimensions (m) — same as web_hmi EgoMesh.jsx.
constexpr float kEgoLength = 4.635f;
constexpr float kEgoWidth  = 1.890f;
constexpr float kEgoHeight = 1.605f;

// Camera presets (design §4.D).
constexpr float kIsoBack   = 60.0f;
constexpr float kIsoHeight = 60.0f;
constexpr float kTopHeight = 100.0f;
constexpr float kZoomMin   = 0.2f;
constexpr float kZoomMax   = 5.0f;
constexpr float kWheelSens = 0.001f;

// Build a wireframe-box edge list (12 line segments, 24 vertices, 3 floats each).
// Centered at origin, dimensions (lx, ly height, lz). +X forward.
void appendBoxLines(std::vector<float>& out,
                    float lx, float ly, float lz, float yOffset = 0.0f) {
  const float hx = lx * 0.5f, hy = ly * 0.5f, hz = lz * 0.5f;
  const float y0 = yOffset - hy, y1 = yOffset + hy;
  const float corners[8][3] = {
      {-hx, y0, -hz}, { hx, y0, -hz}, { hx, y0,  hz}, {-hx, y0,  hz},
      {-hx, y1, -hz}, { hx, y1, -hz}, { hx, y1,  hz}, {-hx, y1,  hz},
  };
  static const int edges[12][2] = {
      {0,1},{1,2},{2,3},{3,0},     // bottom
      {4,5},{5,6},{6,7},{7,4},     // top
      {0,4},{1,5},{2,6},{3,7},     // verticals
  };
  for (int i = 0; i < 12; ++i) {
    for (int e = 0; e < 2; ++e) {
      const float* v = corners[edges[i][e]];
      out.push_back(v[0]);
      out.push_back(v[1]);
      out.push_back(v[2]);
    }
  }
}

// Build a heading arrow (single line segment in +X direction). Caller should
// place this at the front-center of the parent box.
void appendArrow(std::vector<float>& out, float length, float yHeight) {
  // Line tail (0,y,0) → tip (length, y, 0)
  out.push_back(0.0f);   out.push_back(yHeight); out.push_back(0.0f);
  out.push_back(length); out.push_back(yHeight); out.push_back(0.0f);
}

// Color rgba [0..1]
QVector4D toVec4(const QColor& c, float alphaMul = 1.0f) {
  return QVector4D(c.redF(), c.greenF(), c.blueF(), c.alphaF() * alphaMul);
}

}  // namespace

// ────────────────────────────────────────────────────────────────────────
// Construction
// ────────────────────────────────────────────────────────────────────────

MapScene::MapScene(QWidget* parent) : QOpenGLWidget(parent) {
  // Multi-sampled framebuffer for cheap line AA on iGPU.
  QSurfaceFormat fmt;
  fmt.setVersion(3, 3);
  fmt.setProfile(QSurfaceFormat::CoreProfile);
  fmt.setSamples(4);
  fmt.setDepthBufferSize(24);
  fmt.setStencilBufferSize(0);
  setFormat(fmt);

  setMinimumSize(320, 240);
  setFocusPolicy(Qt::StrongFocus);   // wheel + key reach us

  // Default visibility = DEFAULT_LAYER_VIS (design §5/M3).
  layerVis_ = qt_hmi_style::defaultLayerVis();

  // 60 fps render loop. /hmi/state arrives at 10 Hz so onStateChanged-only
  // updates render at 10 fps and the camera follow looks choppy compared to
  // ThreeScene.jsx's requestAnimationFrame loop. Schedule update() every 16 ms
  // to drive paintGL at display refresh; ego pose still snaps to the latest
  // 10 Hz sample but the scene chrome (grid, layers, tracks) stays smooth.
  renderTimer_ = new QTimer(this);
  renderTimer_->setTimerType(Qt::PreciseTimer);
  connect(renderTimer_, &QTimer::timeout,
          this, QOverload<>::of(&QWidget::update));
  renderTimer_->start(16);
}

MapScene::~MapScene() {
  // Stop the 60 fps tick before tearing down GL — guards against a queued
  // timeout firing while the context is half-destroyed.
  if (renderTimer_) renderTimer_->stop();

  // GL teardown must happen with our GL context current.
  makeCurrent();
  destroyMapLayers();
  destroyTrackSlots();
  destroyEgoBox();
  destroyGrid();
  if (progLine_)  glDeleteProgram(progLine_);
  if (progPoint_) glDeleteProgram(progPoint_);
  doneCurrent();
}

// ────────────────────────────────────────────────────────────────────────
// GL init
// ────────────────────────────────────────────────────────────────────────

GLuint MapScene::compileShader(GLenum type, const char* src, const char* tag) {
  GLuint sh = glCreateShader(type);
  glShaderSource(sh, 1, &src, nullptr);
  glCompileShader(sh);
  GLint ok = 0;
  glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char buf[1024]{};
    glGetShaderInfoLog(sh, sizeof(buf), nullptr, buf);
    qWarning() << "[MapScene]" << tag << "compile failed:" << buf;
    glDeleteShader(sh);
    return 0;
  }
  return sh;
}

GLuint MapScene::linkProgram(GLuint vs, GLuint fs, const char* tag) {
  GLuint p = glCreateProgram();
  glAttachShader(p, vs);
  glAttachShader(p, fs);
  glLinkProgram(p);
  GLint ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (!ok) {
    char buf[1024]{};
    glGetProgramInfoLog(p, sizeof(buf), nullptr, buf);
    qWarning() << "[MapScene]" << tag << "link failed:" << buf;
    glDeleteProgram(p);
    p = 0;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);
  return p;
}

void MapScene::initializeGL() {
  if (!initializeOpenGLFunctions()) {
    qWarning() << "[MapScene] initializeOpenGLFunctions() failed — "
                  "GL 3.3 Core context unavailable.";
    return;
  }
  // Background — same hex as web_hmi ThreeScene.jsx (#04060a).
  glClearColor(0.0156f, 0.0235f, 0.0392f, 1.0f);

  // GL state: depth-test for ego/object boxes; transparent line/polygon outlines
  // use plain blending (no depth write). We toggle these per-draw.
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_PROGRAM_POINT_SIZE);   // honor gl_PointSize in vertex shader
  glEnable(GL_LINE_SMOOTH);          // best-effort AA on Mesa

  // Compile shaders.
  GLuint vs = compileShader(GL_VERTEX_SHADER,   kVertLine,  "line.vert");
  GLuint fs = compileShader(GL_FRAGMENT_SHADER, kFragLine,  "line.frag");
  progLine_ = (vs && fs) ? linkProgram(vs, fs, "line.prog") : 0;
  if (progLine_) {
    uLineMvp_   = glGetUniformLocation(progLine_, "uMvp");
    uLineColor_ = glGetUniformLocation(progLine_, "uColor");
  }

  GLuint vsp = compileShader(GL_VERTEX_SHADER,   kVertPoint, "point.vert");
  GLuint fsp = compileShader(GL_FRAGMENT_SHADER, kFragPoint, "point.frag");
  progPoint_ = (vsp && fsp) ? linkProgram(vsp, fsp, "point.prog") : 0;
  if (progPoint_) {
    uPointMvp_   = glGetUniformLocation(progPoint_, "uMvp");
    uPointColor_ = glGetUniformLocation(progPoint_, "uColor");
    uPointSize_  = glGetUniformLocation(progPoint_, "uPointSize");
  }

  buildEgoBox();
  buildGrid();

  // If onMapReceived arrived before the GL context, build deferred map now.
  if (mapDirty_) {
    rebuildMapLayers();
    mapDirty_ = false;
  }
}

// ────────────────────────────────────────────────────────────────────────
// Resize / projection
// ────────────────────────────────────────────────────────────────────────

void MapScene::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
}

QMatrix4x4 MapScene::projectionMatrix() const {
  QMatrix4x4 p;
  const int w = std::max(1, width());
  const int h = std::max(1, height());
  p.perspective(50.0f, float(w) / float(h), 0.5f, 5000.0f);
  return p;
}

QMatrix4x4 MapScene::viewMatrix() const {
  const float ex  = float(smEast_  - mapOrigin_.x());
  const float ez  = -float(smNorth_ - mapOrigin_.y());
  const float yaw = float(smYaw_);

  QMatrix4x4 v;
  if (camMode_ == QLatin1String("top")) {
    // Bird's-eye, north-up (= -Z screen-up after our z-flip).
    QVector3D eye(ex, kTopHeight / zoom_, ez);
    QVector3D ctr(ex, 0.0f, ez);
    QVector3D up(0.0f, 0.0f, -1.0f);
    v.lookAt(eye, ctr, up);
  } else {
    // Iso chase — camera behind ego along heading.
    const float back = kIsoBack   / zoom_;
    const float h    = kIsoHeight / zoom_;
    const float cx = ex      - back * std::cos(yaw);
    const float cz = ez      + back * std::sin(yaw);
    QVector3D eye(cx, h, cz);
    QVector3D ctr(ex, 0.0f, ez);
    QVector3D up(0.0f, 1.0f, 0.0f);
    v.lookAt(eye, ctr, up);
  }
  return v;
}

// ────────────────────────────────────────────────────────────────────────
// Map layers
// ────────────────────────────────────────────────────────────────────────

void MapScene::onMapReceived(const HmiMap3D& m) {
  pendingMap_ = m;
  mapOrigin_  = m.origin;
  mapReady_   = true;

  if (context() && context()->isValid()) {
    makeCurrent();
    rebuildMapLayers();
    doneCurrent();
    update();
  } else {
    // GL not initialised yet; flag for build in initializeGL().
    mapDirty_ = true;
  }
}

void MapScene::rebuildMapLayers() {
  destroyMapLayers();

  if (pendingMap_.layers.isEmpty()) return;

  for (auto it = pendingMap_.layers.constBegin();
       it != pendingMap_.layers.constEnd(); ++it) {
    const QString& name   = it.key();
    const HmiLayer3D& lyr = it.value();

    const auto* style = qt_hmi_style::findLayerStyle(name);
    if (!style) {
      // Unknown layer — skip (web_hmi MapLayers.jsx does the same).
      continue;
    }

    // Payload kind drives geometry construction (memory: project_qt_hmi
    // §JSON 파서 함정 — payload kind authoritative, not style.kind).
    std::vector<float> verts;        // flat x,y,z … in our world frame
    GLenum primitive = 0;
    float  alpha     = 1.0f;
    float  pointSz   = 1.0f;

    if (lyr.kind == QLatin1String("polyline")) {
      // For each polyline feature, build GL_LINES vertex pairs.
      for (const QVector<QPointF>& line : lyr.features) {
        if (line.size() < 2) continue;
        for (int i = 0; i + 1 < line.size(); ++i) {
          const QPointF& a = line.at(i);
          const QPointF& b = line.at(i + 1);
          verts.push_back(float(a.x())); verts.push_back(0.0f);
          verts.push_back(-float(a.y()));
          verts.push_back(float(b.x())); verts.push_back(0.0f);
          verts.push_back(-float(b.y()));
        }
      }
      primitive = GL_LINES;
      alpha     = 1.0f;     // polylines opaque per types.js (no alpha key)
    } else if (lyr.kind == QLatin1String("polygon")) {
      // Outline only (matches web_hmi LineLoop) — emit closed-loop GL_LINES.
      for (const QVector<QPointF>& ring : lyr.features) {
        if (ring.size() < 3) continue;
        const int n = ring.size();
        for (int i = 0; i < n; ++i) {
          const QPointF& a = ring.at(i);
          const QPointF& b = ring.at((i + 1) % n);
          verts.push_back(float(a.x())); verts.push_back(0.0f);
          verts.push_back(-float(a.y()));
          verts.push_back(float(b.x())); verts.push_back(0.0f);
          verts.push_back(-float(b.y()));
        }
      }
      primitive = GL_LINES;
      alpha     = style->alpha > 0.0f ? style->alpha : 0.4f;
    } else if (lyr.kind == QLatin1String("point")) {
      // Single feature with all points (or per-feature; we accept either).
      for (const QVector<QPointF>& bag : lyr.features) {
        for (const QPointF& p : bag) {
          verts.push_back(float(p.x())); verts.push_back(0.0f);
          verts.push_back(-float(p.y()));
        }
      }
      primitive = GL_POINTS;
      alpha     = 1.0f;
      // Three.js sizeAttenuation=true. We approximate by mapping size (m) to
      // pixel size at a reference distance (~60 m camera height). 8 px / m.
      pointSz   = std::max(2.0f, style->size * 8.0f);
    } else {
      // Unknown kind — skip.
      continue;
    }

    if (verts.empty()) continue;

    LayerGL g;
    g.primitive = primitive;
    g.color     = style->color;
    g.alpha     = alpha;
    g.pointSz   = pointSz;
    g.count     = int(verts.size() / 3);
    g.visible   = layerVis_.value(name, true);

    glGenVertexArrays(1, &g.vao);
    glGenBuffers(1, &g.vbo);
    glBindVertexArray(g.vao);
    glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 GLsizeiptr(verts.size() * sizeof(float)),
                 verts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);
    glBindVertexArray(0);

    mapLayers_.insert(name, g);
  }
}

void MapScene::destroyMapLayers() {
  for (auto& g : mapLayers_) {
    if (g.vbo) glDeleteBuffers(1, &g.vbo);
    if (g.vao) glDeleteVertexArrays(1, &g.vao);
  }
  mapLayers_.clear();
}

// ────────────────────────────────────────────────────────────────────────
// Tracks
// ────────────────────────────────────────────────────────────────────────

QColor MapScene::trackColor(const QString& type) const {
  // Mirrors web_hmi types.js OBJ_PALETTE.
  static const QHash<QString, QColor> palette = {
      {QStringLiteral("car"),        QColor("#ff5ea8")},
      {QStringLiteral("truck"),      QColor("#7c3aed")},
      {QStringLiteral("motorcycle"), QColor("#22e09a")},
      {QStringLiteral("pedestrian"), QColor("#ffb547")},
      {QStringLiteral("other"),      QColor("#7a8492")},
      {QStringLiteral("unknown"),    QColor("#7a8492")},
  };
  return palette.value(type, palette.value(QStringLiteral("other")));
}

float MapScene::trackHeight(const QString& type) const {
  // OBJ_DIMS.h
  if (type == QLatin1String("truck"))      return 2.5f;
  if (type == QLatin1String("car"))        return 1.5f;
  if (type == QLatin1String("pedestrian")) return 1.7f;
  if (type == QLatin1String("motorcycle")) return 1.0f;
  return 1.0f;
}

void MapScene::onTracksChanged(const QVector<HmiTrack3D>& tracks) {
  if (!context() || !context()->isValid()) {
    return;   // initializeGL not yet called; tracks before map is unusual.
  }
  makeCurrent();

  // Mark all alive=false; entries we touch this frame become alive again.
  for (auto& s : trackBoxes_)  s.alive = false;
  for (auto& s : trackClouds_) s.alive = false;

  for (const HmiTrack3D& t : tracks) {
    updateOrCreateTrackBox(t);
    updateOrCreateTrackCloud(t);
  }
  pruneTrackSlots(tracks);

  doneCurrent();
  update();
}

void MapScene::updateOrCreateTrackBox(const HmiTrack3D& t) {
  // Bridge sends ego-frame x/y for tracks (x=forward, y=left).
  // We render in world space relative to ego: stored as offsets, the view
  // matrix takes care of world-from-ego.
  // Track group anchored at ego pose with rotation = -yaw (matches three.js
  // TrackBoxes.jsx line 61).
  TrackBoxSlot& slot = trackBoxes_[t.id];
  const float lx  = std::max(0.3f, float(t.sizeX > 0 ? t.sizeX : 1.0f));
  const float ly  = trackHeight(t.type);
  const float lz  = std::max(0.3f, float(t.sizeY > 0 ? t.sizeY : 1.0f));

  if (slot.vao == 0) {
    std::vector<float> verts;
    appendBoxLines(verts, lx, ly, lz, /*yOffset=*/ ly * 0.5f);
    if (showHeading_) {
      appendArrow(verts, std::max(0.5f, float(t.sizeX) * 0.6f), ly * 0.5f);
    }
    slot.count = int(verts.size() / 3);

    glGenVertexArrays(1, &slot.vao);
    glGenBuffers(1, &slot.vbo);
    glBindVertexArray(slot.vao);
    glBindBuffer(GL_ARRAY_BUFFER, slot.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 GLsizeiptr(verts.size() * sizeof(float)),
                 verts.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
  }

  slot.color = trackColor(t.type);
  slot.cx    = t.x;       // ego frame forward (m)
  slot.cz    = t.y;       // ego frame left (m) — translates to ego-local
  slot.yaw   = t.orientation;
  slot.alive = true;
}

void MapScene::updateOrCreateTrackCloud(const HmiTrack3D& t) {
  TrackCloudSlot& slot = trackClouds_[t.id];
  const int numPoints = t.points.size() / 3;

  if (numPoints <= 0) {
    // No points this frame — leave slot for prune to delete (alive stays false).
    return;
  }

  // Allocate / grow VBO if needed (capacity in floats, num*3).
  if (slot.vao == 0 || slot.capacity < numPoints * 3) {
    if (slot.vbo) glDeleteBuffers(1, &slot.vbo);
    if (slot.vao) glDeleteVertexArrays(1, &slot.vao);
    slot.vao = 0; slot.vbo = 0;

    int newCap = 4096 * 3;   // initial cap = 4096 points
    while (newCap < numPoints * 3) newCap *= 2;

    glGenVertexArrays(1, &slot.vao);
    glGenBuffers(1, &slot.vbo);
    glBindVertexArray(slot.vao);
    glBindBuffer(GL_ARRAY_BUFFER, slot.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 GLsizeiptr(newCap * sizeof(float)),
                 nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
    slot.capacity = newCap;
  }

  // Re-pack incoming flat (x,y,z,...) for ego frame:
  //   lidar/base_link:   x=fwd, y=left, z=up  (REP-103)
  //   our world (ego):   x=fwd, y=up,   z=-left
  std::vector<float> repacked(numPoints * 3);
  const float* p = t.points.constData();
  for (int i = 0; i < numPoints; ++i) {
    repacked[i*3 + 0] = p[i*3 + 0];           // forward
    repacked[i*3 + 1] = p[i*3 + 2];           // up
    repacked[i*3 + 2] = -p[i*3 + 1];          // -left → +right (z)
  }
  glBindBuffer(GL_ARRAY_BUFFER, slot.vbo);
  glBufferSubData(GL_ARRAY_BUFFER, 0,
                  GLsizeiptr(repacked.size() * sizeof(float)),
                  repacked.data());
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  slot.count = numPoints;
  slot.color = trackColor(t.type);
  slot.alive = true;
}

void MapScene::pruneTrackSlots(const QVector<HmiTrack3D>& /*tracks*/) {
  // Drop boxes whose track id disappeared.
  for (auto it = trackBoxes_.begin(); it != trackBoxes_.end(); ) {
    if (!it.value().alive) {
      if (it.value().vbo) glDeleteBuffers(1, &it.value().vbo);
      if (it.value().vao) glDeleteVertexArrays(1, &it.value().vao);
      it = trackBoxes_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = trackClouds_.begin(); it != trackClouds_.end(); ) {
    if (!it.value().alive) {
      if (it.value().vbo) glDeleteBuffers(1, &it.value().vbo);
      if (it.value().vao) glDeleteVertexArrays(1, &it.value().vao);
      it = trackClouds_.erase(it);
    } else {
      ++it;
    }
  }
}

void MapScene::destroyTrackSlots() {
  for (auto& s : trackBoxes_) {
    if (s.vbo) glDeleteBuffers(1, &s.vbo);
    if (s.vao) glDeleteVertexArrays(1, &s.vao);
  }
  trackBoxes_.clear();
  for (auto& s : trackClouds_) {
    if (s.vbo) glDeleteBuffers(1, &s.vbo);
    if (s.vao) glDeleteVertexArrays(1, &s.vao);
  }
  trackClouds_.clear();
}

// ────────────────────────────────────────────────────────────────────────
// Ego + grid
// ────────────────────────────────────────────────────────────────────────

void MapScene::buildEgoBox() {
  std::vector<float> verts;
  // Box centered horizontally on origin, sitting on Y=0.
  appendBoxLines(verts, kEgoLength, kEgoHeight, kEgoWidth,
                 /*yOffset=*/ kEgoHeight * 0.5f);
  // Heading arrow: from front-center forward.
  appendArrow(verts, 2.5f, kEgoHeight * 0.5f);
  egoCount_ = int(verts.size() / 3);

  glGenVertexArrays(1, &egoVao_);
  glGenBuffers(1, &egoVbo_);
  glBindVertexArray(egoVao_);
  glBindBuffer(GL_ARRAY_BUFFER, egoVbo_);
  glBufferData(GL_ARRAY_BUFFER,
               GLsizeiptr(verts.size() * sizeof(float)),
               verts.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glBindVertexArray(0);
}

void MapScene::destroyEgoBox() {
  if (egoVbo_) { glDeleteBuffers(1, &egoVbo_); egoVbo_ = 0; }
  if (egoVao_) { glDeleteVertexArrays(1, &egoVao_); egoVao_ = 0; }
  egoCount_ = 0;
}

void MapScene::buildGrid() {
  // 80×80 m grid centred on origin, 1 m spacing — same params as ThreeScene.jsx.
  std::vector<float> verts;
  const int half = 40;
  for (int i = -half; i <= half; ++i) {
    // Lines parallel to X (varying Z)
    verts.push_back(float(-half)); verts.push_back(0.0f); verts.push_back(float(i));
    verts.push_back(float( half)); verts.push_back(0.0f); verts.push_back(float(i));
    // Lines parallel to Z (varying X)
    verts.push_back(float(i)); verts.push_back(0.0f); verts.push_back(float(-half));
    verts.push_back(float(i)); verts.push_back(0.0f); verts.push_back(float( half));
  }
  gridCount_ = int(verts.size() / 3);

  glGenVertexArrays(1, &gridVao_);
  glGenBuffers(1, &gridVbo_);
  glBindVertexArray(gridVao_);
  glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
  glBufferData(GL_ARRAY_BUFFER,
               GLsizeiptr(verts.size() * sizeof(float)),
               verts.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glBindVertexArray(0);
}

void MapScene::destroyGrid() {
  if (gridVbo_) { glDeleteBuffers(1, &gridVbo_); gridVbo_ = 0; }
  if (gridVao_) { glDeleteVertexArrays(1, &gridVao_); gridVao_ = 0; }
  gridCount_ = 0;
}

// ────────────────────────────────────────────────────────────────────────
// Render
// ────────────────────────────────────────────────────────────────────────

void MapScene::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (!progLine_) return;

  // Smooth ego pose toward the latest sample. The data source is now
  // /localization/to_control_team at ~50 Hz (vs the prior 10 Hz /hmi/state
  // path), so kAlpha is bumped to 0.5 — at a 16 ms render tick that gives
  // a ~32 ms time constant, and 50 Hz samples (20 ms apart) are tracked
  // almost 1:1, matching rviz's TF-interpolated camera. yaw unwrapped via
  // atan2(sin Δ, cos Δ) so a +π↔-π wrap doesn't spin the camera.
  if (!smInit_) {
    smEast_  = lastState_.ego.east;
    smNorth_ = lastState_.ego.north;
    smYaw_   = lastState_.ego.yaw;
    smInit_  = true;
  } else {
    constexpr double kAlpha = 0.5;
    smEast_  += (lastState_.ego.east  - smEast_)  * kAlpha;
    smNorth_ += (lastState_.ego.north - smNorth_) * kAlpha;
    const double dy = std::atan2(std::sin(lastState_.ego.yaw - smYaw_),
                                 std::cos(lastState_.ego.yaw - smYaw_));
    smYaw_ += dy * kAlpha;
  }

  const QMatrix4x4 P = projectionMatrix();
  const QMatrix4x4 V = viewMatrix();

  // Ego world position (delta coords); ego-frame draws use this transform.
  const float ex  = float(smEast_  - mapOrigin_.x());
  const float ez  = -float(smNorth_ - mapOrigin_.y());
  const float yaw = float(smYaw_);

  // ── 1) Grid (centered at ego, follows it on Y=0) ───────────────────
  if (gridCount_ > 0) {
    glUseProgram(progLine_);
    QMatrix4x4 M;
    M.translate(ex, 0.0f, ez);
    QMatrix4x4 mvp = P * V * M;
    glUniformMatrix4fv(uLineMvp_, 1, GL_FALSE, mvp.constData());
    QVector4D col(0.055f, 0.169f, 0.239f, 0.6f);   // #0e2b3d-ish
    glUniform4fv(uLineColor_, 1, &col[0]);
    glBindVertexArray(gridVao_);
    glDrawArrays(GL_LINES, 0, gridCount_);
  }

  // ── 2) Map layers (world-space, identity model) ────────────────────
  if (!mapLayers_.isEmpty()) {
    QMatrix4x4 M;     // identity
    QMatrix4x4 mvp = P * V * M;

    for (auto it = mapLayers_.constBegin(); it != mapLayers_.constEnd(); ++it) {
      const LayerGL& g = it.value();
      if (!g.visible) continue;
      const bool wantVisible = layerVis_.value(it.key(), true);
      if (!wantVisible)      continue;
      if (g.count <= 0)      continue;

      if (g.primitive == GL_POINTS) {
        glUseProgram(progPoint_);
        glUniformMatrix4fv(uPointMvp_,  1, GL_FALSE, mvp.constData());
        QVector4D col = toVec4(g.color, g.alpha);
        glUniform4fv(uPointColor_, 1, &col[0]);
        glUniform1f(uPointSize_, g.pointSz);
      } else {
        glUseProgram(progLine_);
        glUniformMatrix4fv(uLineMvp_,   1, GL_FALSE, mvp.constData());
        QVector4D col = toVec4(g.color, g.alpha);
        glUniform4fv(uLineColor_, 1, &col[0]);
      }

      glBindVertexArray(g.vao);
      glDrawArrays(g.primitive, 0, g.count);
    }
  }

  // ── 3) Ego (world-space at delta_e, delta_-n; rotated by -yaw) ─────
  if (egoVao_ && egoCount_ > 0) {
    glUseProgram(progLine_);
    QMatrix4x4 M;
    M.translate(ex, 0.0f, ez);
    M.rotate(qRadiansToDegrees(-yaw), 0.0f, 1.0f, 0.0f);
    QMatrix4x4 mvp = P * V * M;
    glUniformMatrix4fv(uLineMvp_, 1, GL_FALSE, mvp.constData());
    QVector4D col(0.0f, 0.898f, 1.0f, 0.85f);   // EGO_COLOR #00e5ff
    glUniform4fv(uLineColor_, 1, &col[0]);
    glBindVertexArray(egoVao_);
    glDrawArrays(GL_LINES, 0, egoCount_);
  }

  // ── 4) Tracks: box wireframes ─────────────────────────────────────
  if (showBoxes_ || showHeading_) {
    glUseProgram(progLine_);
    for (auto it = trackBoxes_.constBegin(); it != trackBoxes_.constEnd(); ++it) {
      const TrackBoxSlot& s = it.value();
      if (s.count <= 0) continue;
      // trackGroup is anchored at ego pose with parent rotation -yaw.
      // Then each box is at (t.x forward, t.y left) with -orientation rotation.
      // World transform = T_world(ex,0,ez) * R_y(-yaw) * T(t.x,0,-t.y) * R_y(-orient)
      QMatrix4x4 M;
      M.translate(ex, 0.0f, ez);
      M.rotate(qRadiansToDegrees(-yaw), 0.0f, 1.0f, 0.0f);
      M.translate(float(s.cx), 0.0f, -float(s.cz));
      M.rotate(qRadiansToDegrees(-float(s.yaw)), 0.0f, 1.0f, 0.0f);
      QMatrix4x4 mvp = P * V * M;
      glUniformMatrix4fv(uLineMvp_, 1, GL_FALSE, mvp.constData());
      QVector4D col = toVec4(s.color, 0.95f);
      glUniform4fv(uLineColor_, 1, &col[0]);
      glBindVertexArray(s.vao);
      glDrawArrays(GL_LINES, 0, s.count);
    }
  }

  // ── 5) Tracks: point clouds ────────────────────────────────────────
  if (showClouds_) {
    glUseProgram(progPoint_);
    glUniform1f(uPointSize_, std::max(2.0f, pointSize_ * 24.0f));
    for (auto it = trackClouds_.constBegin(); it != trackClouds_.constEnd(); ++it) {
      const TrackCloudSlot& s = it.value();
      if (s.count <= 0) continue;
      QMatrix4x4 M;
      M.translate(ex, 0.0f, ez);
      M.rotate(qRadiansToDegrees(-yaw), 0.0f, 1.0f, 0.0f);
      QMatrix4x4 mvp = P * V * M;
      glUniformMatrix4fv(uPointMvp_, 1, GL_FALSE, mvp.constData());
      QVector4D col = toVec4(s.color, 0.85f);
      glUniform4fv(uPointColor_, 1, &col[0]);
      glBindVertexArray(s.vao);
      glDrawArrays(GL_POINTS, 0, s.count);
    }
  }

  glBindVertexArray(0);
  glUseProgram(0);
}

// ────────────────────────────────────────────────────────────────────────
// State updates / camera
// ────────────────────────────────────────────────────────────────────────

void MapScene::onEgoPoseChanged(double east, double north, double yaw) {
  // 50 Hz path: directly overwrite ego fields used by viewMatrix() and ego
  // box draw. Smoothing in paintGL still applies but at α=0.5 it tracks
  // 20 ms-spaced samples almost instantaneously.
  lastState_.ego.east  = east;
  lastState_.ego.north = north;
  lastState_.ego.yaw   = yaw;
}

void MapScene::onStateChanged(const HmiState& s) {
  // /hmi/state arrives at 10 Hz with the same host_east/north/yaw fields.
  // To avoid two sources fighting (50 Hz onEgoPoseChanged vs 10 Hz here),
  // preserve any newer ego values the 50 Hz stream already set since the
  // last 10 Hz tick. Non-ego fields (speed/gear/mode/...) come only from
  // /hmi/state so they always overwrite.
  HmiState merged = s;
  if (smInit_) {
    merged.ego.east  = lastState_.ego.east;
    merged.ego.north = lastState_.ego.north;
    merged.ego.yaw   = lastState_.ego.yaw;
  }
  lastState_ = merged;
  update();
}

void MapScene::setLayerVisibility(const QHash<QString, bool>& vis) {
  layerVis_ = vis;
  // Update existing GL slots.
  for (auto it = mapLayers_.begin(); it != mapLayers_.end(); ++it) {
    it.value().visible = vis.value(it.key(), true);
  }
  update();
}

void MapScene::setShowBoxes(bool on)   { showBoxes_   = on; update(); }
void MapScene::setShowHeading(bool on) { showHeading_ = on; update(); }
void MapScene::setShowClouds(bool on)  { showClouds_  = on; update(); }
void MapScene::setPointSize(float sz)  { pointSize_   = sz; update(); }

void MapScene::setCameraMode(const QString& mode) {
  if (mode != camMode_) {
    camMode_ = (mode == QLatin1String("top")) ? QStringLiteral("top")
                                              : QStringLiteral("iso");
    update();
  }
}

void MapScene::wheelEvent(QWheelEvent* e) {
  // Mirror three.js CameraController (factor = exp(-deltaY * 0.001)).
  // QWheelEvent::angleDelta in 1/8 of a degree; deltaY gets the y axis.
  const int dy = e->angleDelta().y();
  const float factor = std::exp(float(-dy) * kWheelSens);
  zoom_ = std::clamp(zoom_ * factor, kZoomMin, kZoomMax);
  e->accept();
  update();
}

}  // namespace qt_hmi_widgets
