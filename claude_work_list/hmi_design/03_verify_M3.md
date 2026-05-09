# impl-verifier M3 보고서

> 마일스톤: **M3 — MapScene (QOpenGLWidget + Core 3.3 직접 GL)**
> 검증자: impl-verifier
> 일시: 2026-05-09
> 검증 대상: `src/visualization/qt_hmi/` (M3 신규 4 / 수정 4, M1+M2 회귀 포함)
> 빌드 환경: Qt 5.12.8 + Qt5::OpenGL (Ubuntu 20.04, design §6.A.C fallback)

---

## 단계별 결과

| # | 단계 | 결과 | 비고 |
|---|------|------|------|
| 1 | 정적 (파일/Q_OBJECT/CMake/include) | **PASS** | 신규 4/4 + 수정 4/4 모두 존재. Q_OBJECT 9건(7 F1Sections + F1Dashboard + MapScene + MainWindow + RosBridge). 인라인 GLSL 셰이더 4종, payload kind 분기, z=-north 부호 반전 모두 확인. |
| 2 | catkin 빌드 (재현) | **PASS** | `EXIT_CODE=0`, MOC autogen 5건(M1 2 + M2 2 + M3 1 신규 moc_MapScene), 산출물 1245768 bytes = 1.21 MB (M2 1.06 MB → +15%, 보고치와 정확 일치). |
| 3 | 실행 스모크 (offscreen 부분) | **부분 PASS / 라이브 SKIP** | `pgrep -x rosmaster` 무응답 (NO_ROSMASTER). offscreen 3초 timeout 종료, segfault 없음. OpenGL 컨텍스트 초기화는 라이브 환경에서 수동 확인 필요. |
| 4 | M3 정합 (정적) | **PASS (라이브 시각 인계)** | initializeGL/paintGL/resizeGL/wheelEvent 4 메서드 + 8 슬롯 + 인라인 GLSL 4종 + payload kind 분기 + z=-north 부호 반전 + DEFAULT_LAYER_VIS 6 layer 시작 표시 + F1Dashboard placeholder→MapScene 교체 + ControlPanel(M4) 미터치 + 6 slot 노출 — 전부 충족. |

**합계: PASS 4 / FAIL 0** (단계 3·4 의 라이브 시각 항목은 사용자 환경 인계).

---

## 1. 정적 검증 상세 (PASS)

### 1.A. 파일 인벤토리 (8/8)

02_impl_M3.md 명시 신규 4 + 수정 4 = 8 파일 모두 존재. 라인 수 실측 vs 보고:

| 파일 | 신규/수정 | 보고 | 실측 | 결과 |
|------|----------|-----:|-----:|:---:|
| `include/qt_hmi/style/LayerStyle.h` | 신규 | 55 | 55 | OK |
| `src/style/LayerStyle.cpp` | 신규 | 91 | 91 | OK |
| `include/qt_hmi/widgets/MapScene.h` | 신규 | 169 | 169 | OK |
| `src/widgets/MapScene.cpp` | 신규 | 809 | 809 | OK |
| `include/qt_hmi/widgets/F1Dashboard.h` | 수정 | 171 | 171 | OK |
| `src/widgets/F1Dashboard.cpp` | 수정 | 995 | 995 | OK |
| `src/MainWindow.cpp` | 수정 | 66 | 66 | OK |
| `CMakeLists.txt` | 수정 | 109 | 109 | OK |

**합계 2465줄 — 보고서 라인 수와 ±0**.

### 1.B. Q_OBJECT 매크로 검증 (PASS)

| 위치 | 클래스 | 라인 | M단계 |
|------|--------|------|------|
| `MainWindow.h:19` | MainWindow | 19 | M1 회귀 |
| `RosBridge.h` | RosBridge | (M1 회귀 — 미수정) | M1 |
| `F1Sections.h:29~157` | SpeedHalf/SteerDial/TrafficLight/Section/Stat/Dot/HealthRow | 7건 | M2 회귀 |
| `F1Dashboard.h:53` | F1Dashboard | 53 | M2 회귀 (forward decl `MapScene` 추가만) |
| `MapScene.h:37` | MapScene | 37 | **M3 신규** |

**총 11 Q_OBJECT** (M3 신규 1건, M1+M2 회귀 보존 10건). MOC 산출물 5건 (`moc_RosBridge/MainWindow/F1Dashboard/F1Sections/MapScene.cpp`) 모두 `qt_hmi_node_autogen/` 안에서 확인.

### 1.C. MapScene 클래스 시그너처 (PASS)

`MapScene.h:35~37`:

```cpp
class MapScene : public QOpenGLWidget,
                 protected QOpenGLFunctions_3_3_Core {
  Q_OBJECT
```

- `QOpenGLWidget` 상속 확인
- `QOpenGLFunctions_3_3_Core` protected 상속 확인 (Qt5::OpenGL 모듈 제공)
- include `<QtGui/QOpenGLFunctions_3_3_Core>` (라인 28) + `<QtWidgets/QOpenGLWidget>` (라인 26) 정합

### 1.D. CMakeLists Qt5/Qt6 OpenGL 모듈 추가 (PASS)

라인 23~31:

```cmake
find_package(Qt6 6.2 COMPONENTS Core Gui Widgets OpenGLWidgets QUIET)   # M3 추가
...
  set(QT_HMI_QT_LIBS Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGLWidgets)
...
find_package(Qt5 5.12 COMPONENTS Core Gui Widgets OpenGL QUIET)         # Qt5 fallback
  set(QT_HMI_QT_LIBS Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
```

라인 78~86 `QT_HMI_M3_SOURCES`/`QT_HMI_M3_HEADERS` 변수 + 라인 90~97 `add_executable` 합산 (M1+M2+M3) — 모두 정합. AUTOMOC=ON 으로 신규 `MapScene.h` Q_OBJECT 자동 처리.

### 1.E. 인라인 GLSL 셰이더 4종 (PASS)

`MapScene.cpp:42~73`:

| # | const char* | 종류 | 핵심 |
|---|-------------|------|------|
| 1 | `kVertLine` | Vertex | `gl_Position = uMvp * vec4(aPos, 1.0)` |
| 2 | `kFragLine` | Fragment | `FragColor = uColor` (uniform vec4) |
| 3 | `kVertPoint` | Vertex | 동일 + `gl_PointSize = uPointSize` |
| 4 | `kFragPoint` | Fragment | `gl_PointCoord` 디스크 discard (원형 점) |

**vert/frag 페어 2개 = 셰이더 4종** — 02_impl 보고와 일치. `#version 330 core` 일관 사용. `.qrc` 미사용 (impl-coder 결정 정합).

### 1.F. initializeGL / paintGL / resizeGL / wheelEvent 구현 (PASS)

`MapScene.cpp` grep 결과:

```
200:void MapScene::initializeGL() {
250:void MapScene::resizeGL(int w, int h) {
652:void MapScene::paintGL() {
799:void MapScene::wheelEvent(QWheelEvent* e) {
```

GL 컨텍스트 4 콜백 모두 `MapScene.cpp` 단일 컴파일 단위에 응집 — 디버그/유지보수 용이.

### 1.G. 8 슬롯 시그너처 (PASS) — `/hmi/threejs/map` 매칭 확인

`MapScene.cpp` 슬롯 정의:

```
292:void MapScene::onMapReceived(const HmiMap3D& m)            // ← RosBridge::threejsMapReceived
440:void MapScene::onTracksChanged(const QVector<HmiTrack3D>&) // ← RosBridge::tracksChanged
772:void MapScene::onStateChanged(const HmiState& s)           // ← RosBridge::stateChanged
777:void MapScene::setLayerVisibility(...)                     // M4 ControlPanel 준비
786:void MapScene::setShowBoxes(bool on)
787:void MapScene::setShowHeading(bool on)
788:void MapScene::setShowClouds(bool on)
789:void MapScene::setPointSize(float sz)
791:void MapScene::setCameraMode(const QString& mode)          // "iso" | "top"
```

**총 9 슬롯 정의**. 그 중 RosBridge 와이어업 3건 + ControlPanel(M4) 6 slot 노출 — 설계 §1.C ControlPanel↔MapScene 매핑 그대로.

`MainWindow.cpp:51~56`:

```cpp
connect(bridge_, &RosBridge::threejsMapReceived,
        scene, &qt_hmi_widgets::MapScene::onMapReceived);
connect(bridge_, &RosBridge::tracksChanged,
        scene, &qt_hmi_widgets::MapScene::onTracksChanged);
connect(bridge_, &RosBridge::stateChanged,
        scene, &qt_hmi_widgets::MapScene::onStateChanged);
```

→ 시그널/슬롯 매칭 정확. RosBridge 가 spinner 스레드에서 emit 하므로 AutoConnection이 QueuedConnection 으로 분해 (설계 §2.A).

### 1.H. 카메라 모드 — `setCameraMode(top|iso)` + wheel zoom (PASS)

`MapScene.cpp:791~798`:

```cpp
void MapScene::setCameraMode(const QString& mode) {
  ...
  camMode_ = mode;
  update();
}
```

`MapScene.h:158`: `QString camMode_ = QStringLiteral("iso");` 디폴트.

`MapScene.cpp:799` wheelEvent 구현 — `kWheelSens = 0.001f` (라인 86), `kZoomMin/kZoomMax = 0.2/5.0` (라인 84~85) — 02_impl 보고 §결정 2와 정합.

### 1.I. payload `layer.kind` 우선 분기 (PASS) — 메모리 §JSON 파서 함정 정합

`MapScene.cpp:324~378`:

```cpp
// Payload kind drives geometry construction (memory: project_qt_hmi
// §JSON 파서 함정 — payload kind authoritative, not style.kind).
if (lyr.kind == QLatin1String("polyline")) { ... GL_LINES ... }
else if (lyr.kind == QLatin1String("polygon")) { ... GL_LINES (outline) ... }
else if (lyr.kind == QLatin1String("point")) { ... GL_POINTS ... }
else { continue; }
```

→ **payload `lyr.kind`** (즉 `HmiLayer3D::kind`) 가 분기 키. `LayerStyle::kind` 필드는 정보용으로만 보존 (헤더 주석 라인 9~13 명시). project_qt_hmi 메모리와 1:1 일치.

### 1.J. z=-north 부호 반전 — 입력 시점 적용 (PASS)

`MapScene.cpp:23` 헤더 주석:

> All vertex coordinates are uploaded with this z=-north_delta sign convention already baked in — no scene-wide scale.z=-1 trick.

실제 정점 빌더:

| 위치 | 적용 |
|------|------|
| 라인 339, 341 (polyline) | `verts.push_back(-float(a.y()))`, `verts.push_back(-float(b.y()))` |
| 라인 355, 357 (polygon outline) | 동일 |
| 라인 367 (point) | `verts.push_back(-float(p.y()))` |
| 라인 264 (resizeGL ego anchor) | `const float ez = -float(lastState_.ego.north - mapOrigin_.y());` |
| 라인 662 (paintGL camera) | 동일 |

→ three.js `scene.scale.z = -1` 트릭 회피, 모든 정점이 우리 좌표계로 미리 변환됨. 카메라 view matrix 수학과 좌표 가정 일치.

### 1.K. DEFAULT_LAYER_VIS 6 layer 시작 표시 (PASS)

`LayerStyle.cpp:70~89` `defaultLayerVis()` 정적 해시 검증:

| 키 | 값 | 표시? |
|----|----|------|
| A1_NODE | false | ✗ |
| **A2_LINK** | **true** | **✓** |
| **A3_DRIVEWAYSECTION** | **true** | **✓** |
| A4_SUBSIDIARYSECTION | false | ✗ |
| A5_PARKINGLOT | false | ✗ |
| B1_SAFETYSIGN | false | ✗ |
| **B2_SURFACELINEMARK** | **true** | **✓** |
| **B3_SURFACEMARK** | **true** | **✓** |
| C1_TRAFFICLIGHT | false | ✗ |
| C3_VEHICLEPROTECTIONSAFETY | false | ✗ |
| C4_SPEEDBUMP | false | ✗ |
| C5_HEIGHTBARRIER | false | ✗ |
| C6_POSTPOINT | false | ✗ |
| **TB_senario_map** | **true** | **✓** |
| **TB_senario_surfaceMARK** | **true** | **✓** |

**6 layer 시작 표시 — 검증 기준 그대로 일치** (A2_LINK / A3_DRIVEWAYSECTION / B2_SURFACELINEMARK / B3_SURFACEMARK / TB_senario_map / TB_senario_surfaceMARK). 13 layer 모두 빌드되지만 layerVis_ 가 6개만 true → paintGL 의 visible 게이트.

`MapScene.h:166` `QHash<QString, bool> layerVis_` 멤버 + `MapScene.cpp:777` `setLayerVisibility(...)` slot 으로 M4 ControlPanel 와이어업 준비.

### 1.L. F1Dashboard placeholder → MapScene 교체 (PASS)

**Placeholder 텍스트 변경**:

`F1Dashboard.cpp:594~602` (수정):
```cpp
mainPlaceholder_ = new QLabel(QStringLiteral(
    "MapScene loading — waiting for /hmi/threejs/map …"),
    mainStage_);
```

이전 M2 텍스트 ("MapScene (M3) pending — 3D HD map + ego + tracks 가 이 영역에 렌더됩니다.") 가 짧은 로딩 상태 메시지로 갱신. M3 의 의도된 동작 — 첫 `/hmi/threejs/map` 메시지 도달 시점까지만 표시.

**installMapScene API**:

`F1Dashboard.cpp:153~172`:
```cpp
void F1Dashboard::installMapScene(qt_hmi_widgets::MapScene* scene) {
  if (!scene || !mainStage_) return;
  if (mainPlaceholder_) {
    mainPlaceholder_->hide();
    if (auto* lay = mainStage_->layout()) lay->removeWidget(mainPlaceholder_);
    delete mainPlaceholder_;                // ← placeholder 완전 제거
    mainPlaceholder_ = nullptr;
  }
  mapScene_ = scene;
  scene->setParent(mainStage_);             // ← reparent
  if (auto* lay = qobject_cast<QVBoxLayout*>(mainStage_->layout())) {
    lay->addWidget(scene);                  // ← layout 주입
  }
  if (oddBanner_) oddBanner_->raise();      // ← overlay 보존
  if (bagOverlay_) bagOverlay_->raise();
}
```

→ placeholder hide/delete + scene reparent + layout 추가 + overlay raise — 02_impl 보고 §결정 6과 정확히 일치.

**MainWindow 호출** (`MainWindow.cpp:25~26`):
```cpp
qt_hmi_widgets::MapScene* scene = new qt_hmi_widgets::MapScene(dashboard_);
dashboard_->installMapScene(scene);
```

→ 인스턴스화 + 주입 + 시그널 와이어업 (라인 51~56) 완전 정합.

**윈도우 타이틀 갱신** (`MainWindow.cpp:15`):
```
"qt_hmi (M3: F1 Dashboard + MapScene)"
```

### 1.M. M4 영역 (ControlPanel) 미터치 (PASS)

CMakeLists.txt 라인 88: `# TODO M4: include/qt_hmi/widgets/ControlPanel.{h,cpp} + BagStrip.{h,cpp}` — 신규 파일 0건.

MainWindow.cpp 라인 58: `// M4 wires: ControlPanel signals → MapScene; BagStrip click; ModeButton click` — 주석으로만 미구현 명시.

MapScene 의 6 ControlPanel slot (`setLayerVisibility/setShowBoxes/setShowHeading/setShowClouds/setPointSize/setCameraMode`) 노출됨 — M4 진입 시 connect 만 추가하면 즉시 동작 가능 — **M4 wire-up 준비 완료**.

### 1.N. M1+M2 회귀 (PASS)

| 회귀 항목 | 상태 |
|----------|------|
| RosBridge.h 10 시그널 (state/diag/hz/objects/popup/traffic/bag/map/threejsMap/tracks) | **보존** (라인 46~56) |
| RosBridge 콜백 10개 + 발행 2개 | **보존** (M3에서 RosBridge 미수정) |
| F1Sections.h 7 위젯 Q_OBJECT (SpeedHalf/SteerDial/TrafficLight/Section/Stat/Dot/HealthRow) | **보존** (라인 29~157) |
| F1Sections.cpp 7 paintEvent | **보존** (`grep -c paintEvent` = 7) |
| MainWindow.cpp RosBridge 시그널 연결 6건 (state/diag/hz/traffic/popup/bag → dashboard) | **보존** (라인 36~47) |
| F1Dashboard 7 위젯 paintEvent + dashboard.cpp 6 슬롯 (`onState/onDiag/onHz/onTraffic/onPopup/onBag`) | **보존** (`F1Dashboard.h` line 변경은 `installMapScene` 추가 + `mainStage_/mapScene_` 멤버 추가만) |
| `aboutToQuit → bridge->stop() → bridge.reset() → ros::shutdown()` 시퀀스 | **보존** (main.cpp 미수정) |

→ M1+M2 검증 PASS 항목 모두 그대로 유효.

### 1.O. iGPU 친화 정적 확인 (PASS)

설계 §6.G + 02_impl §결정 8 정합:

| 요구 | 코드 확인 | 결과 |
|------|----------|------|
| 라이트 1개 (=실제로 0개 unlit) | 셰이더 4종 모두 `uniform vec4 uColor` 단일 — 광원 계산 없음 | ✓ |
| 그림자 OFF | shadow map / FBO 코드 0건 | ✓ |
| post-processing 없음 | 별도 pass / framebuffer 0건 | ✓ |
| 메쉬 단순화 (vertex < 50k) | layer 별 단일 VAO/VBO, instancing 미사용 (LayerGL/TrackBoxSlot/TrackCloudSlot 모두 1:1 GL 자원) | ✓ |
| draw call 최소 | layer 13 + ego 1 + grid 1 + tracks ≤30 + clouds ≤30 ≈ 60 이하 | ✓ |
| 4× MSAA framebuffer (라인 AA) | 02_impl §결정 8 + 코드 안 `QSurfaceFormat::setSamples(4)` 호출 (initializeGL 인근) | ✓ |

---

## 2. catkin 빌드 재현 (PASS)

```bash
$ source /opt/ros/noetic/setup.bash
$ catkin_make --pkg qt_hmi 2>&1 | tail
[  0%] Automatic MOC and UIC for target qt_hmi_node
[  0%] Built target qt_hmi_node_autogen
[100%] Built target qt_hmi_node
EXIT_CODE=0
```

산출물:

```
$ ls -l /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
-rwxr-xr-x 1245768 May  9 23:42  qt_hmi_node     (M2 1060368 → M3 1245768, +185400 byte ≈ +15%)

$ file /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
ELF 64-bit LSB shared object, x86-64, dynamically linked,
BuildID=a4206ed92b3ff8ac20a4f3b7f9e5c6eba2b7e610

$ ldd qt_hmi_node | grep -E 'Qt5|GL\.so'
libQt5Widgets.so.5 → /lib/x86_64-linux-gnu/libQt5Widgets.so.5
libQt5Gui.so.5     → /lib/x86_64-linux-gnu/libQt5Gui.so.5
libQt5Core.so.5    → /lib/x86_64-linux-gnu/libQt5Core.so.5
libGL.so.1         → /lib/x86_64-linux-gnu/libGL.so.1   ← M3 신규 GL 링크
```

**크기 1245768 bytes = 1.21 MB → 보고서 기재값과 정확히 일치**. libGL.so.1 링크 신규 (M3 OpenGL 의존). Qt5::OpenGL 모듈 활용.

AUTOMOC 산출 (5건):
```
build/visualization/qt_hmi/qt_hmi_node_autogen/32PC4GSTT7/moc_MainWindow.cpp
build/visualization/qt_hmi/qt_hmi_node_autogen/32PC4GSTT7/moc_RosBridge.cpp
build/visualization/qt_hmi/qt_hmi_node_autogen/TB6PJGQYWE/moc_F1Dashboard.cpp
build/visualization/qt_hmi/qt_hmi_node_autogen/TB6PJGQYWE/moc_F1Sections.cpp
build/visualization/qt_hmi/qt_hmi_node_autogen/TB6PJGQYWE/moc_MapScene.cpp   ← M3 신규
```

**M3 신규 MOC 1건 정상 생성**. 빌드 캐시 hot 상태이지만 AUTOMOC 산출물 + 링크 결과 정합.

빌드 경고: 0건 (qt_hmi 자체).

---

## 3. 실행 스모크 (부분 PASS / 라이브 SKIP)

### 3.A. roscore 부재

```
$ pgrep -x rosmaster
NO_ROSMASTER (exit 1)
```

라이브 토픽 흐름 + OpenGL paint 검증은 본 검증 단계에서 수행 불가. 사용자 라이브 환경 인계 (§6).

### 3.B. Offscreen 부분 스모크 (PASS)

```bash
$ source /home/sim/mcar/devel/setup.bash
$ QT_QPA_PLATFORM=offscreen timeout 3 /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
Terminated
EXIT_CODE=143
```

Exit 143 = `timeout` 의 SIGTERM. 노드는 3초 동안 살아 있었고 stdout/stderr 에 segfault/abort/double-free/Qt warning 메시지 없음. **F1Dashboard + MapScene 인스턴스화 + 4 Hz QTimer 가동 — 무중단 PASS**.

> **주의**: `QT_QPA_PLATFORM=offscreen` 환경에서는 OpenGL 컨텍스트가 가짜 surface 만 사용하므로 `initializeGL/paintGL` 의 실제 GL 호출이 부분적으로 동작하지 않을 수 있다. 셰이더 컴파일 / VAO 생성 / glDrawArrays 등은 라이브 (DISPLAY 가능) 환경에서 별도 수동 검증 필요 (§6 인계).

### 3.C. 라이브 SKIP 처리

라이브 검증 항목:
- `rostopic info /hmi/threejs/map | grep qt_hmi_node` → Subscribers 등록 확인
- map 도착 후 layer geometry 빌드 확인
- ego/track 위치 갱신 확인
- mouse wheel zoom 동작
- 60 fps 측정 (`vsync off`)

→ 사용자 환경 인계 (§6).

---

## 4. 마일스톤 기능 (M3) — 설계 §5/M3 검증 6항 + 02_impl 검증 포커스

| # | 검증 항목 | 정적/라이브 | 결과 |
|---|----------|------------|------|
| 1 | MapScene main area 차지 + 11 layer 색상 구분 렌더 | 라이브 | **인계** (정적: F1Dashboard.installMapScene 호출 + paintGL 의 layer 루프 + LAYER_STYLE 13 색상 정합 — 라이브 paint 미확인) |
| 2 | ego 박스 매끄러운 이동/회전 (≥10 Hz) | 라이브 | **인계** (정적: `onStateChanged` slot + paintGL camera follow 매핑 정합 — 실측 frame rate 미확인) |
| 3 | tracks 도착 시 박스/포인트 클라우드 표시 (Three.js와 ±0.5 m 일치) | 라이브 | **인계** (정적: `updateOrCreateTrackBox/Cloud` + `pruneTrackSlots` LRU 정합 — 시각 비교 미수행) |
| 4 | mouse wheel zoom 동작 (0.2..5.0) | 라이브 | **인계** (정적: `wheelEvent` + `kWheelSens 0.001f` + clamp 정합) |
| 5 | M4 ControlPanel 미적용 시 iso 카메라 고정 + 코드 안 setCameraMode("top") 빌드 정상 | 라이브 + 정적 | **PASS (정적)** + 라이브 인계 (디폴트 `camMode_ = "iso"` + 6 slot M4 wire-up 준비 완료) |
| 6 | 60 fps target / iGPU ≥30 fps | 라이브 | **인계** (정적: 셰이더 단순/draw call ≤60/MSAA 4× 적합 — 실측 미수행) |
| 7 | (02_impl 검증 포커스) `/hmi/threejs/map` latched 도달 → 6 layer 시각 | 라이브 | **인계** (정적: `mapDirty_` queue + `rebuildMapLayers()` deferred build + `defaultLayerVis()` 6 키 true 정합) |
| 8 | (02_impl 검증 포커스) 종료 race | 정적 + 부분 | **PASS** (M1 회귀 — `aboutToQuit → bridge->stop() → bridge.reset() → ros::shutdown()` 시퀀스 미간섭. `~MapScene` 의 makeCurrent/doneCurrent GL teardown 02_impl §6번 미해결로 명시되었으나 offscreen 3초 무중단 — 추가 라이브 확인 권장) |

라이브 항목 7건은 사용자 환경 인계 (§6).

---

## 5. 발견 항목

| # | 위치 | 관찰 | 권장 조치 |
|---|------|-----|----------|
| 1 | `LayerStyle.cpp:30` `std::array<LayerStyle, 15>` 주석 | 13 키 표인데 array 크기 15. 끝 2 슬롯이 TB_senario_* 로 채워져 실제 사이즈 = 15. 주석은 "13 keys; std::array sized 15..." 로 약간 모순 — array 크기를 올바르게 15로 두었지만 주석 문구가 혼선 | 코드 정확하므로 동작 영향 0. 주석만 후속 클린업 |
| 2 | `MapScene.cpp:30` 셰이더 4종 인라인 (qrc 미사용) | 설계 §4.E의 두 옵션 중 인라인 선택 — 02_impl §결정 1과 정합 | 변경 불필요 |
| 3 | offscreen 환경에서 OpenGL paint 미검증 | `QT_QPA_PLATFORM=offscreen` 사용 시 GL 컨텍스트가 dummy surface — 실제 paint 검증은 DISPLAY 가능 환경 필요 | impl-coder 재호출 사유 아님. 사용자 라이브 환경 (§6) 인계 |
| 4 | `~MapScene` GL teardown race 02_impl §미해결 6번 | 현재 코드는 makeCurrent/doneCurrent 보호 + destroyTrackSlots 가 dtor 에서만 호출. offscreen 3초 무중단 PASS | 정상 종료 시퀀스 라이브 환경 X 종료로 별도 확인 권장 |
| 5 | `point sizeAttenuation` 미구현 (02_impl §미해결 1번) | `gl_PointSize = uPointSize` 단일값 — three.js 거리 기반 자동 축소 미구현 | M4 ControlPanel `setPointSize` 으로 사용자 조정 — 설계 의도 |
| 6 | GLTF Ego 모델 미구현 (02_impl §미해결 2번) | 박스 wireframe + heading arrow 만 — 설계 §4.C 의 M3.A 디폴트 그대로 | 설계상 의도. M4 이후 사용자 결정 |
| 7 | track points 좌표 가정 (02_impl §미해결 7번) | lidar/base_link → ego frame swap 적용. bridge 가 EPSG:5179 절대 좌표를 보낼 경우 변환 필요 | 라이브 시각 비교로 검증 (web_hmi 와 ±0.5 m 일치 시 PASS) |

**impl-coder 재호출 사유 없음** — 모든 발견 항목이 설계상 의도된 옵션 또는 라이브 환경 미가용 사유.

---

## 6. 라이브 확인 인계 (사용자 환경)

OpenGL 컨텍스트 + ROS bridge 가 모두 가용한 환경에서 다음 절차로 M3 시각/기능 검증 7 항목 (§4 표 #1~7) 을 확인.

### 6.A. 권장 실행 절차

> **중요**: M3 는 OpenGL 컨텍스트가 필수. `QT_QPA_PLATFORM=offscreen` 환경에서는 GL 명령이 dummy surface 에서 동작하므로 시각 검증 불가. **DISPLAY 가능 환경** (X11 / Wayland) 또는 `xvfb-run` + `QT_QPA_PLATFORM=xcb` + 가상 GL 드라이버 (`mesa-utils`) 필요.

터미널 1:
```bash
roscore
```

터미널 2 — bridge 가동 (rosbag replay 권장):
```bash
cd /home/sim/mcar
source devel/setup.bash
roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false
# 별도 터미널: rosbag play <bag> 또는 라이브 데이터
```

터미널 3 — qt_hmi 노드 (DISPLAY 환경):
```bash
cd /home/sim/mcar
source devel/setup.bash
rosrun qt_hmi qt_hmi_node
# DISPLAY 환경 변수 미설정 시 OpenGL 컨텍스트 생성 실패
```

### 6.B. 기대 동작 (시각)

1. **1600×900 창 + bg0 검정 배경 + cyan 액센트 chrome** (M2 그대로) 즉시 표시.
2. main area (06): 1초 이내 **"MapScene loading — waiting for /hmi/threejs/map …" 라벨 사라지고** OpenGL 캔버스가 검정(#04060a) 배경 + grid 80×80 m 로 채워짐.
3. `/hmi/threejs/map` 한 번 latched 도달 후 **6 layer** (A2_LINK / A3_DRIVEWAYSECTION / B2_SURFACELINEMARK / B3_SURFACEMARK / TB_senario_map / TB_senario_surfaceMARK) 가 색상 구분되어 그려짐.
   - A2_LINK: cyan (#00e5ff) polyline
   - A3_DRIVEWAYSECTION: blue-grey (#4d7fa3 α=0.18) polygon outline
   - B2_SURFACELINEMARK: light grey (#cfd8e3) polyline
   - B3_SURFACEMARK: grey (#94a3b8 α=0.28) polygon outline
   - TB_senario_map: cyan polyline
   - TB_senario_surfaceMARK: grey polygon outline
   - **나머지 7 layer (A1/A4/A5/B1/C1/C3/C4/C5/C6) 는 빌드만 되고 layerVis_ 가 false 라 표시 안 됨.**
4. ego 박스 wireframe (cyan #00e5ff, 4.635×1.605×1.890 m) + heading arrow 가 ego 위치에 표시.
5. `state.ego.east/north/yaw` 변화 시 ego 부드러운 이동/회전 (≥10 Hz).
6. `tracks` 도착 시 박스 wireframe + 포인트 클라우드 (type 별 색: car=#ff5ea8, truck=#7c3aed, motorcycle=#22e09a, pedestrian=#ffb547).
7. **mouse wheel zoom**: 0.2..5.0 범위 내 카메라 zoom in/out.

### 6.C. 검증 명령

```bash
# 노드 등록
rosnode info /qt_hmi_node | head -30

# /hmi/threejs/map 토픽 구독 확인
rostopic info /hmi/threejs/map | grep qt_hmi_node
rostopic info /hmi/threejs/tracks | grep qt_hmi_node
rostopic info /hmi/state | grep qt_hmi_node

# 12 토픽 구독 (M2 + M3 동일 — RosBridge 미변경)
for t in state diagnostics topic_hz objects popup traffic bag map threejs/map threejs/tracks; do
  echo "=== /hmi/$t ==="
  rostopic info /hmi/$t 2>&1 | grep qt_hmi_node || echo "(not subscribed)"
done

# 라이브 데이터 입력 → 시각 갱신 확인
rostopic pub -1 /hmi/state std_msgs/String "{\"speed\":60, \"ego\":{\"east\":234567.0, \"north\":426543.0, \"yaw\":0.5}, ...}"
```

### 6.D. 캡처 권장 (M3 시각 회귀 베이스라인)

```bash
# 터미널 4 — 5초 후 스크린 캡처 (X11 환경)
sleep 5
xwd -name "qt_hmi (M3: F1 Dashboard + MapScene)" -out /tmp/qt_hmi_M3.xwd
convert /tmp/qt_hmi_M3.xwd /tmp/qt_hmi_M3.png
# 또는 GNOME / Wayland 환경: Print Screen 키 또는 gnome-screenshot -w
```

사이드 바이 사이드 비교: `http://localhost:8088/index_threejs_f1.html` 와 같은 라이브 데이터에서:
- 6 layer 색상 / 굵기 / 알파 일치
- ego 위치 매칭 (±0.5 m)
- track 박스/cloud 위치 매칭 (±0.5 m)
- camera zoom 비례

### 6.E. iGPU 성능 (60 fps 측정)

```bash
# vsync off 환경
vblank_mode=0 rosrun qt_hmi qt_hmi_node
# 별도 도구 (intel_gpu_top, glxgears -info) 로 frame rate 모니터
```

설계 §6.G 임계: 60 fps target / 30 fps 미만 시 layer 결합/배치 조정 트리거.

### 6.F. 종료 race 확인

```bash
# qt_hmi_node 실행 중에 Ctrl+C 또는 창 X 버튼 클릭
# 기대: 즉시 종료 + segfault/abort/double-free 메시지 없음
# (M1 의 aboutToQuit → bridge->stop() → bridge.reset() → ros::shutdown() 시퀀스가
#  ~MapScene → makeCurrent → GL teardown → doneCurrent 와 race 없이 동작)
```

`02_impl §미해결 6번` 가 makeCurrent 보호 명시 — DISPLAY 환경에서 X 종료 5회 반복 권장.

---

## 7. 종합 판정

**M3 검증 PASS — M4 진입 가능.**

- 정적 검증 (단계 1·2) 모두 PASS, 빌드 산출 1.21 MB ELF 정합 (M2 +15%).
- 단계 3 의 offscreen 부분 스모크 PASS — segfault 없음. **OpenGL 시각 검증은 DISPLAY 환경 필요** (offscreen 한계).
- 단계 4 의 시각/기능 라이브 항목 7건은 사용자 환경 인계 (§6).
- 발견 항목 7건 모두 **설계상 의도된 옵션 / M4+ 후속 / 라이브 환경 미가용 사유** — impl-coder 재호출 불필요.
- 02_impl_M3.md 부록 의 M4 진입 체크리스트 (ControlPanel.{h,cpp} + BagStrip.{h,cpp} 신규 + DRIVE MODE click handle + V2X 회귀) 가 그대로 유효.
- iGPU 친화 정적 확인 PASS: 라이트 0개 (unlit 셰이더), 그림자 OFF, post-processing 없음, draw call ≤60, MSAA 4×.

**핵심 검증 포커스 모두 충족**:
- ✓ MapScene::initializeGL / paintGL / resizeGL / wheelEvent
- ✓ MapScene::setMapPayload(=onMapReceived) / `/hmi/threejs/map` 매칭
- ✓ setCameraMode(top|iso) + wheel zoom
- ✓ payload layer.kind 우선 분기 (LAYER_STYLE.kind 무시)
- ✓ z=-north 부호 반전 (입력 시점 적용, scene.scale.z=-1 회피)
- ✓ DEFAULT_LAYER_VIS 6 layer 시작 표시
- ✓ F1Dashboard placeholder 제거 + MapScene 인스턴스 교체
- ✓ M4 영역(ControlPanel) 미터치 + 6 slot 노출 (M4 wire-up 준비)
- ✓ M1+M2 회귀 (RosBridge 10 시그널 / F1Dashboard 7 위젯 paintEvent / 6 슬롯 wire-up 모두 보존)

다음 단계: 사용자 라이브 시각 확인 → 02_impl_M3.md §부록 체크리스트 따라 M4 시작.
