# impl-coder M3 보고서

> 마일스톤: **M3 — MapScene (QOpenGLWidget + Core 3.3 직접 GL)**
> 작성: impl-coder
> 일시: 2026-05-09
> 빌드 환경: Qt 5.12.8 + Qt5::OpenGL (Ubuntu 20.04). Mesa iGPU (NUC) 호환 GL 3.3 Core profile, 4× MSAA.

---

## 작성/수정 파일

신규 4개, 수정 4개.

| 파일 | 신규/수정 | 줄수 | 역할 |
|------|----------|-----:|------|
| `src/visualization/qt_hmi/include/qt_hmi/style/LayerStyle.h` | 신규 | 55 | 13 layer table 시그너처 (`LayerStyle{name,color,width,size,alpha,kind}`) + accessor (`layerStyleTable/Count/find`) + `defaultLayerVis()`. payload kind 우선 정책을 헤더 주석에 기록. |
| `src/visualization/qt_hmi/src/style/LayerStyle.cpp` | 신규 | 91 | 13 layer 컬러/굵기/사이즈/알파 (types.js LAYER_STYLE 1:1 이식) + DEFAULT_LAYER_VIS 13 키. `std::array<LayerStyle,15>` 함수-로컬 static. |
| `src/visualization/qt_hmi/include/qt_hmi/widgets/MapScene.h` | 신규 | 169 | `class MapScene : QOpenGLWidget, QOpenGLFunctions_3_3_Core`. 슬롯 8개 (`onMapReceived/onTracksChanged/onStateChanged/setLayerVisibility/setShowBoxes/setShowHeading/setShowClouds/setPointSize/setCameraMode`). 멤버: `LayerGL/TrackBoxSlot/TrackCloudSlot` 3 구조체 + 카메라 상태. |
| `src/visualization/qt_hmi/src/widgets/MapScene.cpp` | 신규 | 809 | GL 컨텍스트 초기화, 셰이더 4종 인라인, 카메라 iso/top, ego 박스, 트랙 박스/포인트 클라우드, wheel zoom, payload kind 분기 빌더. |
| `src/visualization/qt_hmi/include/qt_hmi/widgets/F1Dashboard.h` | 수정 | 171 | `installMapScene()` 메서드 + `mainStage_/mapScene_` 멤버 추가. `qt_hmi_widgets::MapScene` forward decl. |
| `src/visualization/qt_hmi/src/widgets/F1Dashboard.cpp` | 수정 | 995 | `buildMainArea` 내 stage를 `mainStage_`로 노출 + placeholder 라벨 텍스트 갱신 + `installMapScene` 정의 (placeholder hide/delete → MapScene reparent → overlay raise). |
| `src/visualization/qt_hmi/src/MainWindow.cpp` | 수정 | 66 | MapScene 인스턴스화 + `installMapScene` 호출 + RosBridge 3 시그널 (`threejsMapReceived/tracksChanged/stateChanged`) → MapScene 슬롯 연결. 윈도우 타이틀 "M3" 갱신. |
| `src/visualization/qt_hmi/CMakeLists.txt` | 수정 | 109 | Qt6:OpenGLWidgets / Qt5:OpenGL 모듈 추가. `QT_HMI_M3_SOURCES`/`HEADERS` 변수 + `add_executable` 합산. |

**M3 신규 코드 1124줄, 수정 합산 약 25줄.**

---

## 핵심 결정

### 1. 셰이더 4종 인라인 const char* (qrc 미사용)

설계 §4.E "embedded `static const char*` 또는 `:/shaders/...` Qt 리소스" 두 가지 옵션 중 인라인 선택. 이유:
- 셰이더 파일 4개 + qrc + 빌드 종속성 추가가 14줄 vert/frag 한 페어를 위해 과함.
- AUTORCC가 이미 enable인데도 별도 .qrc 없는 게 더 단순.
- M3 GL 코드 단일 컴파일 단위로 응집 → 디버그 용이.

셰이더 종류: line.{vert,frag} (uniform color), point.{vert,frag} (uniform color + uPointSize + 원형 disk discard). 즉 **vert/frag 페어 2개 = 셰이더 4종**.

### 2. 카메라: 설계 §4.D 그대로 + z=-north 부호 반전 입력 시점에 적용

three.js의 `scene.scale.z = -1` 트릭을 회피하기 위해 모든 정점 업로드 시 `z = -delta_north`로 부호를 미리 반전. 카메라 view matrix 계산도 동일 좌표 가정 (eg. `ez = -(ego.north - origin.y)`). 결과적으로 web_hmi와 화면상 동일하지만 GL 좌표는 right-handed, scene scale 변형 없음 → 후속 좌표계 분석이 한결 단순.

iso (default) / top 두 모드 지원. wheel scroll로 zoom (factor `exp(-deltaY*0.001)`, clamp 0.2..5.0) — three.js CameraController 식 그대로.

### 3. payload `layer.kind` 우선, types.js LAYER_STYLE.kind는 무시 (메모리 §JSON 파서 함정)

`MapScene::rebuildMapLayers()` 분기가 `lyr.kind` (페이로드)에 의존. `LayerStyle` 헤더의 `LayerKind` 필드는 정보용에 그치고, GL 빌드 코드는 실제로 그것을 읽지 않는다. 헤더 주석에도 "payload.kind authoritative"로 명시. project_qt_hmi 메모리 §JSON 파서 함정과 일치.

### 4. point primitive size 매핑: types.js size(m) → 픽셀 8 px/m

three.js `PointsMaterial.sizeAttenuation=true`는 거리 기반 자동 축소를 하지만, GL_POINTS는 `gl_PointSize` (픽셀 단위) 단일값. 60 m 카메라 높이 기준 1 m 정점이 ~8 px 정도 보이는 비율로 매핑 (`max(2, style->size * 8.0)`). top/iso 모드 전환 시 스케일이 살짝 어긋나지만 — 마커 크기는 시각적 단서일 뿐 측정용이 아니라 허용 오차 안. M4 ControlPanel 시각 토글로 재조정 가능.

### 5. 트랙 박스 + 포인트 클라우드 — 슬롯 캐싱 + LRU 단순 prune

매 onTracksChanged 호출마다 모든 슬롯의 `alive=false` 마킹 → 입력 트랙 id별 update/create → 끝에 `alive=false`인 슬롯 GL delete. three.js TrackBoxes.jsx의 set seen 패턴 그대로. 박스 VAO 생성 시 size_x/size_y 기반 BoxGeometry; 추후 size 변경 시 새 vertex로 다시 build (포인터 교체) — 시각적 글리치 1프레임 발생 가능하지만 트랙 리시즈는 드물어 무시.

포인트 클라우드: 초기 capacity 4096, overflow 시 next pow2 grow + glBufferSubData 매 프레임 업로드. lidar/base_link (REP-103 x=fwd, y=left, z=up) → 우리 ego frame (x=fwd, y=up, z=-left)으로 입력 시 swap.

### 6. F1Dashboard placeholder → MapScene 교체 = `installMapScene(scene*)` API

placeholder QLabel을 직접 hide+delete + `mapStage_->layout()->addWidget(scene)` + `bagOverlay_/oddBanner_->raise()`. 이로써 MapScene이 stage 컨테이너 layout에 들어가도 overlay 두 개는 항상 위에 표시 (Qt z-order는 mainStage_의 child 순서에 따라 raise()로 강제 가능).

`mainStage_` 변수를 멤버로 export — buildMainArea 내부의 local `stage` 별칭은 다른 코드 경로(StageFilter 이벤트 필터 등)와의 호환을 유지하기 위해 그대로 둠.

### 7. M3에서 layer visibility 적용 = DEFAULT_LAYER_VIS만 (M4 ControlPanel은 손대지 않음)

`MapScene` 생성자에서 `layerVis_ = qt_hmi_style::defaultLayerVis()` 한 번 호출 — 11 + 2 layer 시작 visibility는 web_hmi `DEFAULT_LAYER_VIS`와 1:1. 추후 M4에서 ControlPanel이 `setLayerVisibility(QHash<QString,bool>)` slot을 호출하면 즉시 GL slot의 `visible` 플래그가 갱신. M3 기본 작동 시점에는 A2_LINK / A3_DRIVEWAYSECTION / B2_SURFACELINEMARK / B3_SURFACEMARK / TB_senario_map / TB_senario_surfaceMARK 6개 layer만 보임.

### 8. 4× MSAA 활성, 라이트 0개, post-processing 없음 (iGPU 친화)

설계 §6.G 요구사항 그대로 — `QSurfaceFormat::setSamples(4)`로 framebuffer MSAA 4× (라인 AA에 충분한 수준). `LIGHTING` 없음 (모든 셰이더 unlit, uniform color), 그림자 없음, post-processing 없음. draw call 수: layer 13 + ego 1 + grid 1 + tracks N + clouds N (≤30 트랙 가정 시 60 draw call 이하).

---

## 빌드 검증 (자체 시도)

```
$ source /opt/ros/noetic/setup.bash
$ catkin_make --pkg qt_hmi
[  0%] Automatic MOC and UIC for target qt_hmi_node
[  0%] Built target qt_hmi_node_autogen
Scanning dependencies of target qt_hmi_node
[  0%] Building CXX object .../src/main.cpp.o
[  0%] Building CXX object .../src/RosBridge.cpp.o
[100%] Building CXX object .../src/MainWindow.cpp.o
[100%] Building CXX object .../src/widgets/F1Sections.cpp.o
[100%] Building CXX object .../src/widgets/F1Dashboard.cpp.o
[100%] Building CXX object .../src/widgets/MapScene.cpp.o
[100%] Building CXX object .../src/style/LayerStyle.cpp.o
[100%] Linking CXX executable /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
[100%] Built target qt_hmi_node
```

**결과: PASS** (exit 0). gcc-9.4.0, C++17, Qt 5.12.8, Qt5::OpenGL.

산출 binary:

```
$ ls -l /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
-rwxr-xr-x  1245768  May  9 23:42  qt_hmi_node     (M2 1.06 MB → M3 1.21 MB, +15%)
```

빌드 경고: 없음 (qt_hmi 자체).

### 첫 시도 실패 → 수정한 1건

| 분류 | 증상 | 원인 | 수정 |
|------|------|------|------|
| 컴파일 에러 (헤더 누락) | `'qRadiansToDegrees' was not declared in this scope` (3곳) | `<QtCore/QtMath>` 미include — F1Dashboard.cpp는 이미 포함하지만 MapScene.cpp는 새로 작성했고 빠뜨림. | `#include <QtCore/QtMath>` 추가. |

수정 후 재빌드 PASS — 실제 시도는 2회.

---

## 다음 마일스톤 인계 (impl-verifier로)

### 검증 포커스

1. **MapScene 라이브 검증**: web_hmi 환경 (`roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false`) + qt_hmi_node 실행 시:
   - F1 main area에 OpenGL 캔버스가 표시되고 검정 배경(#04060a)이 채워짐.
   - 1초 내 grid 80×80 m가 ego 위치 중심에 표시.
   - `/hmi/threejs/map` 한 번 latched 도착 후 6개 layer (A2_LINK / A3_DRIVEWAYSECTION / B2_SURFACELINEMARK / B3_SURFACEMARK / TB_senario_map / TB_senario_surfaceMARK)가 색상 구분되어 그려짐. 13개 layer 모두 build됐지만 `layerVis_` 기본값이 6개만 true.
   - ego 박스 wireframe (cyan #00e5ff, 4.635×1.605×1.890 m) + heading arrow가 ego 위치에 표시.
   - `state.ego.east/north/yaw`가 변하면 ego가 매끄럽게 이동/회전.
   - `tracks` 도착 시 트랙별 박스 wireframe이 ego 기준 보정 위치에 표시. 색상은 type별 (car=#ff5ea8, truck=#7c3aed, etc.).
   - mouse wheel zoom 동작 (zoom 0.2..5.0 범위).
2. **카메라 모드**: 설계 §4.D에 따라 코드 안 `camMode_ = "iso"` 디폴트. M4 ControlPanel 미적용 시 iso만 사용. 코드에서 `setCameraMode("top")` 호출 가능 — M4 검증 시 시연.
3. **종료 race**: 창 닫기 시 `~MapScene` → `makeCurrent` → GL teardown → segfault 없음 확인. M3에서 `RosBridge::stop` → `~RosBridge` → `~MainWindow` 순서 유지하기 때문에 GL context destroy가 RosBridge 콜백과 겹치지 않아야 함.
4. **iGPU 성능**: `vsync off` 상태에서 60 fps 측정. layer 6 visible + ego + 트랙 0 → 60 fps 목표. 트랙 30 + cloud 1k → ≥30 fps 목표.

### 라이브 확인 권장 시나리오

```bash
# 터미널 1
roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false

# 터미널 2 (qt_hmi)
source /home/sim/mcar/devel/setup.bash
rosrun qt_hmi qt_hmi_node
```

기대:
1. 1600×900 창 + F1 chrome (M2와 동일)
2. main area placeholder 라벨 1초 이내 사라지고 (M3) GL 캔버스 + map + ego가 표시.
3. mouse wheel zoom 동작.
4. ego 위치 변경 시 카메라 (iso 모드) 자동 추종.

---

## 미해결 / 추정

1. **point sizeAttenuation 미구현**: three.js `PointsMaterial.sizeAttenuation=true`는 거리 기반 자동 축소. GL_POINTS는 vertex shader에서 매번 거리 계산해 `gl_PointSize` 결정 가능하지만 M3 시점에는 단순 매핑(`size_m * 8 px/m`) 채택. 카메라 zoom/높이가 크게 변할 때 점 크기가 살짝 안 맞을 수 있음. M4 ControlPanel이 `setPointSize(float)` slot을 호출해 사용자 조정 허용.
2. **GLTF Ego 모델 미구현 (M3.B 보류)**: 설계 §4.C/§6.C 그대로 — 박스 wireframe + heading arrow만. Assimp/Qt3D 도입은 M4 이후 사용자 의사 따라 결정.
3. **GL_LINE_SMOOTH best-effort**: Mesa Intel 드라이버 일부에서 `GL_LINE_SMOOTH` 무시. 4× MSAA framebuffer가 대신해서 선 AA를 처리하지만 시각적 품질이 GPU에 따라 미세 차이.
4. **HD map "build once" 동작**: `/hmi/threejs/map`은 latched 1회 발행 (web_hmi bridge). qt_hmi_node 시작 시 이미 latched 메시지를 받아 `onMapReceived` 호출 → `mapDirty_=true` (initializeGL 미완 단계) 또는 즉시 build. initializeGL 종료 후 deferred build를 처리. 정상 동작.
5. **Qt6 경로 미검증**: 사용자 환경 Qt 5.12.8. `find_package(Qt6 ... OpenGLWidgets)` 분기는 syntax-correct이나 라이브 빌드 미수행.
6. **트랙 박스/cloud 슬롯 GL teardown — makeCurrent 보호**: 현재 `~MapScene`에서 `makeCurrent()/doneCurrent()`로 보호. 단, 다른 destroyXxx 호출이 makeCurrent 없는 경로 (예: ControlPanel slot에서 setCameraMode 변경 시 destroyTrackSlots 호출하지 않음)에서 호출되지 않도록 주의 — 현재 destroyTrackSlots는 dtor에서만 호출.
7. **track points (REP-103 → 우리 좌표) 변환**: lidar 입력 좌표가 ego frame 가정. 만약 bridge가 절대 EPSG:5179 좌표로 보낸다면 `ez = -delta_north` 변환이 필요 — 검증 시 web_hmi와 시각 일치 여부 확인 필요. 현재 코드는 ego-frame 가정 (web_hmi TrackPointClouds.jsx와 동일).

---

## 부록 — M4 진입 체크리스트

(impl-verifier M3 검증 통과 후 다음 마일스톤 진입 준비)

- [ ] `include/qt_hmi/widgets/ControlPanel.h` + `.cpp` 신규 — `QDockWidget`, layer 13 체크박스 + camera mode radio + point size slider + bbox/heading/clouds 토글.
- [ ] `include/qt_hmi/widgets/BagStrip.h` + `.cpp` 신규 — 우상단 floating overlay click → `RosBridge::publishBagToggle()`.
- [ ] `MainWindow.cpp` ControlPanel signal → MapScene slot 연결 (이미 `setLayerVisibility/setShowBoxes/setShowHeading/setShowClouds/setPointSize/setCameraMode` 6 slot 노출됨).
- [ ] DRIVE MODE click handle → `RosBridge::publishModeRequest(bool)` (F1Dashboard 내부 inner widget).
- [ ] V2X TrafficLight: 이미 M2에서 traffic state 동작. M4 회귀 테스트만.
- [ ] `CMakeLists.txt`의 M4 슬롯에 ControlPanel/BagStrip 소스 추가.
