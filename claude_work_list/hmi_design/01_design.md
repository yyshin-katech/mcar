# qt_hmi 설계 문서

> 신규 패키지 `src/visualization/qt_hmi/` 의 Qt6/C++ 네이티브 HMI.
> **목표**: 기존 `web_hmi` (variant=`threejs_f1`) 와 동등한 기능을 단일 Qt 데스크톱 프로세스로 재현.
> **작성**: design-architect (분석/설계만, 코드 변경 없음).
> **참조 코드**: `src/visualization/web_hmi/` (기능 사양), `src/visualization/pyqt_hmi/scripts/utils/hmi_state.py` (ROS 콜백 패턴).

---

## 0. 결정 요약 (TL;DR)

| 항목 | 결정 | 근거 |
|------|------|------|
| 기술 스택 | Qt 6 Widgets + QOpenGLWidget (M3) | 사용자 결정. iGPU에서도 안정적이며 Three.js와 사고방식이 가장 가깝다. |
| C++ 표준 | C++17 | Qt6/ROS Noetic 권장. (`std::variant`, `if constexpr`, `std::filesystem` 활용) |
| 기능 범위 | web_hmi `threejs_f1` variant 와 동등 | F1 dashboard + 3D scene + ControlPanel + V2X 모두 포함. |
| 마일스톤 | M1 → M2 → M3 → M4 | 각 M = catkin_make PASS + 단일 기능 demo. |
| ROS↔Qt | `ros::AsyncSpinner(2)` + `Qt::QueuedConnection` | 콜백 스레드 → Qt main 스레드 자동 격리. |
| Map 데이터 소스 | **`/hmi/threejs/map` 토픽 재사용** (M3 기본) | web_hmi_threejs_bridge.py 가 이미 EPSG:32652→5179 변환 수행. 직접 .shp 로드는 후속 옵션 (§4.B). |
| Map state | `/hmi/state` (`speed/gear/mode/ego/gps/...`) JSON 도 그대로 재사용 | 동일 bridge에 의존 → 데이터 호환성 보장. |
| 좌표계 | 내부 + 표시 모두 EPSG:5179 (Korean TM) — bridge 가 origin shift 까지 수행 | three.js 코드와 동일한 origin-shifted delta 좌표 사용. |
| 빌드 의존 | catkin (`roscpp`, custom msgs) + `Qt6` (Core/Gui/Widgets/OpenGLWidgets) + `proj` (선택, 직접 .shp 로드 시) | proj 6.3.1 시스템 설치 확인. |

### 핵심 결정: web_hmi bridge 노드를 그대로 재사용한다

`web_hmi_bridge.py` + `web_hmi_threejs_bridge.py` 가 발행하는 **9개 `/hmi/*` JSON 토픽**을 qt_hmi 가 그대로 구독한다. 이로써:

- 메시지 정의(`mmc_msgs`, `katech_*`, `v2x_msgs`, `perception_ros_msg`) 의존을 qt_hmi 패키지 단에서 제거 가능 (`std_msgs::String` 만 필요).
- EPSG 좌표 변환·shapefile 파싱·진단 debounce 가 web_hmi 측 코드 한 곳에서 통일되게 유지됨.
- 향후 web/qt 양쪽이 동일한 데이터 모델을 공유 → 한 곳을 고치면 두 HMI 가 동시 갱신.

**대안**(직접 ROS 메시지 구독 + .shp 직접 파싱)은 §4.B 미해결 항목으로 유지. 시작은 JSON 기반.

---

## 1. 기능 매트릭스 (web_hmi 역공학)

### 1.A. ROS 토픽 → qt_hmi RosBridge signal 1:1 매핑

`/hmi/*` 모두 `std_msgs/String` (JSON payload). RosBridge 가 JSON 파싱 후 구조화된 Qt signal 로 emit.

| # | 토픽 | 발행 빈도 | qt_hmi RosBridge signal | 페이로드 키 | 사용 위젯 (M) |
|---|------|----------|-------------------------|------------|--------------|
| 1 | `/hmi/state` | 10 Hz | `stateChanged(const HmiState&)` | `speed,gear,mode,aeb,steering, ego{east,north,yaw}, gps{rtk,lon_std,lat_std}, speed_limit,link_id,lane_label, on_odd,road_state,selected_mode` | F1Dashboard (M2), MapScene (M3) |
| 2 | `/hmi/diagnostics` | 10 Hz | `diagChanged(const QHash<QString,int>&)` | `status{gps,adcu,lidar,radar,v2x,hmi,vcu,cam,ipc}` (0=OK,1=WARN,2=ERR) | F1Dashboard.HealthRow (M2) |
| 3 | `/hmi/objects` | up to 10 Hz | `objectsChanged(const QVector<HmiObject>&)` | `count, data:[{id,type,x,y,vx,vy,width,length,orientation,...}]` | F1 SVG fallback (M2 옵션), MapScene 비사용 (3D는 /hmi/threejs/tracks) |
| 4 | `/hmi/popup` | latched | `popupChanged(QString text, QString severity)` | `{text, severity}` | F1Dashboard.OddBanner (M2) |
| 5 | `/hmi/traffic` | latched | `trafficChanged(int color, int timeDecisec, int interId, int sgId)` | `{color,time_decisec,look_at{intersection_id,signal_group_id}}` | F1Dashboard.TrafficLight (M4) |
| 6 | `/hmi/bag` | latched | `bagChanged(bool recording, QString info)` | `{recording, info}` | F1Dashboard.BagButton (M4) |
| 7 | `/hmi/topic_hz` | 1 Hz | `hzChanged(const QHash<QString,double>&)` | `{gps:Hz, adcu:Hz, ...}` | F1Dashboard.HealthRow Hz suffix (M2) |
| 8 | `/hmi/map` | latched | `mapPolylinesReceived(const QVector<Polyline>&)` | `{polylines:[[[e,n],...],...]}` (EPSG:5179 절대 좌표) | F1 Environment SVG (M2 fallback only) |
| 9 | `/hmi/threejs/map` | latched | `threejsMapReceived(const HmiMap3D&)` | `{epsg:5179, origin:[e0,n0], layers:{<name>:{kind:point|polyline|polygon, data}}}` | MapScene (M3) |
| 10 | `/hmi/threejs/tracks` | 10 Hz | `tracksChanged(const QVector<HmiTrack3D>&)` | `{stamp, tracks:[{id,type,x,y,vx,vy,size_x,size_y,orientation,confidence, points?:[x,y,z,...]}]}` | MapScene.TrackBoxes + MapScene.TrackPointClouds (M3) |

**outbound (ROS publish)**: web_hmi와 동일하게 `/hmi/cmd/mode_request` (`std_msgs/Bool`), `/hmi/cmd/bag_toggle` (`std_msgs/Empty`).

| Outbound | 트리거 위젯 | RosBridge slot |
|---|---|---|
| `/hmi/cmd/mode_request` | ControlPanel.ModeButton (M4) — toggle Manual/Autonomous | `void publishModeRequest(bool autonomous)` |
| `/hmi/cmd/bag_toggle` | F1Dashboard.BagButton (M4) | `void publishBagToggle()` |

### 1.B. F1 dashboard 위젯 인벤토리 (M2 대상)

`F1HMI.jsx` 의 `F1HMIShell` 그리드 (1600×900):

| Section | 위젯 | 데이터 출처 |
|---------|------|------------|
| TOP BAR | KATECH 로고, UTC/KST 시계, TICK (uptime), NET (msg age), ROS 상태등 | local clock + RosBridge.connectionStatus() |
| 01 VELOCITY · STEER | SpeedHalf (반원 게이지, 한계선 marker) | `state.speed`, `state.speed_limit` |
| | SteerDial (조향 다이얼) + GEAR letter | `state.steering`, `state.gear` |
| | THR / BRK / ACCEL (placeholder) | "—" (PR-F1) |
| 02 DRIVE MODE | MANUAL / AUTONOMOUS 토글 시각, ENGAGED/STANDBY, v_max, ODD 라벨 | `state.mode`, `state.speed_limit`, `state.on_odd` |
| 03 V2X | TrafficLight (3 LED + PHASE + CHANGE IN sec) | `traffic.color`, `traffic.time_decisec` |
| 04 LOCALIZATION | LANE/LINK/σ-EAST/σ-NORTH/σ-UP/HDOP/SATS/HEADING + RTK 라벨 | `state.gps.*`, `state.link_id`, `state.lane_label`, `state.ego.yaw` |
| 05 SYSTEM HEALTH | 6 row health (GPS-RTK, K-ADCU/VCU, LIDAR, RADAR, CAMERA, V2X) + 4-dot summary | `diagnostics.status[*]`, `topic_hz[*]` |
| 06 ENVIRONMENT (main area) | Environment SVG (top-down rings + ego + objects + map polylines) | M2 fallback. M3 부터 QOpenGLWidget 으로 교체. |
| ODD banner (overlay) | 경고 텍스트 strip | `popup.severity != "info"` 일 때만 표시 |
| BAG strip (overlay) | REC/IDLE 버튼 + 파일명 | `bag.recording`, `bag.info`, click → `publishBagToggle()` |
| BOTTOM TELEMETRY | 8-cell grid (EGO-VEL, Δ-LIM, TRACKS, …) | 일부 live (`speed`, `speed_limit-speed`, `objects.count`), 나머지 PR-F1 placeholder |

### 1.C. MapScene (3D) 위젯 인벤토리 (M3 대상)

`threejs/*.jsx` 5개 파일이 빌드하는 scene:

| 컴포넌트 | qt_hmi 등가 클래스 | 책임 |
|---------|-------------------|------|
| `ThreeScene.jsx` (THREE.Scene + light + grid + groups: map/tracks/ego) | `MapScene : QOpenGLWidget` (`paintGL`, `initializeGL`, `resizeGL`) | OpenGL 컨텍스트, scene root group, render loop (60Hz QTimer 또는 update()) |
| `MapLayers.jsx` (HD map 11+2 layer LineSegments/LineLoop/Points) | `MapScene::buildHDMap()` + `LayerStyle.h` const table | `/hmi/threejs/map` 도착 시 한 번 빌드, layerVisibility 토글 적용 |
| `EgoMesh.jsx` (Ioniq5 GLTF + fallback wireframe + heading arrow) | `MapScene::EgoModel` | M3.A: 박스 wireframe + arrow. M3.B 후속: GLTF 로딩 (Assimp 또는 Qt3D 활용 — §4.C). |
| `TrackBoxes.jsx` (id별 wireframe box, color by type, heading arrow) | `MapScene::TrackBoxes` (`std::unordered_map<int, TrackSlot>`) | `/hmi/threejs/tracks` 매 프레임 add/update/remove |
| `TrackPointClouds.jsx` (track별 dynamic VBO, capacity grow by pow2) | `MapScene::TrackClouds` | `points` 배열 → VBO 업데이트 (`glBufferSubData`) |
| `CameraController.jsx` (iso vs top, wheel zoom) | `MapScene::Camera` (struct) | mode + zoom 상태 → view matrix 매 프레임 갱신 |
| `ControlPanel.jsx` (layer toggle, camera mode, point cloud size, bbox/heading toggle) | `ControlPanel : QWidget` (M4) | 사이드 도크. signal → MapScene slot. |

### 1.D. ROS msg type (재사용 시 직접 구독 미사용 — 참조용)

| 메시지 | 토픽 | 핵심 필드 (HMI 사용분) | 패키지 |
|--------|------|----------------------|--------|
| `mmc_msgs/chassis_msg` | `/sensors/chassis` | `vcu_VS`(km/h), `vcu_SAS_Angle`, `Curr_gear`, `vcu_ADMDStatus`, `AEB_flag` | mmc_msgs |
| `katech_custom_msgs/ioniq5_ad_can_msg` | `/sensors/ioniq5_ad_can` | `autonomous_mode` | katech_custom_msgs |
| `katech_custom_msgs/v_can_msg` | `/sensors/v_can` | `steering_angle`, `wheel_speed_*`, `gear_status` (legacy) | katech_custom_msgs |
| `mmc_msgs/to_control_team_from_local_msg` | `/localization/to_control_team` | `host_east/north/yaw`, `LINK_ID`, `lane_*`, `Speed_Limit`, `On_ODD`, `Road_State`, `look_at_IntersectionID`, `look_at_signalGroupID` | mmc_msgs |
| `v2x_msgs/intersection_array_msg` | `/siheung_spat` | `data[].IntersectionID`, `Movements.SignalGroupID`, `MovementPhaseStatus` (3=red, 6=green, 8=amber), `TimeChangeDetails` | v2x_msgs |
| `perception_ros_msg/object_array_msg` | `/track_Multi_RS` | `data[].id, x, y, vx, vy, orientation, size_x/y, valid_level, status` | perception_ros_msg |
| `perception_ros_msg/RsPerceptionMsg` | `/percept_topic` | `lidarframe.objects.objects[].coreinfo.*, supplementinfo.cloud_indices` | perception_ros_msg |
| `sensor_msgs/PointCloud2` | `/fusion_lidar_points` | xyz at offset 0..12, point_step | sensor_msgs |
| `katech_diagnostic_msgs/*` (9 종) | `/diagnostic/{cpt7_gps, adcu, lidar, radar, v2x, hmi, vcu, cam, ipc}` | StatCode, AliveCount | katech_diagnostic_msgs |

> qt_hmi 본 설계에서는 **위 메시지를 직접 구독하지 않는다**. web_hmi bridge 가 모두 JSON 으로 변환해 발행하며 qt_hmi 는 그것만 받는다. 따라서 qt_hmi `package.xml` 의존은 `roscpp` + `std_msgs` 로 충분.

---

## 2. ROS ↔ Qt 통합

### 2.A. RosBridge 클래스 (스레드 모델)

```
   +-----------------------+         signals (Qt::QueuedConnection)
   |  ros::AsyncSpinner    |   ----->  ----------------------------->  +------------------+
   |  (2 threads)          |         (auto cross-thread marshalling)   |  Qt main thread  |
   |  ros::Subscriber CB   |         emit stateChanged(parsedState);   |  widget slots    |
   +-----------------------+                                            +------------------+
       ^   |                                                                  |
       |   v                                                                  v
   ros::Publisher (mode_req, bag_toggle)   <----  publishXXX() (main thread invoked)
```

- `ros::AsyncSpinner(2)` 로 콜백을 별도 스레드에서 처리.
- `RosBridge` 는 `QObject`. 모든 signal 은 `Qt::QueuedConnection` 으로 위젯 slot 에 자동 marshal.
- 생성자에서 `nodeHandle_.subscribe(...)` 로 9 토픽 등록. 콜백 안에서:
  1. `msg.data` (JSON 문자열) 를 `nlohmann::json::parse()` (또는 Qt의 `QJsonDocument::fromJson`) 로 파싱.
  2. 구조체 (struct HmiState 등) 채움.
  3. `emit signalName(data)`.
- Qt 가 metatype 등록을 요구하므로 `qRegisterMetaType<HmiState>("HmiState");` 등을 `RosBridge` 생성자에서 1회 호출.

### 2.B. Signal 시그너처 (확정)

`include/qt_hmi/RosBridge.h`:

```cpp
#pragma once
#include <QObject>
#include <QHash>
#include <QString>
#include <QVector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "qt_hmi/HmiTypes.h"

class RosBridge : public QObject {
  Q_OBJECT
 public:
  explicit RosBridge(QObject* parent = nullptr);
  ~RosBridge() override;

  // Outbound — slot semantics, callable from main thread.
 public slots:
  void publishModeRequest(bool autonomous);
  void publishBagToggle();

 signals:
  // /hmi/state — 10 Hz snapshot
  void stateChanged(const HmiState& s);
  // /hmi/diagnostics — 10 Hz status hash
  void diagChanged(const QHash<QString,int>& status);
  // /hmi/topic_hz — 1 Hz Hz table
  void hzChanged(const QHash<QString,double>& hz);
  // /hmi/objects — up to 10 Hz
  void objectsChanged(const QVector<HmiObject>& objs);
  // /hmi/popup — latched, change-only
  void popupChanged(const QString& text, const QString& severity);
  // /hmi/traffic — latched, change-only
  void trafficChanged(int color, int timeDecisec,
                      int intersectionId, int signalGroupId);
  // /hmi/bag — latched, change-only
  void bagChanged(bool recording, const QString& info);
  // /hmi/map — latched (one-shot per launch)
  void mapPolylinesReceived(const QVector<Polyline>& polylines);
  // /hmi/threejs/map — latched
  void threejsMapReceived(const HmiMap3D& map);
  // /hmi/threejs/tracks — 10 Hz
  void tracksChanged(const QVector<HmiTrack3D>& tracks);
  // Connection status (computed locally from spinner liveness)
  void rosConnectionChanged(bool connected, qint64 lastMsgAgeMs);

 private:
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner_;          // 2 threads
  std::vector<ros::Subscriber> subs_;
  ros::Publisher pubModeReq_;
  ros::Publisher pubBagToggle_;

  // Per-topic JSON parse + emit
  void onState(const std_msgs::String::ConstPtr& msg);
  void onDiag(const std_msgs::String::ConstPtr& msg);
  void onHz(const std_msgs::String::ConstPtr& msg);
  void onObjects(const std_msgs::String::ConstPtr& msg);
  void onPopup(const std_msgs::String::ConstPtr& msg);
  void onTraffic(const std_msgs::String::ConstPtr& msg);
  void onBag(const std_msgs::String::ConstPtr& msg);
  void onMap(const std_msgs::String::ConstPtr& msg);
  void onThreejsMap(const std_msgs::String::ConstPtr& msg);
  void onTracks(const std_msgs::String::ConstPtr& msg);
};
```

### 2.C. Data types (`HmiTypes.h`)

```cpp
#pragma once
#include <QMetaType>
#include <QString>
#include <QVector>
#include <QHash>

struct HmiEgo  { double east=0, north=0, yaw=0; };
struct HmiGps  { int rtk=0; double lonStd=0, latStd=0; };
struct HmiState {
  double speed=0, steering=0;
  int gear=0, mode=0;
  bool aeb=false;
  HmiEgo ego;
  HmiGps gps;
  int speedLimit=0, linkId=0, onOdd=0, roadState=0, selectedMode=0;
  QString laneLabel;
};
struct HmiObject {
  int id=0;
  QString type;            // "car", "pedestrian", "truck", ...
  double x=0, y=0;          // ego frame meters
  double vx=0, vy=0;
  double width=0, length=0, orientation=0;
};
struct Polyline { QVector<QPointF> pts; };  // EPSG:5179 absolute
struct HmiLayer3D {
  QString kind;            // "point" | "polyline" | "polygon"
  // For polyline/polygon: QVector<QVector<QPointF>>
  // For point: QVector<QPointF>
  QVector<QVector<QPointF>> features;  // outermost = feature, inner = vertices
};
struct HmiMap3D {
  int epsg = 5179;
  QPointF origin;          // {east0, north0}
  QHash<QString, HmiLayer3D> layers;
};
struct HmiTrack3D {
  int id=0;
  QString type;
  double x=0, y=0;
  double vx=0, vy=0;
  double sizeX=0, sizeY=0;
  double orientation=0;
  double confidence=0;
  QVector<float> points;   // flat [x0,y0,z0, x1,y1,z1, ...]
};

Q_DECLARE_METATYPE(HmiState)
Q_DECLARE_METATYPE(HmiObject)
Q_DECLARE_METATYPE(Polyline)
Q_DECLARE_METATYPE(HmiMap3D)
Q_DECLARE_METATYPE(HmiTrack3D)
```

### 2.D. JSON 파싱 라이브러리 선택

- **선택**: Qt 내장 `QJsonDocument` / `QJsonObject` / `QJsonArray`.
- 근거: 외부 의존 없음. 성능은 nlohmann/json 보다 약간 떨어지나 10Hz 텍스트 페이로드 (대부분 < 200KB) 에서 무시 가능. 본 빌드에 nlohmann 미설치.
- `/hmi/threejs/map` 은 launch 1회 latched (수 MB 가능). 첫 파싱은 별도 worker QThread 에서 수행 후 `QtConcurrent::run` → `QFutureWatcher` 로 main 으로 결과 전달 (M3 시점 미세 최적화 — 일단 동기로).

---

## 3. 패키지 구조 + CMakeLists 골격

### 3.A. 디렉토리 트리

```
src/visualization/qt_hmi/
├── CMakeLists.txt
├── package.xml
├── README.md                       # 옵션 (M1 후속)
├── include/qt_hmi/
│   ├── HmiTypes.h                  # M1
│   ├── RosBridge.h                 # M1
│   ├── MainWindow.h                # M1
│   ├── widgets/
│   │   ├── F1Dashboard.h           # M2
│   │   ├── F1Sections.h            # M2 (SpeedHalf, SteerDial, TrafficLight, …)
│   │   ├── MapScene.h              # M3 (QOpenGLWidget)
│   │   ├── MapScene_glsl.h         # M3 (embedded shader strings)
│   │   ├── ControlPanel.h          # M4
│   │   └── BagStrip.h              # M4 (REC overlay)
│   └── style/
│       ├── F1Tokens.h              # 색·폰트 상수 (T.bg0, T.cyan, ...)
│       └── LayerStyle.h            # HD map layer color/size table
├── src/
│   ├── main.cpp                    # M1
│   ├── RosBridge.cpp               # M1
│   ├── MainWindow.cpp              # M1
│   ├── widgets/
│   │   ├── F1Dashboard.cpp         # M2
│   │   ├── F1Sections.cpp          # M2
│   │   ├── MapScene.cpp            # M3
│   │   ├── ControlPanel.cpp        # M4
│   │   └── BagStrip.cpp            # M4
│   └── style/
│       └── LayerStyle.cpp          # const table
├── shaders/                        # M3
│   ├── line.vert
│   ├── line.frag
│   ├── point.vert
│   └── point.frag
├── resources/
│   ├── qt_hmi.qrc                  # M2 (fonts, ego mesh, shaders)
│   └── fonts/
│       └── JetBrainsMono-Regular.ttf  # M2 — license 확인 후 추가 (§6)
└── launch/
    └── qt_hmi.launch               # M1 (web_hmi.launch 와 함께 실행 권장)
```

### 3.B. `package.xml`

```xml
<?xml version="1.0"?>
<package format="2">
  <name>qt_hmi</name>
  <version>0.0.1</version>
  <description>Qt6/C++ native HMI for IONIQ 5 (web_hmi 동등 기능).</description>
  <maintainer email="yyshin@katech.re.kr">Y. Shin</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <depend>roscpp</depend>
  <depend>std_msgs</depend>

  <!-- 데이터 소스: web_hmi bridge (런타임 의존) -->
  <exec_depend>web_hmi</exec_depend>

  <!-- Qt6 시스템 패키지 (cmake find_package) -->
  <!-- buildtool에는 cmake 만 들어감. Qt6 자체는 시스템 install 가정 (§6 참고) -->
</package>
```

### 3.C. `CMakeLists.txt` (핵심 발췌, M1 시점에 그대로 사용 가능)

```cmake
cmake_minimum_required(VERSION 3.16)
project(qt_hmi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

# Qt6 (M3에서 OpenGLWidgets 추가). M1/M2는 Core/Gui/Widgets만 필요.
find_package(Qt6 6.2 COMPONENTS Core Gui Widgets OpenGLWidgets)
if (NOT Qt6_FOUND)
  message(WARNING "Qt6 not found — qt_hmi will not be built. "
                  "See claude_work_list/hmi_design/01_design.md §6 for install.")
  catkin_package()
  return()
endif()

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

set(QT_HMI_SOURCES
  src/main.cpp
  src/RosBridge.cpp
  src/MainWindow.cpp
  src/widgets/F1Dashboard.cpp        # M2
  src/widgets/F1Sections.cpp         # M2
  src/widgets/MapScene.cpp           # M3
  src/widgets/ControlPanel.cpp       # M4
  src/widgets/BagStrip.cpp           # M4
  src/style/LayerStyle.cpp
)

set(QT_HMI_HEADERS
  include/qt_hmi/RosBridge.h
  include/qt_hmi/MainWindow.h
  include/qt_hmi/widgets/F1Dashboard.h
  include/qt_hmi/widgets/F1Sections.h
  include/qt_hmi/widgets/MapScene.h
  include/qt_hmi/widgets/ControlPanel.h
  include/qt_hmi/widgets/BagStrip.h
)

set(QT_HMI_RESOURCES resources/qt_hmi.qrc)

add_executable(qt_hmi_node
  ${QT_HMI_SOURCES} ${QT_HMI_HEADERS} ${QT_HMI_RESOURCES})

target_link_libraries(qt_hmi_node
  Qt6::Core Qt6::Gui Qt6::Widgets Qt6::OpenGLWidgets
  ${catkin_LIBRARIES}
)

install(TARGETS qt_hmi_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

> Qt6 부재 시 `return()` 으로 graceful skip — 다른 패키지 빌드를 막지 않는다.

### 3.D. `launch/qt_hmi.launch`

```xml
<?xml version="1.0"?>
<!--
  qt_hmi launcher.
  ROS data 는 web_hmi bridge 가 발행하는 /hmi/* 토픽을 그대로 구독.
  따라서 보통은 web_hmi.launch (variant=threejs_f1) 와 함께 실행.
-->
<launch>
  <arg name="rosbridge"  default="false"/>  <!-- web_hmi.launch 가 이미 띄움 -->
  <arg name="bridge"     default="true"/>   <!-- web_hmi.launch 단독 실행 안 했을 때 켬 -->

  <include if="$(arg bridge)" file="$(find web_hmi)/launch/web_hmi.launch">
    <arg name="variant"      value="threejs_f1"/>
    <arg name="open_browser" value="false"/>
  </include>

  <node name="qt_hmi_node"
        pkg="qt_hmi" type="qt_hmi_node" output="screen" required="true"/>
</launch>
```

---

## 4. 3D 렌더 결정 (M3)

### 4.A. 렌더러: `QOpenGLWidget` + 직접 GL (확정)

비교:

| 후보 | 장점 | 단점 |
|------|------|------|
| **QOpenGLWidget + 직접 GL** ★ | iGPU 친화 (지오메트리 단순화 직접 통제), Three.js 사고방식 동일, Qt 통합 자연스러움 | shader 직접 작성, 카메라 수학 직접 |
| Qt3D | scene graph 고수준 API | iGPU 미묘 (성능 케이스), 의존성 큼, Qt6 지원 스펙 변경 잦음 |
| OSG / VTK | 성숙한 3D 엔진 | 무거움, 의존성 추가, ROS와 양립 어려움 |

**채택**: `QOpenGLWidget` + 직접 GL. OpenGL Core profile 3.3 (확정 — Mesa/iGPU 호환 최댓값 안정점). `QOpenGLFunctions_3_3_Core` 사용.

### 4.B. Map 데이터 소스: `/hmi/threejs/map` (확정 — M3 기본)

- bridge 가 EPSG:32652→5179 변환 + `origin` shift 까지 수행 → qt_hmi 는 즉시 GL 좌표로 사용 가능.
- 단점: bridge 노드가 죽으면 map 도 사라짐. → M3 데모 안정성에는 충분. 후속 안 (직접 .shp 로드, libshp 또는 GDAL/OGR) 은 §6 미해결로 유지.

`HmiLayer3D::features`:
- `kind == "polyline"`: `features[i]` = i 번째 polyline 의 정점 리스트 (delta_east, delta_north).
- `kind == "polygon"`: `features[i]` = outer ring (delta).
- `kind == "point"`: `features` 의 첫 행 `features[0]` 만 사용, 안에 모든 point 가 들어감 (또는 `features[i]` 가 1점 — 구현은 두 방식 다 허용; bridge JSON 은 `data: [[e,n],[e,n],...]` 평면 리스트).

> **bridge JSON shape 정확성**: `web_hmi_threejs_bridge.py:_load_polylines` 는 polyline `data: [[ [e,n],... ], [ [e,n],... ], ...]`, point `data: [[e,n], [e,n], ...]`, polygon `data: [[ [e,n],... ], ...]` 로 발행. qt_hmi 파서는 `kind` 별 분기 필요.

### 4.C. Ego 모델

M3.A: 박스 wireframe (4.635 × 1.890 × 1.605 m) + heading arrow (Three.js fallback과 동일).
M3.B (옵션): GLTF 로딩. 후보:
- Assimp (libassimp-dev) + 직접 mesh 빌드.
- Qt3D (의존만 추가하고 wrap 한 mesh 사용).
- 단순 OBJ wrapper.

→ M3 시점에 결정 보류. **M3 디폴트는 박스 wireframe** (web_hmi 도 GLTF 실패 시 동일 fallback 사용).

### 4.D. 좌표/카메라 수학

three.js 의 `scene.scale.z = -1` 트릭 대신 qt_hmi 는 모든 정점 입력 시 `z = -delta_north` 로 부호 반전. 카메라 mode:

```
delta_e = state.ego.east  - map.origin.x
delta_n = state.ego.north - map.origin.y
ex = delta_e
ez = -delta_n            // world Z (left-handed -> -north)

iso (chase):
  back   = 60 / zoom
  height = 60 / zoom
  cam.pos = (ex - back*cos(yaw), height, ez + back*sin(yaw))
  cam.up  = (0, 1, 0)
  cam.lookAt(ex, 0, ez)

top (bird's-eye):
  cam.up  = (0, 0, -1)    // -Z to up on screen (= real north)
  cam.pos = (ex, 100/zoom, ez)
  cam.lookAt(ex, 0, ez)
```

`QOpenGLWidget::mouseWheelEvent` 에서 zoom 누적, `QOpenGLWidget::keyPressEvent` 또는 ControlPanel radio 로 mode 전환.

### 4.E. 셰이더

총 4 개 (vert/frag × line/point) — embedded `static const char*` 또는 `:/shaders/...` Qt 리소스. 단순 모델·뷰·프로젝션 변환 + uniform color. AA 는 GL_LINES + GL_LINE_SMOOTH (지원되면) 또는 둔감하게 라인 폭 1.0 + MSAA on framebuffer.

---

## 5. 마일스톤

각 마일스톤은 **(빌드 PASS) ∧ (단일 기능 demo 가능)** 을 종료 조건으로 한다. 모든 시각적 해법은 web_hmi 의 동등 페이지(`http://localhost:8088/index_threejs_f1.html`) 와 비교 검증.

### M1 — 패키지 스켈레톤 + RosBridge

#### 신규 파일

| 경로 | 역할 |
|------|------|
| `src/visualization/qt_hmi/CMakeLists.txt` | §3.C 발췌. M2~M4 소스는 `# M2`·`# M3` 주석으로 비활성/추가. |
| `src/visualization/qt_hmi/package.xml` | §3.B |
| `src/visualization/qt_hmi/include/qt_hmi/HmiTypes.h` | §2.C 전부. `Q_DECLARE_METATYPE` 포함. |
| `src/visualization/qt_hmi/include/qt_hmi/RosBridge.h` | §2.B 전부. M1 시 9 signal 모두 선언 (위젯이 구독 안 해도 OK). |
| `src/visualization/qt_hmi/src/RosBridge.cpp` | 9 토픽 subscribe + JSON parse + emit. M1 demo 위해 9 signal 모두 console log (qDebug) 도 함께. |
| `src/visualization/qt_hmi/include/qt_hmi/MainWindow.h` | placeholder `QMainWindow`. central widget = `QLabel("qt_hmi M1: ROS bridge ready")`. |
| `src/visualization/qt_hmi/src/MainWindow.cpp` | RosBridge instance 보유, `stateChanged` signal 을 label text 로 연결 (`speed: %.1f km/h`). |
| `src/visualization/qt_hmi/src/main.cpp` | `QApplication` + `ros::init(argc, argv, "qt_hmi_node")` + `MainWindow::show()` + `app.exec()`. ros 종료는 `ros::shutdown()` in `QApplication::aboutToQuit`. |
| `src/visualization/qt_hmi/launch/qt_hmi.launch` | §3.D |

#### RosBridge.cpp 핵심 시그너처

```cpp
RosBridge::RosBridge(QObject* parent)
  : QObject(parent), spinner_(2)
{
  qRegisterMetaType<HmiState>("HmiState");
  qRegisterMetaType<HmiObject>("HmiObject");
  qRegisterMetaType<QVector<HmiObject>>("QVector<HmiObject>");
  qRegisterMetaType<QHash<QString,int>>("QHash<QString,int>");
  qRegisterMetaType<QHash<QString,double>>("QHash<QString,double>");
  qRegisterMetaType<HmiMap3D>("HmiMap3D");
  qRegisterMetaType<QVector<HmiTrack3D>>("QVector<HmiTrack3D>");
  qRegisterMetaType<QVector<Polyline>>("QVector<Polyline>");

  subs_.push_back(nh_.subscribe<std_msgs::String>(
    "/hmi/state", 2, &RosBridge::onState, this));
  // ... 9개 모두

  pubModeReq_   = nh_.advertise<std_msgs::Bool>("/hmi/cmd/mode_request", 1);
  pubBagToggle_ = nh_.advertise<std_msgs::Empty>("/hmi/cmd/bag_toggle", 1);

  spinner_.start();
}
```

#### M1 검증

1. 사전: `roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false` 실행 중.
2. `catkin_make --pkg qt_hmi` PASS (Qt6 미설치 시는 `qt_hmi` 가 자동 skip — 별도 PASS 조건).
3. `rosrun qt_hmi qt_hmi_node` 실행 → 빈 창 + label "qt_hmi M1: ROS bridge ready" 표시.
4. label 이 1초 내에 "speed: X km/h" 형태로 갱신됨 (state.speed).
5. `qDebug` 출력에 9 토픽 중 최소 5개 이상 (`state`, `diagnostics`, `topic_hz`, `objects`, `traffic`) 의 첫 메시지 도착 로그가 보임.
6. 종료 시 segfault 없이 깨끗한 `ros::shutdown()`.

---

### M2 — F1Dashboard 위젯 (텔레메트리 + 진단)

#### 신규 파일

| 경로 | 역할 |
|------|------|
| `include/qt_hmi/style/F1Tokens.h` | F1HMI.jsx의 `T` 색팔레트 상수화 (`constexpr QColor`). 토큰 전체 19 개. |
| `include/qt_hmi/widgets/F1Sections.h` + `.cpp` | `SpeedHalf`, `SteerDial`, `TrafficLight`, `Section`, `Stat`, `Dot`, `HealthRow` 7개 작은 위젯. 각각 `QWidget`, `paintEvent` 또는 SVG-equivalent QPainter 코드. |
| `include/qt_hmi/widgets/F1Dashboard.h` + `.cpp` | 1600×900 grid layout (`QGridLayout`) — top bar / left panel / main area placeholder / bottom strip. 입력은 `RosBridge` signal 만. |
| `resources/qt_hmi.qrc` | 폰트 파일 (있다면). 없으면 시스템 mono fallback (예: "DejaVu Sans Mono"). |
| `resources/fonts/JetBrainsMono-Regular.ttf` | 옵션 — 라이선스 확인 후 (`§6`). |

#### 위젯 책임 매핑 (F1HMIShell prop → Qt widget update)

| prop | F1Sections 위젯 / 슬롯 | RosBridge signal |
|------|------------------------|------------------|
| `speed`, `speedLimit` | `SpeedHalf::setValues(double v, double limit)` | `stateChanged(s)` → `setValues(s.speed, s.speedLimit)` |
| `steeringAngle`, `gearLetter` | `SteerDial::setAngle(double deg)`, `setGear(int)` | `stateChanged(s)` |
| `engaged`, `vMaxText`, `oddText` | `DriveModePanel::setEngaged(bool)` 등 | `stateChanged(s)` |
| `trafficPhase`, `trafficRemain` | `TrafficLight::setState(int color, int sec)` | `trafficChanged(c, t, ...)` |
| `health`, `summary` | `HealthList::setStatuses(const QHash<QString,int>&, const QHash<QString,double>&, RtkLabel)` | `diagChanged(...)` ∧ `hzChanged(...)` |
| `oddBanner` | `OddBanner::setText(QString)` (overlay on main area) | `popupChanged(text, sev)` (sev != "info" → text else 비워) |
| `bagRecording`, `bagInfo` | M4 (`BagStrip`) | `bagChanged(...)` |
| `objs` | (M2 fallback) `EnvironmentSvg` widget — `paintEvent` 에서 SVG 와 동일한 도형. M3에서 `MapScene` 으로 교체. | `objectsChanged(...)`, `mapPolylinesReceived(...)`, `stateChanged(s.ego)` |

#### 좌표 변환 정확성

- F1HMIScreen.jsx 의 `objectKindLabel`, `buildF1Objects` 로직을 그대로 C++ 함수로 이식 (`HmiObject` → `F1ObjectVisual` 빌더 in `F1Dashboard.cpp`).
- `pxPerMeter = 5.6`, `ringRX = {120,230,340,450,560}` 등 모두 그대로.

#### M2 검증

1. M1 demo 환경 + qt_hmi_node 실행 시 1600×900 창에 F1 chrome 이 모두 표시.
2. `state.speed` 가 변하면 SpeedHalf 게이지가 부드럽게 (≥10 Hz) 갱신.
3. `traffic.color = 1/2/3` 일 때 적/황/녹 LED 가 켜지며 `time_decisec/10` 초가 표시.
4. `diag.status.gps = 1` 시 GPS-RTK 행이 amber WARN.
5. `popup.severity = "warn"` 시 main area 상단 amber banner.
6. ROS 끊김 (rosbridge 종료) 시 TOP BAR ROS 배지가 적색 OFFLINE.
7. (옵션) `objs`, `mapPolylines` 가 main area SVG-equivalent 위젯에 표시 — M3에 의해 교체될 임시 구현이라 정확도는 ±픽셀 단위 차이 허용.

---

### M3 — MapScene (3D)

#### 신규 파일

| 경로 | 역할 |
|------|------|
| `include/qt_hmi/style/LayerStyle.h` + `src/style/LayerStyle.cpp` | `types.js` 의 `LAYER_STYLE` 13 키 (HD map 11 + TB_senario_* 2) 를 `static constexpr struct{const char* name; QColor color; float widthOrSize; float alpha; LayerKind kind;}` 배열로. `DEFAULT_LAYER_VIS` 도 이식. |
| `include/qt_hmi/widgets/MapScene.h` + `.cpp` | `class MapScene : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core`. 내부 멤버: `MapBuilder`, `EgoModel`, `TrackBoxes`, `TrackClouds`, `Camera`. |
| `shaders/line.vert`, `line.frag`, `point.vert`, `point.frag` | embedded via `qt_hmi.qrc`. |
| `resources/qt_hmi.qrc` | 셰이더 추가. |

#### MapScene 내부 구조 (한 클래스, 여러 helper 메서드)

```cpp
class MapScene : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core {
  Q_OBJECT
 public:
  explicit MapScene(QWidget* parent = nullptr);
 public slots:
  void onMapReceived(const HmiMap3D& m);
  void onTracksChanged(const QVector<HmiTrack3D>& tracks);
  void onStateChanged(const HmiState& s);              // ego pose for camera
  void setLayerVisibility(const QHash<QString,bool>& v);
  void setShowBoxes(bool on);
  void setShowHeading(bool on);
  void setShowClouds(bool on);
  void setPointSize(float sz);
  void setCameraMode(QString mode);                    // "iso" or "top"
 protected:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int w, int h) override;
  void wheelEvent(QWheelEvent* e) override;
 private:
  // GL resources
  GLuint shaderLine_, shaderPoint_;
  // Built once after onMapReceived
  struct LayerGL { GLuint vao=0, vbo=0; int vertexCount=0; LayerKind kind; QColor color; float wOrS=1.0f; float alpha=1.0f; bool visible=true; };
  QHash<QString, LayerGL> mapLayers_;
  // Per-track point cloud
  struct TrackSlot { GLuint vao=0, vbo=0; int capacity=0; int count=0; QColor color; QString type; double x=0,y=0,sx=0,sy=0,yaw=0; };
  QHash<int, TrackSlot> tracks_;
  // ego
  GLuint egoVao_=0, egoVbo_=0;       // wireframe box vertices
  // camera
  HmiState lastState_;
  QPointF mapOrigin_;
  QString camMode_ = "iso";
  float zoom_ = 1.0f;
  // track display options
  bool showBoxes_ = true, showHeading_ = true, showClouds_ = true;
  float pointSize_ = 0.08f;
};
```

#### 시그널 라우팅 (MainWindow 책임)

```
RosBridge::threejsMapReceived  -> MapScene::onMapReceived
RosBridge::tracksChanged       -> MapScene::onTracksChanged
RosBridge::stateChanged        -> MapScene::onStateChanged
ControlPanel (M4) signals      -> MapScene 프로퍼티 setter
```

#### M3 검증

1. (M2 환경 + bag replay) MapScene 가 main area 를 차지하며 HD map 11 layer 가 색상 구분되어 렌더.
2. ego 박스가 `state.ego.east/north/yaw` 에 따라 매끄럽게 (≥10 Hz) 이동/회전.
3. `tracks` 가 도착하면 박스/포인트 클라우드가 ego 기준 보정 위치에 표시 (Three.js 결과와 ±0.5 m 이내 일치).
4. mouse wheel zoom 동작.
5. (M4 ControlPanel 미적용) iso 카메라 고정. 코드 안 `camMode_ = "top"` 으로 빌드해도 정상 렌더.
6. 60 fps target. iGPU 에서 30 fps 미만이면 즉시 §6 의 "메쉬 단순화" 후속 검토 트리거.

---

### M4 — ControlPanel + V2X + Bag

> **참고**: V2X 자체는 M2에서 RosBridge::trafficChanged 가 이미 도착하지만, M2의 TrafficLight 위젯은 단방향 표시. M4에서는 **사용자 토글로 layer ON/OFF**, **camera 모드 변경**, **mode_request publish**, **bag_toggle publish** 가 추가된다.

#### 신규 파일

| 경로 | 역할 |
|------|------|
| `include/qt_hmi/widgets/ControlPanel.h` + `.cpp` | `QDockWidget` 우측. 위쪽: Display(checkbox bbox/heading/clouds, slider point size), Camera(radio iso/top), Map layers(13 checkbox + swatch). 모든 변경 → MapScene slot. |
| `include/qt_hmi/widgets/BagStrip.h` + `.cpp` | 우상단 floating overlay. RecState UI, click → `RosBridge::publishBagToggle()`. |
| `include/qt_hmi/widgets/ModeButton.h` (옵션, F1Dashboard.cpp 내부 inner class 도 가능) | DRIVE MODE 토글에 클릭 핸들 추가. click → `RosBridge::publishModeRequest(autonomous)`. |

#### ControlPanel ↔ MapScene 시그널

```
ControlPanel::layerToggled(QString name, bool on) -> MapScene::setLayerVisibility(...)
ControlPanel::showBoxesChanged(bool)               -> MapScene::setShowBoxes
ControlPanel::showHeadingChanged(bool)             -> MapScene::setShowHeading
ControlPanel::showCloudsChanged(bool)              -> MapScene::setShowClouds
ControlPanel::pointSizeChanged(float)              -> MapScene::setPointSize
ControlPanel::cameraModeChanged(QString)           -> MapScene::setCameraMode
```

기본값은 `DEFAULT_LAYER_VIS` (Three.js와 동일).

#### M4 검증

1. ControlPanel 13 layer 체크박스 토글 시 MapScene 의 해당 layer GL drawElements 가 실제로 호출/스킵.
2. camera radio 변경 시 즉시 (1 frame 내) 카메라 mode 전환.
3. bag overlay click → `/hmi/cmd/bag_toggle` publish 후 1초 이내에 `/hmi/bag.recording` 가 변경되어 overlay 가 RED REC 로 갱신 (수동 검증: `web_hmi.launch` 가 띄운 bridge 가 실제 rosbag 시작).
4. DRIVE MODE 클릭 → `/hmi/cmd/mode_request` publish 후 1초 펄스 동안 `state.selected_mode == 1` (qDebug 로 확인).
5. V2X TrafficLight 가 `traffic.color = 0/1/2/3` 에 따라 OFF/GREEN/AMBER/RED 표시 (M2에서 이미 동작; M4 회귀 테스트).

---

## 6. 미해결 / 후속 결정

### 6.A. **(중요) Qt6 시스템 미설치**

- 검사 결과: `qmake` 는 `5.12.8`, `apt list --installed | grep qt6` 결과 0건. Ubuntu 20.04 기본 저장소에는 `qt6-base-dev` 패키지가 없음 (Ubuntu 22.04+ 또는 Qt 공식 installer 필요).
- 옵션:
  - **A. Qt 공식 installer (권장)**: `https://www.qt.io/download-qt-installer` 로 Qt 6.5+ LTS 를 `/opt/Qt/6.x.x/gcc_64` 에 설치. 빌드 시 `CMAKE_PREFIX_PATH=/opt/Qt/6.x.x/gcc_64`. 라이선스: open source (LGPL).
  - **B. KDE Neon / Kubuntu PPA**: `sudo add-apt-repository ppa:beineri/...` 류 — 안정성 검토 필요.
  - **C. Qt5 로 다운그레이드 (대안 plan)**: Ubuntu 20.04 에 이미 5.12.8 + libqt5opengl5-dev 모두 설치됨. `find_package(Qt5 5.12 COMPONENTS Core Gui Widgets OpenGL)` 로 변경 가능. 단, M3 에서 `Qt5::OpenGL` 의 `QOpenGLWidget` 만 사용하므로 호환됨. F1 디자인 토큰·signal/slot 코드는 Qt5/Qt6 모두 호환.
- **결정 보류**: M1 시작 직전에 사용자 결정. 기본 설계는 Qt6 가정 (위 §3.C). Qt5 로 fallback 시 `Qt6::OpenGLWidgets` → `Qt5::OpenGL`, `find_package(Qt5 ... )` 로만 바꾸면 동작.

### 6.B. M3 Map 데이터 소스 후속

- 현재 결정: `/hmi/threejs/map` 토픽 재사용. → bridge 노드 의존.
- 후속 옵션 (qt_hmi 단독 운영):
  - **GDAL/OGR 직접 로드** — `libgdal-dev 3.0.4` 설치 확인됨. `OGRRegisterAll()` + `GDALOpenEx(..., GDAL_OF_VECTOR, ...)` 로 `.shp` 읽고 `OGRSpatialReference::importFromEPSG(32652)` + `OGRCoordinateTransformation` 으로 EPSG:5179 변환.
  - **shapelib (libshp)** — Ubuntu 저장소에 1.5.0 존재 (미설치). 더 가벼움. proj 6.3.1 과 결합 필요.
- 결정 시점: M3 시작 시 또는 M4 후 선택.

### 6.C. GLTF Ego 모델 (M3.B)

- web_hmi 는 Sketchfab "Hyundai Ioniq 5 - Lowpoly" 사용. 라이선스 CC-BY-4.0.
- qt_hmi 옵션:
  - Assimp (`libassimp-dev 5.0.x`) + 직접 mesh + texture 빌드.
  - Qt3D (의존성 추가).
  - 단순 OBJ → 직접 파서.
- 결정 보류, M3.A 박스 wireframe 으로 M3 종료 가능.

### 6.D. JetBrains Mono 폰트

- web_hmi `vendor/` 에 포함되어 있다면 그대로 복사 사용. 라이선스: SIL OFL 1.1 (재배포 OK).
- 미포함 시 시스템 fallback 사용 또는 `apt install fonts-jetbrains-mono` (Ubuntu 22.04+; 20.04에서는 deb 직접 설치).
- 결정 보류, M2 시작 직전 확인.

### 6.E. F1 dashboard `objs` 표시 (M2)

- web_hmi `F1HMIScreen.jsx` 는 `/hmi/objects` 를 받아 SVG 환경에 그림. `ThreejsF1Screen.jsx` 는 `objs={[]}` 로 비워 두고 3D scene 으로 대체.
- qt_hmi 의 M2 main area 는 어떤 상태?
  - 옵션 A: M2 까지는 빈 placeholder (검정 배경 + "MapScene M3 will fill this") — M3가 즉시 채움.
  - 옵션 B: M2 에서 SVG-equivalent QPainter 로 `objs` 와 `polylines` 표시 (= web_hmi `Environment` 함수 이식). M3 에서 동일 영역을 OpenGL widget으로 교체.
- **결정**: 옵션 A (간결성 우선). `polylines/objects` 가 와도 라벨만 표기 (`"polylines: 1827, tracks: 12"`). 시각화는 M3 부터.

### 6.F. 빌드 시스템 — Qt6 자동 감지

- §3.C 의 `find_package(Qt6 ...)` 가 실패하면 `qt_hmi` 만 skip 하도록 `return()`. 다른 패키지 빌드는 영향 없음.
- 후속: catkin profile 분리 (qt-release vs ros-only) 고려. 현재 설계로 충분.

### 6.G. iGPU 성능 회귀

- ASUS NUC iGPU 제약 (`MEMORY.md project_target_hardware`):
  - 메쉬 단순화 (지오메트리 vertex < ~50k).
  - 그림자 없음, post-processing 없음, 라이트 1개 (ambient + 1 directional 동일).
  - draw call 최소 — layer당 단일 VAO/VBO, instancing 미사용.
- 후속 모니터링: M3 종료 시 `vsync off` 상태에서 60 fps 측정 → 미달 시 layer 결합/배치 조정.

### 6.H. 메모리 정리 / 종료 시 race

- `ros::AsyncSpinner` 가 콜백 진행 중에 `RosBridge` 가 destroy 되면 segfault 위험.
- 패턴: `MainWindow` 는 `RosBridge` 를 `unique_ptr` 로 보유 → `aboutToQuit` 에서 spinner stop → 그 다음 destroy.
- M1 시점에 동작 확인 필수.

---

## 7. 구현 순서 (impl-coder 가 즉시 따라갈 작업 큐)

```
M1.1   디렉토리 + package.xml + CMakeLists 골격 (§3)
M1.2   HmiTypes.h, RosBridge.h/.cpp (9 sub + 2 pub) (§2.B/C)
M1.3   MainWindow placeholder, main.cpp (§5/M1)
M1.4   launch/qt_hmi.launch (§3.D)
M1.5   catkin_make + manual demo (§5/M1 검증)
-----
M2.1   F1Tokens.h
M2.2   F1Sections.cpp (SpeedHalf, SteerDial, TrafficLight, Section, Stat, Dot, HealthRow)
M2.3   F1Dashboard.cpp (grid, signal wire-up)
M2.4   Top bar (clock, ROS badge), Bottom strip (8 cells)
M2.5   Build + manual demo (§5/M2 검증)
-----
M3.1   LayerStyle.h/.cpp (13 layer table + DEFAULT_LAYER_VIS)
M3.2   MapScene.h skeleton + initializeGL/paintGL/resizeGL
M3.3   shaders/* (line/point) + qt_hmi.qrc
M3.4   onMapReceived → MapBuilder → mapLayers_
M3.5   onStateChanged → camera + ego model
M3.6   onTracksChanged → boxes + clouds (capacity grow)
M3.7   wheelEvent zoom
M3.8   Build + manual demo (§5/M3 검증)
-----
M4.1   ControlPanel.h/.cpp (도크 + 위젯)
M4.2   ControlPanel signal ↔ MapScene slot 연결
M4.3   BagStrip.cpp + click → publishBagToggle
M4.4   F1Dashboard DriveMode click → publishModeRequest
M4.5   Build + manual demo (§5/M4 검증)
```

---

## 8. 변경 / 갱신 정책

- 본 문서는 impl-coder 가 시작하기 전 동결. 마일스톤 진행 중 중요한 발견(예: Qt5 fallback, GLTF 누락, GL 호환성 문제)은 아래 "변경 이력" 표에 추가하고 영향 받는 §섹션을 갱신.

| 날짜 | 변경자 | §섹션 | 변경 내용 |
|------|--------|------|----------|
| 2026-05-09 | design-architect | (전체) | 초안 작성 (web_hmi v3.2 / threejs_f1 variant 기준) |

