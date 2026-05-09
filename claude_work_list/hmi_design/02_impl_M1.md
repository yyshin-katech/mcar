# impl-coder M1 보고서

> 마일스톤: **M1 — 패키지 스켈레톤 + RosBridge + MainWindow placeholder**
> 작성: impl-coder
> 일시: 2026-05-09
> 빌드 환경: Qt 5.12.8 (Ubuntu 20.04, design §6.A.C fallback)

---

## 작성/수정 파일

신규 9개 (수정 0개).

| 파일 | 신규/수정 | 줄수 | 역할 |
|------|----------|-----:|------|
| `src/visualization/qt_hmi/package.xml` | 신규 | 19 | catkin manifest. `roscpp`/`std_msgs` build+exec, `web_hmi` exec_depend (런타임 bridge). |
| `src/visualization/qt_hmi/CMakeLists.txt` | 신규 | 80 | Qt6 → Qt5 자동 감지 (Qt6 미설치 시 Qt5 5.12 fallback). 둘 다 없으면 `catkin_package(); return()` 으로 graceful skip. M1 대상 3 src + 3 헤더만 빌드. M2/M3/M4 슬롯은 `# TODO` 주석. |
| `src/visualization/qt_hmi/include/qt_hmi/HmiTypes.h` | 신규 | 99 | `HmiState/HmiEgo/HmiGps/HmiObject/Polyline/HmiLayer3D/HmiMap3D/HmiTrack3D` POD 구조체. `Q_DECLARE_METATYPE` 8건. |
| `src/visualization/qt_hmi/include/qt_hmi/RosBridge.h` | 신규 | 77 | QObject. 9 signal + 2 publisher slot + 10 콜백. `ros::AsyncSpinner(2)` 보유. `stop()` 메서드 (race-safe shutdown). |
| `src/visualization/qt_hmi/include/qt_hmi/MainWindow.h` | 신규 | 45 | `QMainWindow` placeholder. 6 QLabel 멤버 + 6 슬롯 (`onState`/`onDiag`/`onTraffic`/`onPopup`/`onBag`/`onTracks`). |
| `src/visualization/qt_hmi/src/RosBridge.cpp` | 신규 | 358 | 9 토픽 구독 + JSON → struct 파싱 + emit. QJsonDocument 기반 (외부 의존 없음). 콜백 10개 (`onState/onDiag/onHz/onObjects/onPopup/onTraffic/onBag/onMap/onThreejsMap/onTracks`). |
| `src/visualization/qt_hmi/src/MainWindow.cpp` | 신규 | 129 | QVBoxLayout + 7 QLabel. RosBridge signal 6개 직접 연결 (`AutoConnection` → cross-thread `QueuedConnection`). M2/M3/M4 TODO 라벨 표시. |
| `src/visualization/qt_hmi/src/main.cpp` | 신규 | 50 | `ros::init` (NoSigintHandler) → `QApplication` → `unique_ptr<RosBridge>` → `MainWindow` → `aboutToQuit` 핸들러 → `app.exec()` → `bridge.reset()` → `ros::shutdown()`. |
| `src/visualization/qt_hmi/launch/qt_hmi.launch` | 신규 | 32 | `bridge:=true` 기본 (web_hmi.launch include, `variant=threejs_f1`, `open_browser=false`). `bridge:=false` 면 qt_hmi 노드만 단독 실행. |

**총 889줄** (코드 + manifest + launch).

---

## 핵심 결정

### 1. Qt6 → Qt5 자동 fallback (CMakeLists.txt)

설계 §6.A.C 의 Qt5 fallback 절차를 그대로 구현. `find_package(Qt6 ...)` 가 실패하면 `find_package(Qt5 5.12 ...)` 로 떨어지고, 둘 다 없으면 `catkin_package()` 만 호출하고 `return()` 으로 패키지를 graceful skip 한다 (다른 catkin 패키지 빌드를 막지 않음).

링크 변수는 `${QT_HMI_QT_LIBS}` 로 추상화 — Qt5/Qt6 라이브러리 이름 차이 (`Qt5::OpenGL` vs `Qt6::OpenGLWidgets`) 가 있을 M3 시점에 한 곳에서 분기 가능.

### 2. JSON 파싱: Qt 내장 `QJsonDocument`

설계 §2.D 결정 그대로. nlohmann/json 미설치, rapidjson 미설치 → 추가 의존을 만들지 않기 위해 `QJsonDocument::fromJson(QByteArray)` 사용. 10 Hz 페이로드 (대부분 ≤ 200 KB) 에서 충분한 성능. `/hmi/threejs/map` (latched, 수 MB 가능) 도 일단 동기 파싱 — 첫 프레임 한 번만 비용 발생하므로 M3 시점에 worker thread 고려.

JSON 파싱 헬퍼 (`jdouble/jint/jbool/jstr`) 를 anonymous namespace 에 넣어 안전한 default 값 보장 (필드 누락/타입 불일치 방어).

### 3. 스레드 모델 (설계 §2.A 그대로)

- `RosBridge` 자체는 GUI 메인 스레드에서 생성 (main.cpp 에서 `make_unique<RosBridge>()`).
- `ros::AsyncSpinner(2)` 가 백그라운드 2 스레드에서 콜백 실행.
- 콜백 안에서 JSON 파싱 후 `emit signalName(...)` 만 수행 → Qt 이 스레드 친화성 검사 후 자동으로 `Qt::QueuedConnection` 으로 marshalling (signal 송신자 = bridge 스레드, 수신자 = main 스레드).
- 모든 metatype 은 `RosBridge` 생성자에서 `qRegisterMetaType<T>("T")` 1회 호출 (8 종).

### 4. Race-safe 종료 (설계 §6.H)

`main.cpp` 에서:

1. `unique_ptr<RosBridge>` 보유 (스택 / 자동 정리 X — 명시적 reset 시점 제어).
2. `QApplication::aboutToQuit` 람다 → `bridge->stop()` (spinner stop, 콜백 진입 차단).
3. `app.exec()` 종료 후 `bridge.reset()` 으로 명시적 destruct.
4. 마지막 `ros::shutdown()`.

`RosBridge::stop()` 은 idempotent — 직접 호출되든 destructor 에서 호출되든 안전.

### 5. `ros::init_options::NoSigintHandler`

Qt 의 SIGINT 처리와 ROS 의 SIGINT 처리가 충돌하지 않도록 ROS 의 기본 핸들러를 비활성화. Qt 표준 종료 (Window close button → `QCoreApplication::quit()`) 가 main path. SIGINT 가 필요하면 후속 마일스톤에서 명시적 핸들러 추가.

### 6. `web_hmi` bridge JSON 키 정확성 검증

`web_hmi/scripts/web_hmi_bridge.py:_publish_periodic` 와 `web_hmi_threejs_bridge.py` 에서 발행하는 모든 키를 직접 확인하여 `RosBridge.cpp` 파서가 정확히 같은 키를 읽도록 함:

- `/hmi/state`: `speed, gear, mode, aeb, steering, ego{east,north,yaw}, gps{rtk,lon_std,lat_std}, speed_limit, link_id, lane_label, on_odd, road_state, selected_mode` ✔
- `/hmi/diagnostics`: `status: {gps:int, ...}` ✔
- `/hmi/objects`: `count, data:[{id,type,x,y,vx,vy,width,length,orientation,...}]` ✔
- `/hmi/popup`: `text, severity` ✔
- `/hmi/traffic`: `color, time_decisec, look_at:{intersection_id, signal_group_id}` ✔
- `/hmi/bag`: `recording, info` ✔
- `/hmi/map`: `polylines:[[[e,n],...],...]` ✔
- `/hmi/threejs/map`: `epsg, origin:[e0,n0], layers:{<name>:{kind, data}}`. `kind` 별 data shape:
  - `point`: `[[e,n],[e,n],...]` → `features[0]` 에 모두 모음.
  - `polyline`/`polygon`: `[[[e,n],...],[[e,n],...],...]` → `features[i]` 별 분리.
- `/hmi/threejs/tracks`: `stamp, tracks:[{id,tid,type,x,y,vx,vy,size_x,size_y,orientation,confidence,points?:[x0,y0,z0,...]}]` ✔ (`tid` 는 무시 — `id` 와 동일)

### 7. M1 demo UI (placeholder)

설계 §5/M1 검증 항목 4 (label 갱신) 충족하기 위해 6 개 라벨로 분리 표시:

- `/hmi/state` 1줄 (speed/gear/mode/steer/ego/rtk)
- `/hmi/diagnostics` 1줄 (모든 키 alphabetized)
- `/hmi/traffic`, `/hmi/popup`, `/hmi/bag`, `/hmi/threejs/tracks` 각 1줄

→ M1 단계에서 9개 토픽 중 7개의 도착을 시각적으로 확인 가능. `objectsChanged`, `mapPolylinesReceived`, `threejsMapReceived`, `hzChanged` 는 emit 만 되고 라벨 미연결 — M2 에서 위젯이 받기 시작.

---

## 빌드 검증 (자체 시도)

```
$ source /opt/ros/noetic/setup.bash
$ catkin_make --pkg qt_hmi
...
[100%] Built target qt_hmi_node
```

**결과: PASS** (exit 0). Qt5 5.12.8 자동 선택, gcc-9.4.0, C++17.

산출 binary:

```
$ ls -la /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
-rwxr-xr-x 1 sim sim 832416 May  9 15:28 .../qt_hmi_node
$ ldd .../qt_hmi_node | grep -E 'Qt5|roscpp'
libQt5Widgets.so.5 => /lib/x86_64-linux-gnu/libQt5Widgets.so.5
libroscpp.so       => /opt/ros/noetic/lib/libroscpp.so
libQt5Core.so.5    => /lib/x86_64-linux-gnu/libQt5Core.so.5
libQt5Gui.so.5     => /lib/x86_64-linux-gnu/libQt5Gui.so.5
libroscpp_serialization.so → 정상 링크
```

빌드 경고: 없음 (qt_hmi 자체).

> 참고: 다른 패키지에서 발생한 `WARNING: Target "..." requests linking to directory "/opt/ros/noetic/lib"` 메시지는 본 작업과 무관 (기존 stat_display, *_diagnostic_node 등의 catkin 설정 이슈로 main 브랜치에서도 출력되는 메시지).

---

## 다음 마일스톤 인계 (impl-verifier 로)

### 검증 포커스

1. **Qt5/Qt6 호환 헤더 prefix**: 모든 include 가 `<QtCore/...>`, `<QtWidgets/...>` 모듈 prefix 형식 → Qt5/Qt6 양립. M2 이후 위젯 추가 시 같은 컨벤션 유지 권장.
2. **메타타입 등록 누락 가능성**: M1 에서 8 종 등록. M2/M3 에서 추가 struct 도입 시 `qRegisterMetaType` 호출도 같이 추가해야 cross-thread emit 시 런타임 경고 없음.
3. **Race-safe 종료 동작**: `aboutToQuit → bridge->stop() → app.exec() 반환 → bridge.reset() → ros::shutdown()` 의 순서가 매우 중요. M2 에서 위젯이 늘어나도 main.cpp 의 이 순서는 변경 금지.

### 라이브 확인 권장

설계 §5/M1 검증 시나리오 (4번 항목까지) 를 라이브 ROS 환경에서 수행:

1. `roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false`
2. (별 터미널) `source /home/sim/mcar/devel/setup.bash && rosrun qt_hmi qt_hmi_node`
3. 빈 창 + 7 라벨 표시 확인.
4. `/hmi/state` 라벨이 1초 내 갱신되는지 (speed/gear/mode 표시).
5. 종료 (창 닫기) 후 segfault 없이 깨끗히 끝나는지.
6. `rostopic info /hmi/state | grep qt_hmi_node` → 구독자 등록 확인.

---

## 미해결 / 추정

1. **GLTF / 폰트 / shader 리소스 (resources/qt_hmi.qrc)**: M1 에서는 미생성. 설계 §3.A 디렉토리 트리에는 표기되어 있지만 M1 범위 밖이라 의도적으로 보류 (M2 에서 폰트, M3 에서 shader 추가).
2. **`/hmi/threejs/map` worker thread 파싱**: M3 에서 startup 지연 측정 후 결정. M1 에서는 동기 파싱이지만 latched + 한 번이라 GUI freeze 는 sub-second 예상.
3. **`ros::init_options::NoSigintHandler` vs Ctrl+C**: 터미널에서 Ctrl+C 종료 시 main.cpp 의 cleanup path 가 아닌 강제 종료 가능 → race 위험 잔존. 해결 필요 시 M1+ 패치로 `signal(SIGINT, ...)` 또는 Qt `QSocketNotifier` 핸들러 추가. M1 검증에서는 창 close 종료가 정식 path.
4. **Qt6 install 후 자동 전환 동작 검증 미완**: 현재는 Qt5 빌드만 실측 PASS. Qt6 미설치 환경이라 Qt6 path (`Qt6::Core/Gui/Widgets`) 의 실제 빌드는 사용자 Qt6 설치 후 별도 검증 필요. CMake 분기 자체는 syntax-correct.
5. **`devel-space wrapper` 미생성**: pyqt_hmi/web_hmi 처럼 Python wrapper 가 아닌 C++ binary 라 `qt_hmi_node` 가 직접 `devel/lib/qt_hmi/` 에 생성됨 (정상). `rosrun qt_hmi qt_hmi_node` 로 호출 가능 확인됨 (binary 파일 존재).

---

## 부록 — M2 진입 체크리스트

(impl-verifier 가 M1 검증 종료 후 M2 시작 시 참고)

- [ ] `include/qt_hmi/style/F1Tokens.h` 신규
- [ ] `include/qt_hmi/widgets/F1Sections.{h,cpp}` 신규 (SpeedHalf, SteerDial, TrafficLight, Section, Stat, Dot, HealthRow)
- [ ] `include/qt_hmi/widgets/F1Dashboard.{h,cpp}` 신규
- [ ] `MainWindow.cpp` 수정: 7 라벨 → `F1Dashboard` 교체. RosBridge signal 7→4 (state, diag, hz, traffic, popup, bag) 모두 F1Dashboard 슬롯에 라우팅.
- [ ] `CMakeLists.txt` 의 `# TODO M2` 슬롯에 신규 sources 등록.
- [ ] `resources/qt_hmi.qrc` 신규 (폰트 — JetBrainsMono OFL 체크 후 추가).
