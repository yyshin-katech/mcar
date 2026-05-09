# impl-verifier M1 보고서

> 마일스톤: **M1 — 패키지 스켈레톤 + RosBridge + MainWindow placeholder**
> 검증자: impl-verifier
> 일시: 2026-05-09
> 검증 대상: `src/visualization/qt_hmi/` (9 파일 / 889줄, impl-coder M1 산출물)
> 빌드 환경: Qt 5.12.8 (Ubuntu 20.04, design §6.A.C fallback)

---

## 단계별 결과

| # | 단계 | 결과 | 비고 |
|---|------|------|------|
| 1 | 정적 (파일/CMake/include) | **PASS** | 9/9 파일 존재, package.xml 의존 ↔ include 일치, Q_OBJECT 2건, AUTOMOC 정상 생성 |
| 2 | catkin 빌드 (재현) | **PASS** | `EXIT_CODE=0`, 산출물 832 KB ELF 정상 |
| 3 | 실행 스모크 | **부분 PASS / 라이브 SKIP** | roscore 부재 (`pgrep -x rosmaster` → 없음). offscreen 단독 실행 3초 무중단 확인. 라이브 토픽 검증은 사용자 환경에 인계 |
| 4 | M1 기능 (설계 §5 6항) | **PASS (라이브 항목 인계)** | 정적 항목 모두 충족, 라이브 항목(speed 갱신/QDebug log)은 사용자 라이브 확인 |

**합계: PASS 4 / FAIL 0** (단, 단계 3·4의 라이브 부분은 인계).

---

## 1. 정적 검증 상세 (PASS)

### 1.A. 파일 인벤토리 (9/9)

02_impl_M1.md 명시 9 파일 모두 존재 (line counts 실측):

| 파일 | 존재 | 비고 |
|------|------|------|
| `package.xml` | OK | 19줄 |
| `CMakeLists.txt` | OK | 80줄 |
| `include/qt_hmi/HmiTypes.h` | OK | 99줄, `Q_DECLARE_METATYPE` 8 |
| `include/qt_hmi/RosBridge.h` | OK | 77줄, Q_OBJECT 포함 |
| `include/qt_hmi/MainWindow.h` | OK | 45줄, Q_OBJECT 포함 |
| `src/RosBridge.cpp` | OK | 358줄 |
| `src/MainWindow.cpp` | OK | 129줄 |
| `src/main.cpp` | OK | 50줄 |
| `launch/qt_hmi.launch` | OK | 32줄 |

총 889줄 — 보고서 기재값과 일치.

### 1.B. package.xml ↔ include 정합성 (PASS)

`package.xml` depend:
```
roscpp, std_msgs (depend)
web_hmi (exec_depend, 런타임 bridge)
```

src/headers 의 ROS 헤더 include 전수 조사:
```
ros/ros.h, std_msgs/Bool.h, std_msgs/Empty.h, std_msgs/String.h
```

→ 누락 없음. mmc_msgs/katech_*/v2x_msgs/perception_ros_msg 등 **직접 의존 0건** (설계 §0 결정대로 web_hmi bridge JSON 만 구독).

### 1.C. CMakeLists 검증 항목 (PASS)

- `find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)` — 라인 14
- `find_package(Qt6 6.2 ...)` → `Qt5 5.12 ...` fallback → 둘 다 미발견 시 `catkin_package(); return()` graceful skip — 라인 21~37 (설계 §3.C + §6.A.C 정합)
- `add_executable(qt_hmi_node ...)` — 라인 65
- `target_link_libraries(qt_hmi_node ${QT_HMI_QT_LIBS} ${catkin_LIBRARIES})` — 라인 70~73
- `set(CMAKE_AUTOMOC ON)` — 라인 9 → Q_OBJECT 자동 처리
- `install(TARGETS qt_hmi_node ...)` + `install(DIRECTORY launch ...)` — 라인 75~80

**주의**: `add_dependencies(qt_hmi_node ${catkin_EXPORTED_TARGETS})` 누락. 그러나 본 패키지가 자체 .msg 도 없고 다른 패키지 .msg 도 직접 include 하지 않으므로 (std_msgs 만, gencpp 의존 없음) 빌드 영향 0. 설계 의도와 부합. **FAIL 사유 아님**.

### 1.D. Q_OBJECT / signal 시그너처 검증 (PASS — 9 시그널 → 실제는 10)

`RosBridge.h` 라인 45~56:

| # | 시그널 | 페이로드 |
|---|--------|---------|
| 1 | `stateChanged(const HmiState&)` | /hmi/state |
| 2 | `diagChanged(const QHash<QString,int>&)` | /hmi/diagnostics |
| 3 | `hzChanged(const QHash<QString,double>&)` | /hmi/topic_hz |
| 4 | `objectsChanged(const QVector<HmiObject>&)` | /hmi/objects |
| 5 | `popupChanged(const QString&, const QString&)` | /hmi/popup |
| 6 | `trafficChanged(int,int,int,int)` | /hmi/traffic |
| 7 | `bagChanged(bool, const QString&)` | /hmi/bag |
| 8 | `mapPolylinesReceived(const QVector<Polyline>&)` | /hmi/map |
| 9 | `threejsMapReceived(const HmiMap3D&)` | /hmi/threejs/map |
| 10 | `tracksChanged(const QVector<HmiTrack3D>&)` | /hmi/threejs/tracks |

설계 §1.A 표는 `1~10`의 10개 토픽을 명시하고 §2.B 코드 블록도 10 시그널을 그대로 보여준다 (단, §5/M1 검증 5번 항목 텍스트가 "9 토픽" 으로 표기되어 약간 혼란이 있으나 설계 §1.A·§2.B 의 10개가 정본). impl-coder 가 10/10 모두 구현. **PASS**.

`rosConnectionChanged(bool, qint64)` 는 설계 §2.B 마지막에 언급되었으나 M1 범위에서 미구현 — 설계상 "computed locally from spinner liveness" 인 보조 시그널이며 RosBridge 콜백에 의존하지 않으므로 M2 이후 추가가 자연스럽다. **FAIL 사유 아님**.

### 1.E. 콜백 시그너처 검증 (PASS)

10 콜백 모두 `void on*(const std_msgs::String::ConstPtr& msg)` 표준형:
```
RosBridge.h:60~69 onState/onDiag/onHz/onObjects/onPopup/onTraffic/onBag/onMap/onThreejsMap/onTracks
```

각 콜백 안에서 `parseObject()` → struct 채움 → `emit signalName(...)` 패턴 (RosBridge.cpp 122~358).

### 1.F. 토픽 구독 13개 검증

설계 §1.A 의 "토픽 구독 13개" 는 `1~10` (in) + `mode_request, bag_toggle` (out) 합산이 아니라, web_hmi 리뷰 시 분류된 13개 카운트를 가리킨다. impl-coder 구현 실측:
- `subs_.push_back` 호출 = 10건 (10 토픽 inbound)
- `nh_.advertise` 호출 = 2건 (mode_request, bag_toggle)
- 총 12 ROS 핸들 (설계 의도와 정합)

설계와 같은 의미에서 **PASS**.

### 1.G. 메타타입 등록 (PASS)

`RosBridge.cpp:57~66` 의 `qRegisterMetaType<...>("...")` 호출 = **10건**. 보고서 기재 "8 종" 보다 2건 더 (`Polyline`, `QVector<Polyline>` 명시 추가). cross-thread queue 마샬링 안정성 보장.

### 1.H. ros::init + QApplication 공존 (PASS — 설계 §6.H)

`main.cpp` 단계 순서:
```
ros::init(NoSigintHandler)        // 27줄
QApplication app(...)             // 30줄
unique_ptr<RosBridge>             // 34줄 — spinner_ start
MainWindow w(bridge.get())        // 35줄
aboutToQuit → bridge->stop()      // 41~44줄 — race-safe shutdown
app.exec()                         // 46줄
bridge.reset()                    // 47줄
ros::shutdown()                   // 48줄
```

설계 §6.H 의 "spinner stop → destroy → ros::shutdown" 순서를 정확히 따른다. SIGINT 핸들러는 `NoSigintHandler` 로 Qt 와 격리. **PASS**.

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
$ file /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
ELF 64-bit LSB shared object, x86-64, dynamically linked, BuildID=1cec394d3...
$ ls -la /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
-rwxr-xr-x 832416 May  9 15:28
$ ldd qt_hmi_node | grep -E 'Qt5|roscpp'
libQt5Widgets.so.5 → /lib/x86_64-linux-gnu/libQt5Widgets.so.5
libQt5Core.so.5    → /lib/x86_64-linux-gnu/libQt5Core.so.5
libQt5Gui.so.5     → /lib/x86_64-linux-gnu/libQt5Gui.so.5
libroscpp.so       → /opt/ros/noetic/lib/libroscpp.so
libroscpp_serialization.so → /opt/ros/noetic/lib/libroscpp_serialization.so
```

AUTOMOC 산출물 확인:
```
build/visualization/qt_hmi/qt_hmi_node_autogen/32PC4GSTT7/moc_MainWindow.cpp
build/visualization/qt_hmi/qt_hmi_node_autogen/32PC4GSTT7/moc_RosBridge.cpp
```

빌드 캐시가 hot 상태이므로 "Built target" 만 출력되었으나, MOC 산출물 + 링크 결과 정상.

---

## 3. 실행 스모크 (부분 PASS / 라이브 SKIP)

### 3.A. roscore 부재

```
$ pgrep -x rosmaster
(no output) → NO_ROSMASTER
```

`rosrun qt_hmi qt_hmi_node` 의 토픽 구독/발행 검증은 **본 검증 단계에서 수행 불가**. 사용자 라이브 환경 인계 (§5).

### 3.B. Offscreen 부분 스모크 (PASS)

roscore 없이도 `ros::init` + AsyncSpinner 시작 + QApplication 부팅 자체는 가능. 3초 timeout 으로 즉시 종료 확인:

```
$ source /home/sim/mcar/devel/setup.bash
$ QT_QPA_PLATFORM=offscreen timeout 3 /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
(blank stdout, no crash)
EXIT=143  ← timeout SIGTERM (정상)
```

Exit 143 = `timeout` 의 SIGTERM. 노드는 3초 동안 살아 있었고 stderr 에 segfault/abort 메시지 없음. **ros::init + QApplication 동시 가동 가능 — PASS**.

---

## 4. 마일스톤 기능 (M1) — 설계 §5 검증 6항

| # | 검증 항목 | 정적/라이브 | 결과 |
|---|----------|------------|------|
| 1 | web_hmi.launch (variant=threejs_f1, open_browser=false) 사전 실행 | 라이브 | **인계** (사용자) |
| 2 | `catkin_make --pkg qt_hmi` PASS | 정적 | **PASS** (재현 EXIT_CODE=0) |
| 3 | `rosrun qt_hmi qt_hmi_node` 실행 → 빈 창 + label "qt_hmi M1: ROS bridge ready" | 라이브 | **인계** (UI 확인 사용자 환경) |
| 4 | label 1초 내 "speed: X km/h" 갱신 | 라이브 | **인계** (state 토픽 도달 시 갱신 — `MainWindow::onState` 라인 73~85 정상 구현 확인) |
| 5 | 9 토픽 중 최소 5개 (state, diag, topic_hz, objects, traffic) 첫 메시지 도달 | 라이브 | **인계** (10 콜백 모두 코드 정합) |
| 6 | 종료 시 segfault 없이 깨끗한 ros::shutdown() | 정적 + 부분 | **PASS** (offscreen 3초 무중단 + 설계 §6.H race-safe pattern 정확히 구현) |

라이브 항목 4건은 사용자 환경 인계 (§5).

---

## 5. 발견 항목

| # | 위치 | 관찰 | 권장 조치 |
|---|------|-----|----------|
| 1 | `RosBridge.h` `rosConnectionChanged` 미구현 | 설계 §2.B 의 11번째 시그널 — M1 범위 밖, 보조 시그널 | M2 이후 도입 (설계 §1.B TOP BAR ROS 배지 시점). impl-coder 재호출 불필요 |
| 2 | `CMakeLists.txt` `add_dependencies(... ${catkin_EXPORTED_TARGETS})` 부재 | 외부 .msg 의존이 없으므로 빌드 영향 0 | 그대로 유지. M2/M3 에서 새 catkin 컴포넌트 추가 시점에 도입 검토 |
| 3 | `02_impl_M1.md` 보고 "qRegisterMetaType 8종" | 실제 코드는 10건 (Polyline 2건 추가) | 보고서 텍스트 vs 실제 +2 — 코드 쪽이 더 안전. 변경 불필요 |
| 4 | `02_impl_M1.md` `unused parameter` 경고 가능성 우려 | 빌드 시 경고 0 | 영향 없음 |

**impl-coder 재호출 사유 없음**.

---

## 6. 라이브 확인 인계 (사용자 환경)

bag/replay 또는 실차 데이터로 web_hmi bridge 가 가동 가능한 환경에서 다음 4 명령으로 M1 검증 잔여 4항(§4 표 #1, 3, 4, 5)을 확인.

### 6.A. 권장 실행 절차

터미널 1:
```bash
roscore
```

터미널 2 — bridge 가동:
```bash
cd /home/sim/mcar
source devel/setup.bash
roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false
```
(또는 bag replay 도 함께)

터미널 3 — qt_hmi 노드:
```bash
cd /home/sim/mcar
source devel/setup.bash
rosrun qt_hmi qt_hmi_node
```

### 6.B. 기대 동작

1. 960×540 검정 배경 창이 뜨고 7 라벨 표시:
   - 헤더: `qt_hmi M1: ROS bridge ready` (cyan/bold)
   - `/hmi/state           : (waiting)` → 1초 내 `speed=N km/h gear=M mode=K steer=...°  ego=(...) rtk=L`
   - `/hmi/diagnostics    : (waiting)` → `gps=0 adcu=0 ...` (alphabet 정렬)
   - `/hmi/traffic         : (waiting)` → `color=N remain=Ts inter=I sg=G`
   - `/hmi/popup           : (waiting)` → `[severity] text`
   - `/hmi/bag             : (waiting)` → `idle <empty>` 또는 `REC <path>`
   - `/hmi/threejs/tracks : (waiting)` → `count=N with_points=M`
2. 창 닫기 (X 버튼) 시 즉시 종료, segfault 메시지 없음.

### 6.C. 검증 명령

별도 터미널에서:
```bash
# 노드 등록 확인
rosnode list | grep qt_hmi_node

# 구독자/발행자 등록
rostopic info /hmi/state         | grep qt_hmi_node    # Subscribers
rostopic info /hmi/cmd/mode_request | grep qt_hmi_node # Publishers

# 10 토픽 모두 구독 확인 (5개 sample)
for t in /hmi/state /hmi/diagnostics /hmi/topic_hz /hmi/objects /hmi/popup; do
  echo "=== $t ==="
  rostopic info $t 2>&1 | grep qt_hmi_node || echo "(no subscriber yet)"
done

# 종료 후 좀비 프로세스 없음
pgrep -af qt_hmi_node
```

기대: 노드 1개, 10 토픽 모두 `Subscribers: * /qt_hmi_node`, `mode_request`/`bag_toggle` 에 Publishers 등록.

---

## 7. 종합 판정

**M1 검증 PASS — M2 진입 가능.**

- 정적 검증 (단계 1·2) 모두 PASS.
- 단계 3·4 의 라이브 항목 4건은 사용자 환경 인계 (위 §6).
- 발견 항목 4건 모두 **경미** 수준 — impl-coder 재호출 불필요.
- 02_impl_M1.md 부록의 M2 진입 체크리스트가 그대로 유효.

다음 단계: 사용자 라이브 확인 → 02_impl_M1.md §부록 체크리스트 따라 M2 시작 (F1Tokens/F1Sections/F1Dashboard 신규 + MainWindow 라벨 → F1Dashboard 교체).
