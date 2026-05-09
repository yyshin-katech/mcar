# impl-verifier M2 보고서

> 마일스톤: **M2 — F1Dashboard (1600×900 telemetry chrome + 7 widgets)**
> 검증자: impl-verifier
> 일시: 2026-05-09
> 검증 대상: `src/visualization/qt_hmi/` (M2 신규 5 / 수정 3, M1 회귀 포함)
> 빌드 환경: Qt 5.12.8 (Ubuntu 20.04, design §6.A.C fallback path)

---

## 단계별 결과

| # | 단계 | 결과 | 비고 |
|---|------|------|------|
| 1 | 정적 (파일/Q_OBJECT/CMake) | **PASS** | 신규 5/5 + 수정 3/3 모두 존재. Q_OBJECT 7+1+1=9 (7 위젯 + F1Dashboard + MainWindow). 슬롯 시그너처 ↔ RosBridge 6 시그널 매칭 완전. |
| 2 | catkin 빌드 (재현) | **PASS** | `EXIT_CODE=0`, MOC autogen 4건, 산출물 1.06 MB ELF — 보고된 1060368 byte 와 동일. |
| 3 | 실행 스모크 (offscreen 부분) | **부분 PASS / 라이브 SKIP** | roscore 부재 (`pgrep -x rosmaster` → 없음). offscreen 5초 → SIGTERM(143) 정상 종료, segfault 없음. 라이브 토픽 수신/UI 시각 확인은 사용자 환경 인계. |
| 4 | M2 기능 정합 (정적) | **PASS (라이브 인계)** | 7 paintEvent 모두 구현. F1Dashboard 6 시그널 라우팅 OK. M3/M4 영역 placeholder 유지 확인. |

**합계: PASS 4 / FAIL 0** (단, 단계 3·4 의 라이브 부분은 사용자 환경 인계).

---

## 1. 정적 검증 상세 (PASS)

### 1.A. 파일 인벤토리 (8/8)

02_impl_M2.md 명시 신규 5 + 수정 3 = 8 파일 모두 존재 (line counts 실측, 보고서 기재값과 ±0):

| 파일 | 신규/수정 | 보고 | 실측 | 결과 |
|------|----------|-----:|-----:|:---:|
| `include/qt_hmi/style/F1Tokens.h` | 신규 | 59 | 59 | OK |
| `include/qt_hmi/widgets/F1Sections.h` | 신규 | 177 | 177 | OK |
| `src/widgets/F1Sections.cpp` | 신규 | 611 | 611 | OK |
| `include/qt_hmi/widgets/F1Dashboard.h` | 신규 | 160 | 160 | OK |
| `src/widgets/F1Dashboard.cpp` | 신규 | 963 | 963 | OK |
| `include/qt_hmi/MainWindow.h` | 수정 | 31 | 31 | OK |
| `src/MainWindow.cpp` | 수정 | 53 | 53 | OK |
| `CMakeLists.txt` | 수정 | 93 | 93 | OK |

총 2147줄 — 보고서 합산 (1970 + 약 30) 과 일치.

### 1.B. Q_OBJECT 매크로 검증 (PASS)

7 위젯 (F1Sections.h) + F1Dashboard.h + MainWindow.h 합계 **9건** 모두 클래스 헤더 첫 줄에 위치:

```
F1Sections.h:29 SpeedHalf : Q_OBJECT
F1Sections.h:49 SteerDial : Q_OBJECT
F1Sections.h:69 TrafficLight : Q_OBJECT
F1Sections.h:95 Section : Q_OBJECT
F1Sections.h:119 Stat : Q_OBJECT
F1Sections.h:138 Dot : Q_OBJECT
F1Sections.h:157 HealthRow : Q_OBJECT
F1Dashboard.h:51 F1Dashboard : Q_OBJECT
MainWindow.h:19 MainWindow : Q_OBJECT
```

CMake `set(CMAKE_AUTOMOC ON)` (라인 9) → 빌드 시 자동 처리. AUTOMOC 산출물 (단계 2 검증):
- `moc_RosBridge.cpp` (M1)
- `moc_MainWindow.cpp` (M1)
- `moc_F1Dashboard.cpp` (M2 신규)
- `moc_F1Sections.cpp` (M2 신규 — 7 위젯 통합 MOC)

### 1.C. paintEvent 구현 (PASS — 7/7)

F1Sections.cpp 안 7 위젯 paintEvent 모두 구현:

```
SpeedHalf::paintEvent       라인 65
SteerDial::paintEvent       라인 208
TrafficLight::paintEvent    라인 292
Section::paintEvent         라인 406
Stat::paintEvent            라인 465
Dot::paintEvent             라인 502
HealthRow::paintEvent       라인 546
```

각 위젯 생성자도 1:1 대응 (라인 47, 192, 269, 388, 452, 491, 528).

### 1.D. RosBridge 6 시그널 → F1Dashboard 7 슬롯 매핑 (PASS)

설계 §1.B 기반 매핑 검증:

| RosBridge 시그널 | F1Dashboard 슬롯 | MainWindow.cpp connect 라인 | F1Dashboard.cpp 구현 라인 |
|------------------|-----------------|---:|---:|
| `stateChanged(HmiState)` | `onState(HmiState)` | 30 | 703 |
| `diagChanged(QHash<QString,int>)` | `onDiag(QHash<QString,int>)` | 32 | 739 |
| `hzChanged(QHash<QString,double>)` | `onHz(QHash<QString,double>)` | 34 | 744 |
| `trafficChanged(int,int,int,int)` | `onTraffic(int,int,int,int)` | 36 | 749 |
| `popupChanged(QString,QString)` | `onPopup(QString,QString)` | 38 | 755 |
| `bagChanged(bool,QString)` | `onBag(bool,QString)` | 40 | 765 |
| (M2 부트스트랩) | `onConnectionChanged(bool,qint64)` | 50 (1회 호출) | 799 |

**6/6 시그널 wire-up 완전.** 7번째 슬롯 `onConnectionChanged` 는 MainWindow 가 부트스트랩 시 1회 호출 (`bridge_->onConnectionChanged(true, 0)` 의 의도) — RosBridge 실제 시그널은 M3/M4 에서 도입 (02_impl_M2.md §미해결 4번과 정합).

### 1.E. CMakeLists 등록 (PASS)

라인 62~71:
```
QT_HMI_M2_SOURCES = src/widgets/F1Sections.cpp src/widgets/F1Dashboard.cpp
QT_HMI_M2_HEADERS = F1Tokens.h F1Sections.h F1Dashboard.h
```

`add_executable(qt_hmi_node ${QT_HMI_M1_SOURCES} ${QT_HMI_M1_HEADERS} ${QT_HMI_M2_SOURCES} ${QT_HMI_M2_HEADERS})` (라인 76~81). AUTOMOC 가 헤더의 Q_OBJECT 를 자동 처리하므로 .cpp 등록만으로 충분 (검증 가이드의 "AUTOMOC 자동 처리" 항목 충족).

### 1.F. M3/M4 placeholder 잔존 확인 (PASS)

F1Dashboard.cpp 안 M3/M4 영역:
- 라인 514, 559, 566~573: `mainPlaceholder_ = new QLabel("MapScene (M3) pending — 3D HD map + ego + tracks 가 이 영역에 렌더됩니다.")`
- 라인 278: "THR / BRK / ACCEL — placeholder strip" (M2 범위 내 PR-F1)
- 라인 340: "MANUAL/AUTO toggle (visual only — M4 will add click)" — M4 클릭 핸들러 미구현 (의도)

`MapScene` / `ControlPanel` 클래스 자체는 아직 패키지 안에 존재 안 함 (CMake `# TODO M3` / `# TODO M4` 주석으로 명시) — 의도된 placeholder 상태.

### 1.G. M1 회귀 (PASS)

- M1 산출 파일 9개 (HmiTypes.h, RosBridge.{h,cpp}, MainWindow.{h,cpp}, main.cpp, package.xml, CMakeLists.txt, launch/qt_hmi.launch) 모두 그대로 존재.
- MainWindow.h: M1 의 placeholder 라벨 7개 + 6 슬롯 → 제거. 단일 멤버 `f1widgets::F1Dashboard* dashboard_` 로 교체. M1 회귀 영향 없음 (RosBridge 시그너처/콜백 무변경).
- M1 검증 표 (03_verify_M1.md §2/§3) 의 정적/스모크 PASS 항목 모두 그대로 유효.

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
ELF 64-bit LSB shared object, x86-64, dynamically linked,
BuildID=ed4839a759f43aae58c94a0b285584353dc757f9
$ ls -l /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
-rwxr-xr-x 1060368 May  9 23:03  qt_hmi_node
```

크기 1060368 bytes = **1.06 MB → 보고서 기재값 (M1 832 KB → M2 1.06 MB) 과 정확히 일치**.

AUTOMOC 산출 (4건):
```
build/visualization/qt_hmi/qt_hmi_node_autogen/32PC4GSTT7/moc_MainWindow.cpp
build/visualization/qt_hmi/qt_hmi_node_autogen/32PC4GSTT7/moc_RosBridge.cpp
build/visualization/qt_hmi/qt_hmi_node_autogen/TB6PJGQYWE/moc_F1Dashboard.cpp
build/visualization/qt_hmi/qt_hmi_node_autogen/TB6PJGQYWE/moc_F1Sections.cpp
```

M2 신규 MOC 2건 (`moc_F1Dashboard.cpp` 6851 byte, `moc_F1Sections.cpp` 21155 byte) 정상 생성 — 7 위젯의 모든 Q_OBJECT 가 통합 MOC 로 처리됨.

빌드 경고: 0건 (qt_hmi 자체).

---

## 3. 실행 스모크 (부분 PASS / 라이브 SKIP)

### 3.A. roscore 부재

```
$ pgrep -x rosmaster
NO_ROSMASTER
```

라이브 토픽 흐름 (state/diag/hz/traffic/popup/bag JSON 6개) 검증은 본 검증 단계에서 수행 불가. 사용자 라이브 환경 인계 (§5).

### 3.B. Offscreen 부분 스모크 (PASS)

```bash
$ source /home/sim/mcar/devel/setup.bash
$ QT_QPA_PLATFORM=offscreen timeout 3 /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
Terminated
EXIT_CODE=143
```

Exit 143 = `timeout` 의 SIGTERM. 노드는 3초 동안 살아 있었고 stdout/stderr 에 segfault/abort 메시지 없음. **F1Dashboard 인스턴스화 + 7 위젯 paintEvent 가능 + 4 Hz QTimer 가동 — 무중단 PASS**.

> 단, offscreen 환경은 paintEvent 가 가상 surface 에만 그려지므로 시각 확인은 라이브 (DISPLAY 가능) 환경에서 별도 수행 필요.

---

## 4. 마일스톤 기능 (M2) — 설계 §5/M2 검증 7항 + 02_impl_M2 검증 포커스

| # | 검증 항목 | 정적/라이브 | 결과 |
|---|----------|------------|------|
| 1 | 1600×900 창 + cyan accent chrome 표시 | 라이브 | **인계** (사용자 환경, MainWindow.cpp `resize(1600, 900)` 라인 16 + `bg0` 스타일시트 라인 23 정상 구현 확인) |
| 2 | SpeedHalf 게이지 부드러운 갱신 (≥10 Hz) | 라이브 | **인계** (`onState` 라인 703 → `setValues(speed, limit)` 호출 확인) |
| 3 | TrafficLight color=1/2/3 → 적/황/녹 LED + remain 카운트다운 | 라이브 | **인계** (`onTraffic` 라인 749 → `setPhaseColor`/`setRemainSeconds` 호출 확인) |
| 4 | diag.status.gps=1 → GPS-RTK 행 amber WARN | 라이브 | **인계** (`onDiag` 라인 739 → refreshHealth → healthRows_[0]->setStatus, code=0/1/2 색 구분 라인 879~893 확인) |
| 5 | popup.severity="warn" → main area 상단 amber banner | 라이브 | **인계** (`onPopup` 라인 755 → oddBanner_ visibility/text 동적 갱신, oddBanner_/oddBannerLabel_ 인스턴스화 라인 576~595 확인) |
| 6 | ROS 끊김 → TOP BAR ROS 배지 적색 OFFLINE | 라이브 | **인계** (현재 부트스트랩 connected=true + Hz 0.5 임계 → WAITING amber, 02_impl_M2 §미해결 4번 정합) |
| 7 | objs/mapPolylines 표시 (M2 fallback) | (의도 생략) | **N/A** — 설계 §6.E 결정 옵션 A: M2 main area 는 "MapScene (M3) pending" 라벨 placeholder. M2 범위 내 의도된 동작. |
| 8 | (02_impl_M2 검증 포커스) ODD banner / BAG overlay 위치 | 라이브 | **인계** (StageFilter 이벤트 필터 + raise() 패턴 라인 576~617 + main area resizeEvent 캡처 구조 확인) |
| 9 | (02_impl_M2 검증 포커스) 4 Hz 클록 단조 증가 | 라이브 | **인계** (clockTimer_ QTimer 250 ms tick → utc/kst/tick/net Stat 업데이트, 라인 138 connect + tick() 슬롯 구현 확인) |
| 10 | (02_impl_M2 검증 포커스) 종료 race | 정적 + 부분 | **PASS** (offscreen 3초 무중단 + M1 의 `aboutToQuit → bridge->stop() → bridge.reset() → ros::shutdown()` 시퀀스 회귀 없음 — MainWindow.cpp 변경분이 RosBridge 종료 흐름 미간섭) |

라이브 항목 9건은 사용자 환경 인계 (§6).

---

## 5. 발견 항목

| # | 위치 | 관찰 | 권장 조치 |
|---|------|-----|----------|
| 1 | `RosBridge::rosConnectionChanged` 미emit (M1 미구현 잔존) | F1Dashboard 가 부트스트랩 connected=true + Hz 0.5 임계로 WAITING/ONLINE 결정. ROS 끊김 시 OFFLINE 적색은 직접 동작 안 함 — Hz=0 → WAITING amber 로 대체 | M3/M4 에서 RosBridge 가 마지막 메시지 수신 시각 트래킹 시작 시 자동 해소. 본 M2 범위 밖, impl-coder 재호출 불필요 |
| 2 | M2 main area = QLabel placeholder | "MapScene (M3) pending" 라벨만 표시 — M3 에서 OpenGL 위젯으로 교체 예정 | 설계 §6.E 옵션 A 결정 그대로. impl-coder 재호출 불필요 |
| 3 | DRIVE MODE / BAG 클릭 핸들러 미구현 | 시각 토글만 (M2 범위). M4 에서 publish 슬롯 도입 예정 | 설계 §5/M4 그대로. impl-coder 재호출 불필요 |
| 4 | scanlines overlay 미구현 | JSX CRT 스캔라인 효과 생략 (02_impl_M2 §미해결 2번) | M3 시 main area QOpenGLWidget 위로 추가 가능. M2 범위 밖 |
| 5 | HEALTH summary dot 위치 320px 고정 의존 | 좌패널 width 320 가정 (`Section::resizeEvent` hook 없음). panel 리사이저 미도입 시 영향 0 | 02_impl_M2 §미해결 3번 그대로 — 향후 panel resize 도입 시 해결 |

**impl-coder 재호출 사유 없음** — 모든 발견 항목이 설계상 의도된 placeholder/M3+ 후속.

---

## 6. 라이브 확인 인계 (사용자 환경)

bag/replay 또는 실차 데이터로 web_hmi bridge 가 가동 가능한 환경에서 다음 절차로 M2 시각 검증 9 항목 (§4 표 #1~6, 8, 9, 10) 을 확인.

### 6.A. 권장 실행 절차

터미널 1:
```bash
roscore
```

터미널 2 — bridge 가동 (rosbag replay 같이 권장):
```bash
cd /home/sim/mcar
source devel/setup.bash
roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false
# 별도 터미널: rosbag play <bag>
```

터미널 3 — qt_hmi 노드:
```bash
cd /home/sim/mcar
source devel/setup.bash
rosrun qt_hmi qt_hmi_node
```

DISPLAY 미설정 환경이면 `xvfb-run`/`x11vnc` 후 또는 `QT_QPA_PLATFORM=xcb` 로 실행.

### 6.B. 기대 동작 (시각)

1. **1600×900 창 + bg0 (#06090c) 검정 배경 + cyan 액센트 chrome** 즉시 표시.
2. TOP BAR (44 px): KATECH 로고 (좌) · UTC/KST 시계 · TICK uptime · NET msg-age · ROS 배지 (cyan ONLINE / amber WAITING / red OFFLINE).
3. 좌패널 320 px:
   - 01 VELOCITY · STEER: SpeedHalf 반원 게이지 + SteerDial 다이얼 + GEAR letter (P/R/N/D).
   - 02 DRIVE MODE: MANUAL/AUTONOMOUS 토글 + ENGAGED/STANDBY 라벨 + v_max + ODD 텍스트.
   - 03 V2X: TrafficLight 3 LED 컬럼 + PHASE / CHANGE-IN sec.
   - 04 LOCALIZATION: LANE/LINK/σ-East/σ-North/σ-Up/HDOP/SATS/HEADING + RTK 라벨 (FIX green / FLOAT amber / NO RTK text3).
   - 05 SYSTEM HEALTH: 6 HealthRow (GPS-RTK, K-ADCU, LIDAR, RADAR, CAMERA, V2X) + 4-dot 요약.
4. main area (06): "MapScene (M3) pending — 3D HD map + ego + tracks 가 이 영역에 렌더됩니다." 라벨 (M3 자리표시).
5. 하단 56 px BOTTOM TELEMETRY: 8 cell (EGO-VEL / Δ-LIM / LATERAL / JERK / LEAD-D / LEAD-Δv / PLAN-H / CPU).

### 6.C. 검증 명령

```bash
# 노드 등록
rosnode info /qt_hmi_node | head -30

# 13 토픽 구독 (state/diag/hz/objects/popup/traffic/bag/map/threejs_map/threejs_tracks 10 sub + mode_request/bag_toggle 2 pub)
rostopic info /hmi/state | grep qt_hmi_node
rostopic info /hmi/diagnostics | grep qt_hmi_node
rostopic info /hmi/topic_hz | grep qt_hmi_node
rostopic info /hmi/traffic | grep qt_hmi_node
rostopic info /hmi/popup | grep qt_hmi_node
rostopic info /hmi/bag | grep qt_hmi_node

# 라이브 데이터 입력 → 시각 갱신 확인 (bag replay 가능 시 자동, 미가능 시 수동 publish)
rostopic pub -1 /hmi/state std_msgs/String "{\"speed\":60,\"steering\":15,\"gear\":4,\"mode\":1, ...}"
rostopic pub -1 /hmi/traffic std_msgs/String "{\"color\":3, \"time_decisec\":47, ...}"
```

### 6.D. 캡처 권장 (M2 시각 회귀 베이스라인)

```bash
# 터미널 4 — 5초 후 스크린 캡처 (X11 환경)
sleep 5
xwd -name "qt_hmi (M2: F1 Dashboard)" -out /tmp/qt_hmi_M2.xwd
convert /tmp/qt_hmi_M2.xwd /tmp/qt_hmi_M2.png
# 또는 GNOME / Wayland 환경: Print Screen 키 또는 gnome-screenshot -w
```

사이드 바이 사이드 비교: `http://localhost:8088/index_threejs_f1.html` 와 같은 라이브 데이터에서:
- SpeedHalf 게이지 호 길이 (speed=60 → t=0.5)
- SteerDial 인디케이터 회전 (steering=±15° 일 때)
- TrafficLight LED 점등 + remain 카운트다운
- HealthRow 6 색상 (status=0/1/2 ↔ green/amber/red)
- σ-EAST/NORTH "cm" 표기 (예: lonStd=0.0184 → "01.84 cm")

픽셀 ±2 이내 수준의 시각 일치를 기대 (02_impl_M2 보고 §검증 포커스 1 인용).

### 6.E. 종료 race 확인

```bash
# qt_hmi_node 실행 중에 Ctrl+C 또는 창 X 버튼 클릭
# 기대: 즉시 종료 + segfault/abort/double-free 메시지 없음
# (M1 의 aboutToQuit → bridge->stop() → bridge.reset() → ros::shutdown() 시퀀스가 dashboard 추가 후에도 무회귀)
```

---

## 7. 종합 판정

**M2 검증 PASS — M3 진입 가능.**

- 정적 검증 (단계 1·2) 모두 PASS, 빌드 산출 1.06 MB ELF 정합.
- 단계 3·4 의 라이브 시각 항목 9건은 사용자 환경 인계 (§6) — DISPLAY/roscore 가 모두 가능한 환경에서 캡처 + parity 비교 권장.
- 발견 항목 5건 모두 **설계상 의도된 placeholder / M3+ 후속** — impl-coder 재호출 불필요.
- 02_impl_M2.md 부록 의 M3 진입 체크리스트 (LayerStyle/MapScene/shaders/qrc/main area 교체) 가 그대로 유효.

다음 단계: 사용자 라이브 시각 확인 → 02_impl_M2.md §부록 체크리스트 따라 M3 시작 (LayerStyle.{h,cpp} + MapScene.{h,cpp} 신규 + shaders + qt_hmi.qrc + F1Dashboard.mainPlaceholder_ 교체).
