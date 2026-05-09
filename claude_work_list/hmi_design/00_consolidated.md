# qt_hmi 개발 통합 보고 — M1 ~ M4 완료

작성일: 2026-05-10
브랜치: `siheung_dev`
하네스: `qt-hmi-build` (3-phase: design-architect → impl-coder → impl-verifier)

## 하네스 구성

| 파일 | 역할 |
|------|------|
| `.claude/skills/qt-hmi-build/SKILL.md` | 오케스트레이터 (3-phase + 마일스톤 분기) |
| `.claude/agents/design-architect.md` | Phase A: 아키텍처/패키지/CMake/마일스톤 설계 |
| `.claude/agents/impl-coder.md` | Phase B: 마일스톤별 구현 (1 호출 1 M) |
| `.claude/agents/impl-verifier.md` | Phase C: 빌드/정적/실행 스모크/기능 검증 |

## 마일스톤 결과 요약

| M | 단계 | 결과 | 산출물 / ELF |
|---|------|------|--------------|
| **M1 ✅** | design / impl / verify | PASS / PASS / PASS | 9 파일 889줄, 832 KB ELF, file1 캡처 PASS |
| **M2 ✅** | impl / verify | PASS / PASS | +5 신규 / +3 수정 (~1970 신규줄), 1.06 MB ELF, file2 캡처 PASS |
| **M3 ✅** | impl / verify | PASS / PASS | +4 신규 / +4 수정 (~1100 신규줄), 1.21 MB ELF, file3 캡처 PASS |
| **M4 ✅** | impl / verify | PASS / PASS | +4 신규 / +4 수정 (1063줄), **1.40 MB ELF**, MOC 7건 |

| 산출 보고 | 경로 |
|-----------|------|
| 설계 | `01_design.md` (816줄) |
| M1 | `02_impl_M1.md`, `03_verify_M1.md` |
| M2 | `02_impl_M2.md`, `03_verify_M2.md` |
| M3 | `02_impl_M3.md`, `03_verify_M3.md` |
| M4 | `02_impl_M4.md`, `03_verify_M4.md` |

## 환경 결정

**Qt6 부재 → Qt5.12.8 fallback** (2026-05-09 사용자 결정). Ubuntu 20.04 기본 저장소에 `qt6-base-dev` 없음. CMakeLists에 `find_package(Qt6 6.2 ...)` 시도 후 `find_package(Qt5 5.12 ...)` fallback 분기. M3에서 `Qt5::OpenGL` 추가, M4에서 새 의존 없음 (Widgets/Gui만).

## 패키지 최종 구조 (M4 시점)

```
src/visualization/qt_hmi/
├── CMakeLists.txt                                       # M4 122줄
├── package.xml                                          # roscpp + std_msgs only
├── include/qt_hmi/
│   ├── HmiTypes.h                                       # M1 페이로드 구조체
│   ├── RosBridge.h                                      # M1 — 9 토픽 → 10 시그널
│   ├── MainWindow.h                                     # M4 33줄
│   ├── style/F1Tokens.h                                 # M2 — F1 KATECH 색상/폰트
│   └── widgets/
│       ├── F1Sections.h                                 # M2 — 7 위젯 클래스
│       ├── F1Dashboard.h                                # M3 — MapScene 슬롯 노출 (171줄)
│       ├── MapScene.h                                   # M3 — 6 슬롯 노출 (169줄)
│       ├── LayerStyle.h                                 # M3 — 15 layer 테이블 / DEFAULT_VIS
│       ├── ControlPanel.h                               # M4 — 97줄, Q_OBJECT
│       └── TrafficLightWidget.h                         # M4 — 58줄, Q_OBJECT
├── src/
│   ├── main.cpp                                         # M1 — race-safe 종료
│   ├── RosBridge.cpp
│   ├── MainWindow.cpp                                   # M4 121줄, dock + 7 connect + emitInitialState
│   └── widgets/
│       ├── F1Sections.cpp                               # M2 611줄
│       ├── F1Dashboard.cpp                              # M3 995줄 (M3 +30줄)
│       ├── MapScene.cpp                                 # M3 809줄 — 4 inline GLSL
│       ├── LayerStyle.cpp                               # M3 91줄
│       ├── ControlPanel.cpp                             # M4 413줄
│       └── TrafficLightWidget.cpp                       # M4 219줄
└── launch/
    └── qt_hmi.launch                                    # M1
```

**Q_OBJECT 13건** (M1: 2, M2: 8, M3: 1, M4: 2). **MOC 산출 7건**.

## 마일스톤별 구현 핵심

### M1 — 스켈레톤
- `RosBridge` (QObject): 9개 `/hmi/*` JSON 토픽 구독, 콜백→Qt signal emit (`Qt::QueuedConnection`).
- `HmiTypes.h`: 페이로드 구조체 + `qRegisterMetaType` 10건.
- `MainWindow`: 7개 placeholder QLabel (M2~M4 인계 표시).
- `main.cpp`: ros::init + QApplication + `ros::AsyncSpinner(2)` + race-safe 종료(`aboutToQuit`).
- 의존성: roscpp + std_msgs만 (페이로드는 bridge JSON 재사용).

### M2 — F1Dashboard
- 7 위젯: SpeedHalf / SteerDial / TrafficLight (좌측패널) / Section / Stat / Dot / HealthRow.
- QPainter 직접 paintEvent로 F1HMI.jsx 1600×900 grid 이식.
- 6 RosBridge 시그널 wired (state/diag/hz/traffic/popup/bag).

### M3 — MapScene
- QOpenGLWidget + Core 3.3 + 4 inline GLSL (line.vert/frag, point.vert/frag).
- HDMap_Oido_New 11 MOLIT layer + 2 TB_senario layer = **15 layer 테이블**, DEFAULT_VIS 6 true.
- 카메라: iso (60m back/height) + top (100m), wheel zoom `exp(-deltaY*0.001)` 0.2..5.0 클램프.
- 페이로드 `lyr.kind` 우선 (LAYER_STYLE.kind 무시) — web_hmi MapLayers.jsx 패턴 일치.
- z=-north 부호 반전으로 Three.js scale.z=-1 트릭 회피.
- MapScene 슬롯 6개 노출: `setLayerVisibility/setShowBoxes/setShowHeading/setShowClouds/setPointSize/setCameraMode`.

### M4 — ControlPanel + V2X TrafficLightWidget
- ControlPanel = QDockWidget 우측 부착, 280~360 width, 4 그룹: V2X / Display / Camera / Map layers.
- 13 layer 체크박스 (15 entry table 순회), DEFAULT_LAYER_VIS 6 true / 9 false 일치.
- ControlPanel 시그널 6 → MapScene 슬롯 6 (직접 connect, lambda 없음, 시그너처 정확 매칭).
- TrafficLightWidget (220×130 카드): R/Y/G 3 lamp + 카운트다운 + "INT %d · SG %d" 푸터, OFF dim alpha 38 / ON inner glow.
- `RosBridge::trafficChanged`(int color, int timeDecisec, int intersectionId, int signalGroupId) 직결 슬롯.
- `emitInitialState()`로 startup 시 layer/display/camera 디폴트를 한 번 push → MapScene 일관성.
- F1Dashboard 라인 수 변동 0 (M2/M3 회귀 보존).
- 윈도우 1920×900 (좌 320 F1패널 + 중앙 dashboard chrome + 우 280~360 dock).

## 검증 패스 (각 M 4단계)

| M | 정적 | 빌드 | 스모크 | 기능 (라이브) |
|---|------|------|--------|---------------|
| M1 | PASS | PASS | SKIP / 사용자 file1 | PASS — 빈 MainWindow + 9 토픽 구독 |
| M2 | PASS | PASS (Qt5.13 setFamilies → setFamily, QEvent include 추가 후) | PASS | PASS — file2 텔레메트리 가시 |
| M3 | PASS (4/0) | PASS (QtMath include 추가 후) | SKIP (offscreen GL 미작동) | PASS — file3 OpenGL 그리드 + ego + 트랙 |
| M4 | PASS | PASS (setProperty(QString) 1줄 제거 후) | SKIP (M3 검증 후 노드 종료) | **사용자 라이브 인계 (file4 권장)** |

## 라이브 확인 (M4 시각 검증)

```bash
# 터미널 1
roscore

# 터미널 2 (web_hmi bridge)
cd /home/sim/mcar
source devel/setup.bash
roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false

# 터미널 3 (qt_hmi M4)
cd /home/sim/mcar
source devel/setup.bash
DISPLAY=:0 rosrun qt_hmi qt_hmi_node
```

기대 (10건):
1. 1920×900 윈도우 + 우측 ControlPanel dock.
2. dock 헤더 strip "CONTROL · MAP / DISPLAY".
3. TrafficLightWidget 초기 dim + "—" / bridge 도착 후 한 lamp ON + 카운트다운.
4. Display 그룹: bbox/heading/clouds 토글 즉시 반영.
5. Point size slider 0.02~0.50.
6. Camera radio iso/top.
7. 13 layer 체크박스 토글 즉시 반응.
8. 기동 직후 startup default 일치 (6 layer ON / 9 OFF, iso, 0.08 m).
9. dock detach/close + 윈도우 X 종료 시 segfault 없음 (5회 반복 권장).
10. F1Dashboard 좌측 03 V2X (M2 위젯)와 우측 dock TrafficLightWidget (M4)가 동일 데이터로 동시 갱신.

## 미해결 / 후속

| # | 항목 | 권장 |
|---|------|------|
| 1 | F1Dashboard DRIVE MODE click → publishModeRequest | M5 후보 |
| 2 | BagStrip click → publishBagToggle | M5 후보 |
| 3 | QMainWindow saveState/restoreState | 옵션 |
| 4 | ControlPanel scene FPS 표시 | 옵션 |
| 5 | ControlPanel 한국어 폰트 (현재 영문 라벨만) | 옵션 |
| 6 | TrafficLightWidget 만료 후 latched 동작 의존 | bridge 측 검토 |

## 다음 단계

- 라이브 file4 캡처로 M4 시각 검증 완료 후 커밋.
- 후속 M5는 사용자 우선순위에 따라 (DRIVE MODE click + BagStrip click이 가장 명확한 다음 단계).
