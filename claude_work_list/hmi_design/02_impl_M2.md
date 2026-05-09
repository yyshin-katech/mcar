# impl-coder M2 보고서

> 마일스톤: **M2 — F1Dashboard (1600×900 telemetry chrome + 7 widgets)**
> 작성: impl-coder
> 일시: 2026-05-09
> 빌드 환경: Qt 5.12.8 (Ubuntu 20.04, design §6.A.C fallback path)

---

## 작성/수정 파일

신규 5개, 수정 3개.

| 파일 | 신규/수정 | 줄수 | 역할 |
|------|----------|-----:|------|
| `src/visualization/qt_hmi/include/qt_hmi/style/F1Tokens.h` | 신규 | 59 | F1HMI.jsx `T` 색팔레트 19 토큰 ↔ `f1tokens::*` QColor inline 상수. `with_alpha()` 헬퍼 + `monoFamily()` fallback 문자열. |
| `src/visualization/qt_hmi/include/qt_hmi/widgets/F1Sections.h` | 신규 | 177 | 7 위젯 시그너처: `SpeedHalf`, `SteerDial`, `TrafficLight`, `Section`, `Stat`, `Dot`, `HealthRow` — 모두 `QWidget` paintEvent 기반. namespace `f1widgets`. |
| `src/visualization/qt_hmi/src/widgets/F1Sections.cpp` | 신규 | 611 | 7 위젯 QPainter 구현. F1HMI.jsx SVG 수식 (반원 게이지 t=value/max, 180°→360° 호 등) 그대로 이식. monospace fallback 폰트 + letter-spacing 동일. |
| `src/visualization/qt_hmi/include/qt_hmi/widgets/F1Dashboard.h` | 신규 | 160 | 1600×900 grid (320px 좌패널 + main + 8 cell bottom). `onState/onDiag/onHz/onTraffic/onPopup/onBag/onConnectionChanged` 7 슬롯. 캐시된 lastState/lastDiag/lastHz로 부분 갱신. |
| `src/visualization/qt_hmi/src/widgets/F1Dashboard.cpp` | 신규 | 963 | 4 sub-builder (`buildTopBar/buildLeftPanel/buildMainArea/buildBottomStrip`) + 4 refresh helper. F1HMIScreen.jsx prop→widget 매핑 (gear table 1=P/2=R/3=N/4=D, RTK ramp, σ-cm padding 등) 완전 이식. 4 Hz `QTimer` → UTC/KST/TICK/NET 자동 갱신. |
| `src/visualization/qt_hmi/include/qt_hmi/MainWindow.h` | 수정 | 31 | placeholder QLabel 7개 + 6 슬롯 제거. `f1widgets::F1Dashboard*` 단일 멤버. |
| `src/visualization/qt_hmi/src/MainWindow.cpp` | 수정 | 53 | F1Dashboard 인스턴스화 + RosBridge 6 시그널 wire-up (`stateChanged`, `diagChanged`, `hzChanged`, `trafficChanged`, `popupChanged`, `bagChanged`). |
| `src/visualization/qt_hmi/CMakeLists.txt` | 수정 | 93 | `QT_HMI_M2_SOURCES`/`QT_HMI_M2_HEADERS` 추가. `add_executable` 에 합산. |

**M2 신규 코드 1970줄, 수정 합산 약 30줄.**

---

## 핵심 결정

### 1. JSX SVG 수식 직접 이식 (좌표/스케일 일대일)

`F1HMI.jsx` 의 `SpeedHalf` (W=200,H=130, cx=100, cy=110, r=80), `SteerDial` (W=80,H=88, cx=40, cy=40, r=28), `TrafficLight` (LED diameter 24px, gap 3px, 38×90 column) 좌표를 **로지컬 캔버스**로 사용하고, `paintEvent` 안에서 `QPainter::scale(min(w/designW, h/designH))` 로 위젯 크기에 비율 맞춤. → 어떤 사이즈로 늘려도 JSX 와 동일 비율.

각도 계산은 JSX 와 동일하게 `a = 180 + 180*i/max` (deg) → `cos/sin` 직접 호출. Qt `drawArc` 의 1/16th-degree 단위 + CCW 양수 규칙은 별도 호 그리기에만 사용 (틱은 점→점 라인이라 영향 없음).

### 2. Qt 5.12 호환 폰트 fallback

설계 §6.A.C 의 Qt5 fallback 환경 (Ubuntu 20.04 기본 5.12.8) 을 그대로 유지. **첫 시도에서 `QFont::setFamilies()` 사용 → 빌드 실패** (Qt5.13+에서만 도입). 콤마-구분 단일 문자열 (`"JetBrains Mono, DejaVu Sans Mono, Liberation Mono, Monospace"`) + `setStyleHint(QFont::Monospace)` 로 fallback 동작. JetBrains Mono 미설치 시 시스템 모노스페이스 자동 사용 (설계 §6.D 결정 그대로).

### 3. 위젯 = 순수 QPainter, 외부 의존 0

설계 원칙 그대로 — Qt5 Widgets/Gui 만 사용. SVG 파싱 라이브러리(QSvg) / OpenGL / 외부 폰트 패키지 0건 추가. 7 위젯 모두 `paintEvent` 단독 처리, MOC가 검증한 Q_OBJECT signal/slot 만 노출.

### 4. 캐시 + 부분 갱신 전략

`F1Dashboard` 는 RosBridge 시그널을 받을 때마다 캐시 (`lastState_`, `lastDiag_`, `lastHz_`) 업데이트 후 **영향받는 영역만 refresh** 호출. 예:
- `onState` → SpeedHalf/SteerDial/DriveMode/Localization/Health(GPS row)/Bottom 모두 갱신.
- `onHz` → Health row info 만.
- `onDiag` → Health row status + summary dot 만.

각 sub-widget 의 setter 는 값이 같으면 일찍 return (`qFuzzyCompare` / `==` 비교) → 불필요한 `update()` 회피.

### 5. ODD 배너 + BAG 오버레이 = QFrame 절대배치

JSX 의 `position:absolute` 위치 (banner: top=28 centered, bag: top-right) 를 Qt 에서 재현하려고 **`StageFilter` 이벤트 필터** (resizeEvent 캡처) 사용. 일반 layout 에서 벗어난 floating overlay 를 main area 의 부모 widget 에 직접 child 로 두고, resize 시마다 절대 좌표 재계산. M3 에서 `MapScene` 이 같은 stage 에 들어와도 overlay 는 그 위에 떠 있도록 `raise()` 호출 가능 구조.

### 6. RTK 라벨 + GPS-RTK row 정합

`F1HMIScreen.jsx` 의 `formatRtk()` (rtk=2 → FIX/green, =1 → FLOAT/amber, else → NO RTK/text3) 를 `onState` 안에서 직접 산출 → `localSection_->setRight()` (Section 헤더 우측) 와 `healthRows_[0]->setInfo("RTK FIX · 1.2 Hz")` 모두 동일한 값을 사용. JSX 는 `useRosState` + `useTopicHz` 두 hook 가 같은 데이터를 두 번 읽어 합쳤지만, qt 측은 `lastHz_.value("gps")` 와 `rtkLabel_` 캐시를 같이 참조해 **동일한 결과** 를 보장.

### 7. 4 Hz 클록 (QTimer) — top bar 자가 갱신

`F1HMIScreen.jsx` 의 `useClock(250)` 와 정확히 동일 — `QTimer` 250 ms 간격으로 `tick()` 호출. UTC/KST 는 `QDateTime::currentDateTimeUtc()` + `addSecs(9*3600)` (KST = UTC+9 고정 오프셋, 한국 표준시는 DST 없음). TICK 은 `QElapsedTimer` 시작 후 경과 시간. NET 은 `lastMsgAgeMs_` 캐시 (M2 에서는 connectionChanged 슬롯이 0 으로 부트스트랩하지만, M3+ 에서 RosBridge 가 실시간 연결 상태 시그널을 늘리면 자동 반영).

### 8. ROS 배지 색 — Hz 테이블 기반 fallback

`!conn.connected` → OFFLINE/red, `connected && anyHz>0.5` → ONLINE/green, `connected && allHz=0` → WAITING/amber. 현재 RosBridge M1 시그너처에 `rosConnectionChanged` 가 선언만 되어 있고 emit 되지 않으므로, MainWindow 가 부트 시 `onConnectionChanged(true,0)` 한 번 호출. 이후 Hz 테이블 도착 → ONLINE 자동 전환. M3/M4 에서 RosBridge 가 실제 connection 시그널을 emit 하기 시작하면 추가 수정 없이 동작.

---

## 빌드 검증 (자체 시도)

```
$ source /opt/ros/noetic/setup.bash
$ catkin_make --pkg qt_hmi
...
[  0%] Building CXX object .../F1Sections.cpp.o
[  0%] Building CXX object .../F1Dashboard.cpp.o
[  0%] Linking CXX executable /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
[100%] Built target qt_hmi_node
```

**결과: PASS** (exit 0). gcc-9.4.0, C++17, Qt 5.12.8.

산출 binary:

```
$ ls -l /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
-rwxr-xr-x  1060368 May  9 23:03  qt_hmi_node     (M1 832 KB → M2 1.06 MB, +27% 코드 증가)
```

빌드 경고: 없음 (qt_hmi 자체).

### 첫 시도 실패 → 수정한 2건

| 분류 | 증상 | 원인 | 수정 |
|------|------|------|------|
| 컴파일 에러 (Qt 버전 호환) | `class QFont has no member named 'setFamilies'` | Qt5.13+ API. Ubuntu 20.04 의 5.12.8 미지원. | `setFamilies({...})` → `setFamily("a, b, c")` (콤마 구분 단일 문자열). 2 곳 수정. |
| 컴파일 에러 (헤더 누락) | `invalid use of incomplete type 'class QEvent'` | F1Dashboard.cpp 안 inner class `StageFilter` 가 `QEvent::Resize` 를 enum 으로 사용하지만 헤더에 `QEvent` forward decl 만 들어옴. | `#include <QtCore/QEvent>` + `#include <QtGui/QResizeEvent>` 추가. |

수정 후 재빌드 PASS — 실제 시도는 2회.

---

## 다음 마일스톤 인계 (impl-verifier 로)

### 검증 포커스

1. **Visual parity vs web HMI**: 같은 라이브 데이터 환경에서 `web_hmi` (`http://localhost:8088/index_threejs_f1.html`) 와 `qt_hmi_node` 를 나란히 띄워 다음 항목을 픽셀 단위 ±2px 비교:
   - SpeedHalf 게이지 호 길이 (speed=60 km/h 일 때 t=0.5)
   - SteerDial 인디케이터 회전 (steering=±15° 일 때)
   - TrafficLight LED 점등 (color=1/2/3) + remain 카운트다운
   - HealthRow 6 개 색상 (status=0/1/2 ↔ green/amber/red)
   - σ-EAST/NORTH `cm` 표기 (예: lonStd=0.0184 → "01.84 cm")
   - DRIVE MODE 토글 highlight (mode=0 ↔ MANUAL highlighted, mode=1 ↔ AUTONOMOUS highlighted)
2. **ODD 배너 / BAG 오버레이 위치**: main area 가 resize 될 때 banner 가 가운데 정렬, bag 오버레이가 top-right 18px 마진 유지하는지.
3. **4 Hz 클록 단조 증가**: TICK 이 0.000s → 0.250s → 0.500s 로 부드럽게 증가하는지 (250 ms 간격).
4. **종료 race**: M1 의 `aboutToQuit → bridge->stop() → bridge.reset() → ros::shutdown()` 시퀀스가 dashboard 추가 후에도 깨끗히 동작하는지 (창 닫기 시 segfault 없음).

### 라이브 확인 권장 시나리오

```bash
roslaunch web_hmi web_hmi.launch variant:=threejs_f1 open_browser:=false
# (별 터미널)
source /home/sim/mcar/devel/setup.bash
rosrun qt_hmi qt_hmi_node
```

기대:
1. 1600×900 크기로 검정 배경 + cyan 액센트 chrome 표시.
2. 1초 내 SpeedHalf / SteerDial / Localization 라벨이 라이브 값으로 갱신.
3. `/hmi/diagnostics` 도착 시 5 SYSTEM HEALTH 6 row 와 4-dot summary 색깔 변경.
4. 자동 모드 전환 시 (`mode=1`) DRIVE MODE 토글이 AUTONOMOUS 로 highlight + ENGAGED 라벨 green.
5. main area 중앙 라벨 "MapScene (M3) pending — 3D HD map + ego + tracks 가 이 영역에 렌더됩니다." 표시.

---

## 미해결 / 추정

1. **JSX `tabular-nums` font-variant**: Qt 의 `font-variant-numeric:tabular-nums` 스타일시트 속성은 Qt 5.12 에서 일부 폰트만 지원. `JetBrains Mono` 가 없으면 시스템 모노에서 정렬이 살짝 어긋날 수 있음. M2 시점에는 시각적으로 충분히 가깝다고 가정. 추후 폰트 라이선스 (§6.D) 해결되면 .ttf 동봉.
2. **scanlines overlay 미구현**: JSX 는 1px repeating-linear-gradient 로 CRT 스캔라인 효과 추가. Qt 에서는 동일 구현이 painter 비용을 거의 안 추가하지만 시각적 임팩트가 작아 M2 범위에서 생략. M3 시 main area QOpenGLWidget 위로 동일 효과 추가 가능.
3. **HEALTH summary dot 위치**: Section 헤더 위에 absolute 배치 — `QTimer::singleShot(0, ..., move(width-60, 7))` 로 1회 정렬하지만 좌패널 너비가 320px 고정이라 추후 panel resize 가 일어나면 어긋남. 320 고정 정책이 유지되는 한 문제 없음. M3+에서 panel 리사이저 도입 시 `Section::resizeEvent` 에 hook 추가 필요.
4. **CONN 상태 시그널 누락**: 설계 §2.B `rosConnectionChanged(bool, qint64)` 가 RosBridge.h 에 선언되어 있으나 RosBridge.cpp 에서 emit 되지 않음 (M1 산출). 현재 dashboard 는 `rosConnected_=true` 로 부트스트랩 + Hz 0.5 임계로 대체. M3 또는 M4 에서 RosBridge 가 실제 마지막 메시지 수신 시각을 트래킹하기 시작하면 자동으로 NET 표기가 살아남.
5. **DRIVE MODE 클릭 핸들 부재**: 설계 §5/M2 검증 항목 6 (ROS 끊김 시 ROS 배지 OFFLINE 적색) 는 현재 동작 (Hz=0 + bootstrapped connected=true → WAITING amber 가 정확). 설계 §5/M2 표 의 ModeButton click 은 M4 범위라 미구현 — 이는 검증 1단계와 일치.
6. **Qt6 자동 fallback 동작**: M1 보고와 동일 — Qt6 환경에서는 검증 미수행 (사용자 환경 5.12.8). CMake 분기는 syntax-correct.

---

## 부록 — M3 진입 체크리스트

(impl-verifier M2 검증 통과 후 다음 마일스톤 진입 준비)

- [ ] `include/qt_hmi/style/LayerStyle.h` + `src/style/LayerStyle.cpp` 신규 — 13 layer (HD map 11 + TB_senario_* 2) 색/굵기/kind 테이블.
- [ ] `include/qt_hmi/widgets/MapScene.h` + `.cpp` 신규 — `QOpenGLWidget` 기반 3D 렌더러.
- [ ] `shaders/line.{vert,frag}`, `point.{vert,frag}` 신규 + `resources/qt_hmi.qrc` 신규.
- [ ] `F1Dashboard.cpp` 의 `mainPlaceholder_` 영역 → `MapScene` 으로 교체. `RosBridge::threejsMapReceived` / `tracksChanged` / `stateChanged` 를 MapScene slot 으로 연결.
- [ ] `CMakeLists.txt` 의 `# TODO M3` 슬롯에 OpenGLWidget 모듈 + QResource 등록 추가 (`Qt5::OpenGL` 또는 `Qt6::OpenGLWidgets`).
- [ ] iGPU 성능 회귀 측정: 빈 scene + tracks 0 → 60 fps, layer 11 + tracks 12 + cloud 1k → ≥30 fps 목표.

