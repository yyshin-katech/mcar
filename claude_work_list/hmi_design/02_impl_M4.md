# impl-coder M4 보고서

> 마일스톤: **M4 — ControlPanel + V2X TrafficLightWidget**
> 작성: impl-coder
> 일시: 2026-05-10
> 빌드 환경: Qt 5.12.8 + Qt5::OpenGL (Ubuntu 20.04). qt_hmi_node 1.40 MB.

---

## 작성/수정 파일

신규 4개, 수정 4개.

| 파일 | 신규/수정 | 줄수 | 역할 |
|------|----------|-----:|------|
| `src/visualization/qt_hmi/include/qt_hmi/widgets/ControlPanel.h` | 신규 | 97 | 우측 dock — V2X 카드 + Display(bbox/heading/clouds + point size slider) + Camera(iso/top radio) + Map layers(13 checkbox). 6개 시그널(`layerVisibilityChanged/showBoxesChanged/showHeadingChanged/showCloudsChanged/pointSizeChanged/cameraModeChanged`) + `trafficLight()` accessor + `emitInitialState()`. |
| `src/visualization/qt_hmi/src/widgets/ControlPanel.cpp` | 신규 | 413 | 4 섹션 빌더(`buildTrafficSection/buildDisplayGroup/buildCameraGroup/buildLayersGroup`) + 슬롯(`onLayerToggled/onPointSliderMoved/onCameraIso/onCameraTop`) + `emitInitialState`. F1Tokens 색상으로 QGroupBox/QCheckBox/QRadioButton/QSlider 스타일링, 9×9 swatch QPixmap 동적 생성, QScrollArea로 720px 환경 호환. |
| `src/visualization/qt_hmi/include/qt_hmi/widgets/TrafficLightWidget.h` | 신규 | 58 | RosBridge::trafficChanged 시그너처(`color, timeDecisec, intersectionId, signalGroupId`) 직결 슬롯 `setTraffic`. Phase enum {OFF/GREEN/AMBER/RED}. paintEvent 카드 220×130. |
| `src/visualization/qt_hmi/src/widgets/TrafficLightWidget.cpp` | 신규 | 219 | QPainter R/Y/G 3 lamp(수직 stack, 동적 사이즈 lampD) + 카운트다운 숫자(픽셀 34px) + "CHANGE IN" 라벨 + "INT %1 · SG %2" 푸터. OFF면 모두 dim alpha 38, ON 단계는 inner highlight glow. 헤더 strip "V2X · TRAFFIC" + 우측 phase tag. |
| `src/visualization/qt_hmi/CMakeLists.txt` | 수정 | 122 | M4 source/header 변수 추가(`QT_HMI_M4_SOURCES/HEADERS`) + `add_executable`에 합산. M3 +13줄. |
| `src/visualization/qt_hmi/include/qt_hmi/MainWindow.h` | 수정 | 33 | `qt_hmi_widgets::ControlPanel` forward decl + `controlPanel_` 멤버 추가. 헤더 코멘트 M4 기준 갱신. |
| `src/visualization/qt_hmi/src/MainWindow.cpp` | 수정 | 121 | ControlPanel 인스턴스 생성 + QDockWidget 우측 부착 + 6 시그널/슬롯 연결(ControlPanel→MapScene) + RosBridge::trafficChanged→TrafficLightWidget 연결 + `emitInitialState()` 호출 + 윈도우 1920×900 + 타이틀 M4. |

**M4 신규 코드 787줄, 수정 합산 약 60줄.**

---

## 핵심 결정

### 1. ControlPanel은 QDockWidget 우측에 부착

설계 §1.C "ControlPanel : QWidget … 사이드 도크"를 그대로. MainWindow가 `addDockWidget(Qt::RightDockWidgetArea, dock)` 호출. 사용자가 dock title을 잡고 떼낼 수 있도록 `DockWidgetMovable | DockWidgetFloatable` 활성, 닫기는 막아 `DockWidgetClosable` 미포함(F1Dashboard와 같이 항상 보이는 게 의도).

좌측 320 패널과 충돌 없도록 ControlPanel `setMinimumWidth(280)` + `setMaximumWidth(360)`. 윈도우 디폴트 1920×900으로 늘려 1600 dashboard + 320 dock가 startup에 모두 보이도록.

### 2. TrafficLightWidget은 ControlPanel 상단(card)에 단독 배치

태스크 사양 "F1Dashboard 상단 stat row 옆 또는 ControlPanel 상단" 중 후자 선택. 이유:
- F1Dashboard 좌측 03 V2X 영역에 이미 `f1widgets::TrafficLight`(M2)가 같은 데이터를 표시 — 회귀 보존이 우선.
- ControlPanel 상단 card로 두면 사용자가 layer 토글하면서 신호 상태를 한 시야에 본다.
- TrafficLightWidget 자체가 카드 헤더 strip("V2X · TRAFFIC")을 직접 paint — ControlPanel은 단순히 호스팅(`buildTrafficSection`).

`f1widgets::TrafficLight`(M2)와 `qt_hmi_widgets::TrafficLightWidget`(M4)는 별개 클래스 — 헤더/네임스페이스 분리. M2 위젯은 좌측 패널 그대로, M4 위젯은 우측 dock 상단. 동일한 RosBridge::trafficChanged 시그널이 두 곳 모두 동시 갱신(F1Dashboard.onTraffic + TrafficLightWidget.setTraffic).

### 3. 13 layer 체크박스 — DEFAULT_LAYER_VIS 정확 일치

ControlPanel `buildLayersGroup`이 `qt_hmi_style::layerStyleTable()` 15 entry를 순회하며 각 layer의 swatch + label + checkbox를 만듦. 초기값은 `qt_hmi_style::defaultLayerVis()`에서 직접 가져오므로 6 layer만 true(A2_LINK / A3_DRIVEWAYSECTION / B2_SURFACELINEMARK / B3_SURFACEMARK / TB_senario_map / TB_senario_surfaceMARK) — 초기 상태 시 web_hmi와 동일.

`emitInitialState()`는 이 snapshot을 `layerVisibilityChanged(QHash<QString,bool>)` 시그널로 푸시 → MapScene::setLayerVisibility 호출 → MapScene 내부 `layerVis_` 갱신. (MapScene 자체도 같은 default를 갖고 있으므로 결과는 동일하지만, 신호 경로를 바로 검증하기 위해 startup에 한 번 emit.)

### 4. point size slider 매핑 0.02 ~ 0.50 m (×0.01 step)

QSlider integer range 2..50. 슬롯에서 `value * 0.01f`로 float 변환 → MapScene::setPointSize. 라벨도 `0.02 ~ 0.50` 표시. 디폴트 8 = 0.08 m로 MapScene 기본값과 일치(M3 02_impl §결정 4).

### 5. camera mode QButtonGroup exclusive — emit on toggle(true)만

QRadioButton::toggled는 모든 버튼이 한 번씩 emit하므로, `if (checked) emit cameraModeChanged(...)` 가드로 한 번만 발사. iso 디폴트 → `camIso_->setChecked(true)` startup. emitInitialState에서도 현재 체크된 라디오 기준으로 `"iso"` or `"top"` 단발 emit.

### 6. ControlPanel ↔ MapScene 시그널 ↔ 슬롯 시그너처 정확 매칭

| ControlPanel signal | MapScene slot | 비고 |
|---------------------|---------------|------|
| `layerVisibilityChanged(const QHash<QString,bool>&)` | `setLayerVisibility(const QHash<QString,bool>&)` | snapshot — diff 미사용 |
| `showBoxesChanged(bool)` | `setShowBoxes(bool)` | |
| `showHeadingChanged(bool)` | `setShowHeading(bool)` | |
| `showCloudsChanged(bool)` | `setShowClouds(bool)` | |
| `pointSizeChanged(float)` | `setPointSize(float)` | |
| `cameraModeChanged(const QString&)` | `setCameraMode(const QString&)` | "iso" or "top" |

모든 슬롯이 M3에서 이미 노출된 그대로(`MapScene.h:51~57`). M4는 connect 6건만 추가 — MapScene 코드 0줄 수정.

### 7. RosBridge::trafficChanged → TrafficLightWidget::setTraffic 직결

bridge 시그널 시그너처(`int color, int timeDecisec, int intersectionId, int signalGroupId`)가 setTraffic 슬롯과 정확 일치 — Qt5 lambda 없이 직접 connect 가능. 두 번째 인자 timeDecisec(decisec=10ms)는 setTraffic 내부에서 `(timeDecisec + 5) / 10`로 round → seconds 변환. f1widgets::TrafficLight도 동일 페이로드를 받으므로 두 위젯이 같은 페이즈/카운트다운을 표시.

### 8. setProperty 우회 — Qt5 QString→QVariant 변환

첫 빌드 시 `cb->setProperty("layer", name)` 에서 `QString` → `const QVariant&` 묵시적 변환 실패(Qt5의 setProperty 시그너처가 explicit). `layerCheckboxes_` QHash가 이미 `QString → QCheckBox*` 매핑을 들고 있어 property 자체가 불필요 — slot에서 hash 순회로 snapshot 생성. property 호출 1줄 제거.

### 9. F1Dashboard 미수정 (외과적 구현 원칙)

설계 §5/M4 "기본값은 DEFAULT_LAYER_VIS (Three.js와 동일)" 외, F1Dashboard 본체는 손대지 않음. 03 V2X의 `f1widgets::TrafficLight`(M2 회귀)는 그대로 동작 — MainWindow.cpp의 `connect(bridge_, &RosBridge::trafficChanged, dashboard_, &f1widgets::F1Dashboard::onTraffic)`(M2)는 보존되며, M4에서 추가된 connect는 별개 슬롯 호출이라 emit 1회로 두 위젯 모두 갱신.

DRIVE MODE click 핸들러(mode_request publish)와 BAG strip click(bag_toggle publish)는 task 사양에 포함되지 않았으므로 M4 범위 외. 이는 추후 M5/별도 마일스톤이나 사용자 후속 요청에서 결정.

---

## 빌드 검증 (자체 시도)

```
$ source /opt/ros/noetic/setup.bash
$ catkin_make --pkg qt_hmi 2>&1 | tail
[  0%] Automatic MOC and UIC for target qt_hmi_node
[  0%] Built target qt_hmi_node_autogen
Scanning dependencies of target qt_hmi_node
[  0%] Building CXX object .../src/widgets/ControlPanel.cpp.o
[  0%] Building CXX object .../src/widgets/TrafficLightWidget.cpp.o
[  0%] Building CXX object .../src/MainWindow.cpp.o
[  0%] Building CXX object .../qt_hmi_node_autogen/mocs_compilation.cpp.o
[100%] Linking CXX executable /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
[100%] Built target qt_hmi_node
```

**결과: PASS** (exit 0). gcc-9.4.0, C++17, Qt 5.12.8, Qt5::OpenGL.

산출 binary:

```
$ ls -l /home/sim/mcar/devel/lib/qt_hmi/qt_hmi_node
-rwxr-xr-x  1400744  May 10 00:02  qt_hmi_node     (M3 1.21 MB → M4 1.40 MB, +15%)
```

신규 MOC 산출 2건:
```
build/visualization/qt_hmi/qt_hmi_node_autogen/TB6PJGQYWE/moc_ControlPanel.cpp        ← M4 신규
build/visualization/qt_hmi/qt_hmi_node_autogen/TB6PJGQYWE/moc_TrafficLightWidget.cpp  ← M4 신규
```

빌드 경고: 0건 (qt_hmi 자체).

### 첫 시도 실패 → 수정한 1건

| 분류 | 증상 | 원인 | 수정 |
|------|------|------|------|
| 컴파일 에러 (Qt5 변환) | `error: cannot convert 'QString' to 'const QVariant&'` (ControlPanel.cpp:337) | Qt5의 `QObject::setProperty(const char*, const QVariant&)` 시그너처가 QString 묵시적 변환을 허용 안 함 | `cb->setProperty("layer", name)` 1줄 제거 — `layerCheckboxes_` QHash로 이미 검색 가능. |

수정 후 재빌드 PASS — 실제 시도는 2회.

---

## 다음 마일스톤 인계 (impl-verifier로)

### 검증 포커스

1. **정적 — 신규 4 / 수정 4 파일 인벤토리**: 02_impl_M4.md 라인 수와 실측 일치(±0). Q_OBJECT 매크로 2건 신규(ControlPanel/TrafficLightWidget) — 총 13건.
2. **정적 — Q_OBJECT 매크로 2건 신규**: ControlPanel.h:42, TrafficLightWidget.h:25.
3. **정적 — 시그널/슬롯 매칭 6건 + 1건**:
   - `ControlPanel::layerVisibilityChanged ↔ MapScene::setLayerVisibility`
   - `ControlPanel::showBoxesChanged       ↔ MapScene::setShowBoxes`
   - `ControlPanel::showHeadingChanged     ↔ MapScene::setShowHeading`
   - `ControlPanel::showCloudsChanged      ↔ MapScene::setShowClouds`
   - `ControlPanel::pointSizeChanged       ↔ MapScene::setPointSize`
   - `ControlPanel::cameraModeChanged      ↔ MapScene::setCameraMode`
   - `RosBridge::trafficChanged             ↔ TrafficLightWidget::setTraffic`
4. **정적 — DEFAULT_LAYER_VIS 매칭**: ControlPanel buildLayersGroup의 13 layer 초기 체크 상태가 `qt_hmi_style::defaultLayerVis()` 6 true / 9 false와 일치.
5. **정적 — Qt5 호환**: `QFont::setFamily(QString)` 단일(setFamilies 미사용), `setProperty(QString)` 미사용, lambda 없이 직접 connect.
6. **정적 — F1Dashboard 미수정 (M2/M3 회귀 보존)**: F1Dashboard.h/.cpp/F1Sections.h/.cpp 라인 수 변동 0 (M3 보고서 라인 수 그대로).
7. **catkin 빌드 재현**: `catkin_make --pkg qt_hmi` exit 0, qt_hmi_node 1.40 MB ELF, MOC 7건(M1 2 + M2 2 + M3 1 + M4 2 = 7).
8. **라이브 시각 확인 (DISPLAY 환경)**:
   - 1920×900 윈도우 + 우측에 ControlPanel dock 표시
   - 상단 V2X 카드: bridge 시그널 도착 시 R/Y/G lamp 한 개만 ON, 카운트다운 숫자 갱신
   - Display 그룹: bbox/heading/clouds 토글 시 MapScene render 즉시 반영
   - point size slider 0.02..0.50 이동 시 MapScene 점 크기 변화
   - Camera radio iso/top 전환 시 즉시 카메라 모드 변경
   - 13 layer 체크박스 토글 시 해당 layer geometry 표시/숨김
9. **종료 race**: dock detach/close + 윈도우 X 종료 시 segfault 없음 — ControlPanel/TrafficLightWidget은 child widget, dock도 setParent로 메인윈도우 소유.

### 라이브 확인 권장 시나리오

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
rosrun qt_hmi qt_hmi_node
```

기대:
1. 1920×900 윈도우 + 좌 320 F1 패널 + 중앙 1280 main(MapScene + dashboard chrome) + 우 ~320 ControlPanel dock.
2. ControlPanel 헤더 strip "CONTROL · MAP / DISPLAY" + V2X 카드 + Display + Camera + Map layers 4 그룹.
3. TrafficLightWidget: 시그널 미도착 시 모든 lamp dim, "—" 카운트다운; 도착 후 한 lamp 강조 + 숫자.
4. 체크박스 토글 시 MapScene 즉시 반응.

---

## 미해결 / 추정

1. **DRIVE MODE click handler 미구현 (mode_request publish)**: 설계 §5/M4 "DRIVE MODE 클릭 → /hmi/cmd/mode_request publish" 항목은 task 사양 외라 미수행. 추후 별도 작업 필요. RosBridge::publishModeRequest 슬롯은 이미 존재(M1).
2. **BagStrip click handler 미구현 (bag_toggle publish)**: 설계 §5/M4 "bag overlay click → /hmi/cmd/bag_toggle publish" 항목 동일 — task 사양 외. bagOverlay_는 F1Dashboard에 이미 시각 표시만 존재.
3. **dockable layout 저장/복원 없음**: QMainWindow::saveState/restoreState 호출 미연결. dock detach 위치는 매 실행마다 default(우측 부착)에서 시작.
4. **ControlPanel 폰트가 한국어 미포함**: 'JetBrains Mono','DejaVu Sans Mono' 우선이라 한글이 fallback 폰트로 깨질 수 있음. 현재 ControlPanel 라벨이 모두 영문이라 영향 없음.
5. **TrafficLightWidget 카운트다운 0초/만료 처리**: timeDecisec ≤ 0 또는 phase OFF면 "—" 표시. /siheung_spat이 카운트다운 만료 후 새 페이즈를 즉시 보내지 않으면 잠깐 잘못된 숫자가 보일 수 있음 — bridge 측 latched 동작에 의존.
6. **ControlPanel emitInitialState — MapScene::initializeGL 이전 호출**: MainWindow 생성자 끝에서 호출되며 MapScene::initializeGL은 첫 paintGL 직전. setLayerVisibility 등은 단순 멤버 갱신 + update() 호출이라 OpenGL 컨텍스트 미필요 — 시퀀스 안전. (단, 추후 setShowClouds가 GL 자원을 직접 건드리는 형태로 변경되면 재검토 필요.)

---

## 부록 — 다음 단계 체크리스트

(impl-verifier M4 검증 후 시각 확인 + 후속 작업)

- [ ] 라이브 환경(DISPLAY + roscore + web_hmi) 시각 검증 6건(§검증 포커스 #8).
- [ ] dock detach/close 종료 race — 5회 반복 PASS.
- [ ] 후속(별도 마일스톤): F1Dashboard DRIVE MODE 토글 click → publishModeRequest, bagOverlay click → publishBagToggle.
- [ ] 후속(옵션): MainWindow saveState/restoreState로 dock 위치 영구화.
- [ ] 후속(옵션): ControlPanel 헤더 strip에 "scene FPS" 텍스트 추가 (M3 §6.G 성능 모니터).
