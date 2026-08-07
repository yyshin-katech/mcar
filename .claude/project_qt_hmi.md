---
name: qt_hmi + web_hmi camera follow rviz-grade smoothness
description: Qt5 qt_hmi (M1~M4 PASS) + web_hmi 모두 /localization/to_control_team 50 Hz 직접 구독 + EMA α=0.5 패턴. 추가로 ego-frame 트랙은 emit-time ego 스냅샷(`ego_at_emit`)과 페어링해야 슬라이드 안 함.
type: project
originSessionId: 2732ef96-e301-44d6-abdd-0935ca63a907
---
`src/visualization/qt_hmi/` — web_hmi(Three.js + 브라우저) 동등 기능을 Qt5/C++로 재구현한 네이티브 패키지. siheung_dev 브랜치 (2026-05-09 시작, 2026-05-10 M4 + 50 Hz patch 완료).

**기술 스택:** Qt5.12.8 (Ubuntu 20.04 기본). Qt6 fallback 분기는 CMakeLists에 남겨둠. C++17 / catkin / roscpp / std_msgs / **mmc_msgs**. Qt5::OpenGL (M3 추가).

**Why Qt5 (not Qt6):** Ubuntu 20.04 기본 저장소에 `qt6-base-dev` 없음. 차량 PC NUC에도 별도 설치 부담 → Qt5 채택.

**아키텍처:**
- `RosBridge(QObject)` + `ros::AsyncSpinner(2)` + `Qt::QueuedConnection`로 GUI 스레드 안전.
- 데이터 소스: `web_hmi_threejs_bridge.py` 발행 `/hmi/*` JSON 토픽 (QJsonDocument 파싱). **단 ego pose만 예외** — 50 Hz raw `/localization/to_control_team`(`mmc_msgs::to_control_team_from_local_msg`) 직접 구독 → `egoPoseChanged(east,north,yaw)` 시그널.
- 종료 race 회피: `unique_ptr<RosBridge>` + `QApplication::aboutToQuit`에서 spinner.stop() → reset() → ros::shutdown().

**카메라 follow rviz-grade smoothness (2026-05-10 추가):**
- 문제: `web_hmi_bridge`의 `_publish_periodic`이 `/hmi/state`를 10 Hz로 throttle. EMA α=0.18로 보간해도 시정수 80 ms로 lag 가시. rviz는 ~50 Hz TF + timestamp 보간이라 더 부드러움.
- 해결: ego pose 한정으로 raw `/localization/to_control_team`(50 Hz, `host_east/north/yaw`)을 RosBridge에서 별도 구독. `MapScene::onEgoPoseChanged`가 `lastState_.ego.*` 직접 갱신. `onStateChanged`는 ego 필드 보존 (10 Hz가 50 Hz를 덮어쓰지 않음). EMA α: 0.18 → **0.5** (50 Hz × α=0.5 ≈ 32 ms 시정수, 거의 raw 추종).
- yaw unwrap: `atan2(sin Δ, cos Δ)`로 +π↔-π wrap 시 카메라 spin 방지.
- 60 fps 갱신: MapScene 생성자에 `QTimer::start(16)` → `QWidget::update()` (ThreeScene.jsx requestAnimationFrame 미러).

**Bag 검증 시 publisher 충돌 회피:** bag 파일에 `/hmi/state`가 녹화되어 있으면 live `web_hmi_bridge`까지 띄우면 dual-publisher → 토픽 값 진동. **roscore + bag + qt_hmi_node**만 띄우는 것이 정답 (web_hmi.launch 생략). bag 자체에 IPC 센서 깜빡임 같은 결함이 있을 수 있음 (해당 bag의 BaseHmiStateController 진동 — qt_hmi 버그 아님).

**하네스:** `.claude/skills/qt-hmi-build/` + `.claude/agents/{design-architect,impl-coder,impl-verifier}.md`. web-hmi-adapt와 동일 3-phase. 차이점: 마일스톤 단위 호출 (1 호출 1 M), 산출물 `claude_work_list/hmi_design/`.

**마일스톤 결과 (모두 PASS):**
- **M1** — 스켈레톤 + RosBridge 9 토픽 + MainWindow placeholder. 832 KB ELF.
- **M2** — F1Dashboard 7 위젯 (SpeedHalf/SteerDial/TrafficLight/Section/Stat/Dot/HealthRow), QPainter paintEvent로 F1HMI.jsx 1600×900 이식. 1.06 MB ELF.
- **M3** — MapScene QOpenGLWidget + Core 3.3 + 4 inline GLSL. HDMap_Oido_New 11 MOLIT + 2 TB_senario = 15 layer table, DEFAULT_VIS 6 true. iso(60m)/top(100m) 카메라 wheel zoom. payload `lyr.kind` 우선 (LAYER_STYLE.kind 무시 — MapLayers.jsx 패턴). z=-north 부호반전으로 Three.js scale.z=-1 회피. 1.21 MB ELF.
- **M4** — ControlPanel QDockWidget(280~360px, 4 그룹 V2X/Display/Camera/Layers, 13 체크박스) + V2X TrafficLightWidget (220×130, R/Y/G + 카운트다운 + INT/SG 푸터). `controlPanel_->emitInitialState()`로 startup 일관성. 1.40 MB ELF, MOC 7건.

**JSON 파서 함정:** `/hmi/threejs/map`의 layer kind는 페이로드 측 `layer.kind`(point/polyline/polygon) 우선. `LAYER_STYLE`(types.js)의 `kind`는 무시 (web_hmi MapLayers.jsx와 동일 정책). qt_hmi의 `LayerStyle.h`도 동일.

**web_hmi 50 Hz 동등화 (2026-05-10, commit 47d72af):**
- `web_hmi_bridge.py`: `BaseHmiStateController._cb_local`이 이미 50 Hz로 emit하던 `'ego_pose_changed'` 이벤트를 `/hmi/ego_pose` (std_msgs/String JSON) 신규 토픽으로 라우팅. `/hmi/state` 10 Hz 스냅샷은 그대로.
- 프론트: `useEgoPose()` 훅 추가 → `CameraController` / `EgoMesh`가 `useRosState` 대신 사용. `useEffect`가 토픽 갱신마다 재실행되므로 50 Hz 추종.

**Ego-frame 트랙 슬라이드 함정 (2026-05-10, commit 후속):**
- 증상: 카메라가 50 Hz로 부드럽게 따라가는데 객체(트랙)들이 매 perception tick(~10 Hz)마다 ~1 m씩 튐.
- 원인: `web_hmi_threejs_bridge._on_percept`는 `coreinfo.center.x/y` (ego-frame) 좌표를 그대로 emit. `TrackBoxes`가 `trackGroup`을 라이브 50 Hz ego로 변환하면, 트랙이 emit된 시점(P_emit)과 현재 ego(P_now) 차이만큼 월드 위치가 어긋남. 다음 tick에 새 트랙이 P_new 기준으로 들어오면 그만큼 스냅 → 가시적 jitter.
- 해결: 브리지가 `_on_local`에서 `/localization/to_control_team` 직접 구독 → `_last_ego` 캐시 → `_on_percept` 페이로드에 `ego_at_emit:{east,north,yaw}` 동봉. `TrackBoxes`는 `useEgoPose()` 제거, `tracks.ego_at_emit`로 trackGroup 변환 (트랙과 변환이 동시간 페어링).
- 일반 패턴: ego-frame으로 발행되는 모든 *느린* 데이터(perception, free space, 등)는 emit-time ego 스냅샷과 페어링되어야 함. 라이브 50 Hz ego는 self motion(카메라/EgoMesh)에만 사용.
