# mcar_v13 프로젝트 메모리

## 프로젝트 개요
- ROS (noetic) 기반 자율주행 시스템
- 경로: `/home/ads/mcar_v13`
- 메인 브랜치: `main`, 작업 브랜치: `ioniq5`, `siheung_dev`, `dev`
- GitHub: `yyshin-katech/mcar`

## 주요 패키지 구조
- `src/sensing/can/` - CAN 통신 (Kvaser canlib/kvaDbLib)
- `src/msgs/katech_custom_msgs/` - 커스텀 메시지 정의
- `src/msgs/mmc_msgs/` - 기존 메시지 정의
- `src/localization/` - GPS/localization
- `src/visualization/` - HMI, rviz
- `src/diagnostic/` - 각종 진단 노드
- 상세: [can_package.md](can_package.md)

## 설정
- `.claude/settings.json`: `bypassPermissions` 모드
- 사용자 언어: 한국어

## 커밋/푸시 규칙
- 커밋·푸시 시 메모리 파일도 프로젝트 `.claude/`에 동기화하여 함께 커밋
- 메모리 원본: `/home/ads/.claude/projects/-home-ads-mcar-v13/memory/` 또는 `/home/katech/.claude/projects/-home-katech-mcar-v13/memory/` (환경별)
- 복사 대상: `<repo>/.claude/`
- 대상 파일: `MEMORY.md`, `can_package.md`, `project_build.md`, `user_style.md`, `siheung_map_senario3.md`, `web_hmi_adapt_harness.md`, `web_hmi_adapt_pitfalls.md`, `feedback_no_japanese.md`, `feedback_honorific.md`, `project_qt_hmi.md`, `project_bag_replay_hmi.md`, `project_bridge_cpp_port.md`, `senario_gps_pub_harness.md` (메모리 파일 추가 시 갱신)

## 최근 작업 이력
- DBC 파일을 `CANdb_IONIQev_PCAN1.dbc` → `CANdb_IONIQ5_AD_CAN_v3.dbc`로 변경 (7개 src 파일)
- `IONIQ_CAN_reader.cpp`: `V_CAN_Release.dbc` 사용, CAN FD 지원 (500K/1M), 14개 전체 메시지 수신
- `CHASSIS_CAN_READER`: 기존 VCU 메시지 → IONIQ5 AD CAN 메시지로 교체
- launch에서 `vision_CAN_reader`, `front_RADAR_CAN_reader` 제거
- `vcu_diagnostic`: `/sensors/v_can` 구독, GearInfo `life_count`로 VCU 상태 체크
- `chassis_CAN_reader`: BrainState `life_count`로 ADCU diagnostic 추가 (`/diagnostic/adcu` 퍼블리시)
- launch: `chassis_CAN_reader`, `IONIQ_CAN_reader` 활성화
- `cpt7_gps_diagnostic`: Novatel → ublox NavPVT(`/ublox/navpvt`) 구독으로 변경 (단, 메인 GPS는 여전히 Novatel — `/sensors/gps/novatel`, `/sensors/gps/bestpos`, `/sensors/gps/inspva` 사용. ublox는 일부 노드/진단·web_hmi 보조 용도)
- `cpt7_gps_diagnostic`: GPSRTK_StatCode를 fixType → carrSoln(`(flags>>6)&0x03`)으로 변경 (0=No RTK, 1=Float, 2=Fixed)
- `stat_display`: GPS 색상 판단을 carrSoln 기반으로 변경 (Fixed=2 초록, Float/No RTK 주황)
- `stat_display`: LIDAR `lidar_status=2` 강제 덮어쓰기 버그 수정, CAM/RADAR 항상 정상 처리
- `stat_display`: `local_msg.Road_State=1` 강제 설정 버그 제거 (ODD 팝업 원인)
- `to_control_team_demo.py`: ODD_YAW_ERR_THRESHOLD 5°→30°로 완화
- `pyqt_hmi`: "Vehicle Top View" 타이틀 제거, 차량 뷰 center_y 0.75→0.5로 상향
- `pyqt_hmi`: Driving Mode 표시를 eps_status → `/sensors/ioniq5_ad_can` autonomous_mode(0=Manual,1=Auto)로 변경
- `pyqt_hmi`: ODD 초기값 2→0, local_callback에 update_sensors_signal.emit() 추가 (ODD 깜빡임 수정)
- `cpt7_gps_diagnostic`: AliveCnt를 항상 증가시키도록 변경 (GPS 텍스트 빨강 깜빡임 수정)
- `pyqt_hmi`: GPS Information 그룹 추가 (Curr LANE, GPSRTK 표시), Auto/Manual 버튼 크기 확대
- `pyqt_hmi`: 차량 뷰에 스티어링 휠 아이콘 추가 (`/sensors/v_can` steering_angle 연동, 각도 회전)
- `pyqt_hmi`: 스티어링 업데이트를 pyqtSignal로 변경 (ROS 콜백 스레드 안전)
- `pyqt_hmi`: GPSRTK 표시를 carrSoln 기반으로 변경 (Fixed/Float/No RTK)
- `stat_display`: Local_Text_Gen에 LANE/RTK 텍스트 추가, 배경 투명, 글자색 흰색
- `stat_display`: Local_Text_Gen stray "11" 문자 버그 수정
- `vspd_CAN_writer`: 신규 노드 추가 - `/sensors/v_can` 구독, 휠스피드 평균→km/h, gear_status R=reverse, 0x123 MGI_vSpd_Gateway 100Hz 전송
- `chassis_CAN_reader`: DBC v3→v4로 변경
- `ublox_gps/node.cpp`: rtcmCallback 및 /rtcm 구독 제거 (GPS 장치 자체에서 RTCM 수신), configure/poll 등 초기화 코드는 원복 유지
- `ublox_gps/config/zed_f9k.yaml`: tmode3=0 추가 (ZED-F9P HPG 제품은 tmode3 필수, 0=Disabled 로버모드)
- `to_control_team_demo.py`: LINK_ID 10 속도 제한(40km/h) 주석 처리(비활성화)
- `diagnostic_only.launch`: base2ego TF 프레임 슬래시 수정 (`ego_frame` → `/ego_frame`, rviz frame_id 매칭)
- `to_control_team_demo.py`: ODD_OCCUPIED_OFFSET_THRESHOLD 0.95m → 2.0m로 완화
- `ublox_gps/config/zed_f9k.yaml`: config_on_startup=false로 변경 (GPS에 설정 명령 전송 방지)
- `pyqt_hmi`: vehicle_view 리디자인 (IONIQ5 스타일 차체, 동심원 그리드, 그라데이션/유리창/라이트/바퀴)
- `pyqt_hmi`: 맵 경로 `/home/yuyeong/` → `/home/ads/`로 수정
- `pyqt_hmi`: `/track_Multi_RS` 구독, 오브젝트 실시간 표시 (차량=빨강, 보행자=파랑, ID+거리)
- `pyqt_hmi`: 오브젝트 좌표 매핑 수정 (x=앞→화면위, y=왼→화면왼)
- `pyqt_hmi`: 시스템 고장/ODD 팝업 오버레이 추가 (stat_display와 동일 로직)
- `pyqt_hmi`: 신호등 표시 추가 (SPAT 파싱, 원형 색상 인디케이터 + 남은 시간)
- `pyqt_hmi`: 신호등 색상 매핑을 stat_display와 동일하게 수정 (color 1=초록, 2=주황, 3=빨강)
- `pyqt_hmi`: 맵 렌더링 최적화 - bbox 기반 자차 주변 200m 이내 feature만 그리기
- stat_display 신호등 색상 매핑: phase 3→color 3(빨강렌더), phase 8→color 2(주황), phase 6→color 1(초록렌더)
- `pyqt_hmi`: 오브젝트 orientation 회전 적용 + 방향 화살표 표시
- `pyqt_hmi`: 현재 속도를 v_can wheel_speed 4개 평균 × 3.6 (m/s→km/h)로 변경 (/sensors/chassis 미사용)
- v_can DBC wheel_speed 단위: m/s (V_CAN_Release.dbc, scale=0.01)
- `pyqt_hmi`: 기어 상태 표시 추가 (P/R/N/D, D=초록, R=빨강, P/N=회색)
- v_can DBC gear_status: 0=N/A, 1=P, 2=R, 3=N, 4=D (ioniq5 브랜치)
- PCAN1 DBC Curr_gear (MOTOR_RPM msg): 0=P, 5=D, 6=N, 7=R (siheung_dev 브랜치)
- `pyqt_hmi`: diagnostic 상태 체크를 msg_received 플래그 + 미수신 카운터 방식으로 변경 (1초 미수신 시 비정상, 깜빡임 수정)
- `pyqt_hmi`: bag 녹화 UI 추가 (차량 뷰 오른쪽 위, 경로 설정, REC/STOP, rosbag record -a --split 10GB)
- `pyqt_hmi`: vehicle_view 성능 최적화 - setter에서 update() 제거, periodic_update 타이머(10Hz)에서만 단일 repaint

### siheung_dev 브랜치 작업 (2026-05-08)
- `web_hmi`: ROS 상태 판단을 `hz.adcu` → `anyTopicOk` (아무 토픽 0.5Hz 이상이면 ONLINE)
- `web_hmi`: IONIQ 5 → IONIQ EV로 명칭 변경 (F1HMI, HMIScreen)
- `web_hmi`: 오브젝트 렌더링에 CSS transition(200ms linear) 적용 (부드러운 보간)
- `web_hmi`: ThreejsF1 센서 목록에서 K-ADCU → VCU로 변경
- `hmi_state.py`: `/sensors/chassis`에서 속도(vcu_VS), 스티어링(-vcu_SAS_Angle, 부호반전), 모드(vcu_ADMDStatus) 읽기 추가
- `chassis_msg.msg`: `Curr_gear` (uint8) 필드 추가
- `can_pub_func.cpp`: MOTOR_RPM 메시지의 Curr_gear 신호 읽기 추가 (case 5)
- `hmi_state.py`: DBC Curr_gear → HMI 기어 매핑 (0=P→1, 5=D→4, 6=N→3, 7=R→2)
- `hmi_state.py`: V2X 미수신 threshold 1초→3초, StatCode>=2만 WARN (간헐적 StatCode=1 무시)
- web_hmi 2차 어댑트(2026-05-08): `hmi_state.py:144` 신호등 토픽 `/katri_v2x_node/katri_spat` → `/siheung_spat` 교체
- web_hmi 2차 어댑트: launch `map_shp`/`threejs_mapdir`을 `gps_system_localizer/src/shp_map` 루트로 변경 (senario3 sub-dir 폐기)
- web_hmi 2차 어댑트: `web_hmi_threejs_bridge.py` LAYERS_ALL = {`TB_senario_map`: polyline (1770), `TB_senario_surfaceMARK`: polygon (372)}
- web_hmi 2차 어댑트: `web/threejs/types.js` LAYER_STYLE/DEFAULT_LAYER_VIS에 두 신규 layer 키 추가 (LAYERS_ALL 동기화 누락 시 화면 빈출)
- bag 재생 시 `/hmi/*` 토픽 충돌 발견: `rosbag play <bag> /hmi/map:=/dev/null/hmi_map /hmi/threejs/map:=/dev/null/threejs_map`로 remap 필요

### siheung_dev 브랜치 작업 (2026-05-09 ~ 2026-05-10)
- web_hmi 3차 어댑트: launch `map_shp`/`threejs_mapdir`을 `HDMap_Oido_New` 폴더로 변경 (시화 시범운행 MOLIT 11 표준 레이어 EPSG:32652)
- `web_hmi_threejs_bridge.py` LAYERS_ALL → 11 MOLIT 레이어 (A1_NODE/A2_LINK/A3_DRIVEWAYSECTION/B1_SAFETYSIGN/B2_SURFACELINEMARK/B3_SURFACEMARK/C1_TRAFFICLIGHT/C3~C6)
- `web/threejs/types.js` DEFAULT_LAYER_VIS: A2_LINK 기본 visible로 변경
- **qt_hmi 신규 패키지** (`src/visualization/qt_hmi/`): Qt5/C++ 네이티브 HMI, web_hmi 동등 기능. M1~M4 PASS (스켈레톤 + RosBridge → F1Dashboard 7 위젯 → MapScene QOpenGLWidget(Core 3.3) + 15 layer + ego/track → ControlPanel QDockWidget 13 체크박스 + V2X TrafficLightWidget). 1.40 MB ELF.
- qt_hmi: 카메라 follow rviz-grade smoothness — `web_hmi_bridge`의 `/hmi/state` 10 Hz throttle 우회. RosBridge가 `/localization/to_control_team`(50 Hz, `mmc_msgs::to_control_team_from_local_msg`) 직접 구독 → `egoPoseChanged(east,north,yaw)` 시그널 → `MapScene::onEgoPoseChanged`. EMA α=0.5 (시정수 ~32 ms). 60 fps QTimer 갱신. yaw atan2(sin/cos) unwrap. `onStateChanged`는 ego 필드 보존 (10 Hz가 50 Hz 덮어쓰지 않음).
- qt-hmi-build 하네스 (`.claude/skills/qt-hmi-build/` + `.claude/agents/{design-architect,impl-coder,impl-verifier}.md`): 마일스톤 단위 3-phase 호출 (1 호출 1 M).
- bag 검증 패턴: `/hmi/state` 녹화 bag 재생 시 live `web_hmi_bridge`까지 띄우면 dual-publisher 진동. **roscore + bag + qt_hmi_node**만 띄우는 것이 정답.
- web_hmi 50 Hz 동등화 (commit 47d72af): `web_hmi_bridge.py`가 `BaseHmiStateController._cb_local`의 `'ego_pose_changed'` 이벤트(이미 50 Hz)를 `/hmi/ego_pose` 신규 토픽으로 라우팅. 프론트 `useEgoPose()` 훅 추가, `CameraController`/`EgoMesh`가 `useRosState` 대신 사용 (`useEffect` 50 Hz 재실행).
- web_hmi ego-frame 트랙 슬라이드 함정 (후속 수정): `web_hmi_threejs_bridge._on_percept`가 ego-frame 좌표(`ci.center.x/y`)로 emit하는데 `TrackBoxes`가 라이브 50 Hz ego로 trackGroup 변환하면 perception tick(~10 Hz) 사이 ego 변동만큼 트랙이 월드에서 ~1 m 슬라이드. **해결**: 브리지가 `/localization/to_control_team` 직접 구독 → `_last_ego` 캐시 → `_on_percept` 페이로드에 `ego_at_emit:{east,north,yaw}` 동봉. `TrackBoxes`는 `useEgoPose()` 제거, `tracks.ego_at_emit`로 trackGroup 변환. **일반 패턴**: ego-frame으로 발행되는 모든 *느린* 데이터(perception/free space 등)는 emit-time ego 스냅샷과 페어링. 라이브 50 Hz ego는 self-motion(카메라/EgoMesh)에만.

### siheung_dev 브랜치 작업 (2026-05-11)
- `diagnostic_only.launch`: pyqt_hmi의 `lateral_offset_relay.py`/`gps_std_relay.py` 두 노드는 rviz overlay text 전용 (web_hmi 와 무관). `ioniq_statdisplay.rviz` 가 이 토픽들(`/rviz/lateral_offset` Plotter2D, `/rviz/jsk/{lane_end_text,stopline_popup,gps_std_text}` OverlayText)을 구독하므로 launch에 활성 유지 필요.
- **rosbridge JSON stutter 함정**: `/hmi/threejs/tracks` 페이로드에 트랙당 `PERCEPT_MAX_POINTS_PER_TRACK=4096` 점군이 포함되어 메시지당 ~2.5MB. percept 5.9 Hz 입력이 rosbridge에서 ~0.9 Hz로 throttle되어 박스가 1초마다 점프 (자차는 50 Hz 부드러움). **해결**: cap 4096→256, 신규 `TRACKS_MAX_RENDERED=6`으로 ego-frame 거리(x²+y²) 정렬 후 가까운 N개만 페이로드/점군 슬라이싱. 페이로드 ~75 KB로 축소. **일반 패턴**: rosbridge std_msgs/String JSON은 1MB 안팎부터 stutter. `rostopic hz`(publish 측) + `rostopic bw`(payload 크기) 동시 측정으로 진단.
- `web_hmi_bridge._publish_objects` (`/hmi/objects`): ego-frame 거리순 정렬 후 `OBJECTS_MAX=6`개만 발행 (HMI 하단 "TRACKS" 카운트/2D 오버레이가 3D 박스 6개와 일관). hmi_state.objects 자체는 안 자름 (pyqt_hmi 영향 회피).
- `_slice_points` 콜백 최적화: `cloud_indices[:cap]` 슬라이스 후 변환. 기존엔 전체 길이 list comp이라 std_msgs/Int32 `.data` attribute access가 콜백 시간 지배 → percept queue_size=1 드롭 유발.
- `TrackBoxes.jsx` `TRACK_MISS_GRACE` 10 → 4: trim된 먼 트랙이 grace만큼 화면 잔존해 6 cap 넘게 보이는 부작용 축소 (입력 hz가 낮을수록 grace 실제 시간이 늘어남).
- **percept-filter-tune 하네스 신규** (`claude_work_list/object_filter.md` 사양): agents 3 (percept-filter-analyst/coder/verifier) + skills/percept-filter-tune. 파이프라인 패턴 (분석→구현→검증). 작업 워크스페이스: `_filter_workspace/`.
- `percept_topic_matcher.cpp` 필터 정책 변경: 기존 `attention_type==1 우선 + priority_id ASC + 14 break` → 공간 기반 (전방 100m 반경 FRONT 우선 / 그 외 SIDE 보충) + `attention_type` 내부 가중치. 신규 상수 `FRONT_RANGE_M=100.0`, `PriorityObj`에 `x/y/dist2` 필드 추가, 단일 `collected` 벡터, 정렬 람다 2종 (FRONT: attention DESC → dist2 ASC / SIDE: attention DESC → |y| ASC → dist2 ASC), `max_objects_to_publish=14` cap, sqrt 0건(제곱 비교만). 로그 `Attention=1/0` → `Front (<=100m)/Side`. 다운스트림 `track_CAN_writer_*` 슬롯 어사인 순서 바뀜에 주의.
- `percept_topic_matcher.cpp` 후속 컷: `LATERAL_RANGE_M=5.0` 신규 상수로 `|y| > 5m` 객체는 confidence 컷 직후 / 보행자 캐시 사용 직전 둘 다에서 컷. FRONT 영역이 실질적으로 전방 100m × 좌우 ±5m 박스로 좁아짐.
- `percept_topic_matcher.cpp` 정책 전면 개편: FRONT/SIDE 분할·attention 우선 정렬 폐기. 인식 영역 OR 3개: A) `|y|≤5 && x∈[-40, 80]` (박스, 자차 옆까지 그대로 포함) / B) `|y|>5 && x∈[10, 80]` (전방 측면) / C) `|y|>5 && x∈[-40, -10]` (후방 측면). 사각지대: 측면(`|y|>5`)에서 자차 주위 `x∈(-10, 10)` 만. 상수: `FRONT_RANGE_M=80`, `REAR_RANGE_M=40`, `LATERAL_RANGE_M=5`, `FRONT_NEAR_X_M=10`, `REAR_NEAR_X_M=10` (측면 cut 전용 — 박스에는 미적용). 단일 정렬: `(int)sqrt(x²+y²)` ASC → `|y|` ASC. 14개 cap 유지. 시각화: `claude_work_list/percept_filter_zones.html` (radial gradient).
- `rviz_filter.cpp` ROI 시각화: `percept_callback` 끝에 마커 5개 (`ns="roi"`, ego_frame, lifetime 1s). A=cyan 닫힌 사각형(박스), B 전방측면=orange / C 후방측면=yellow는 "ㄷ"자(안쪽 `|y|=±5` 라인 + 양 끝 2m 외향 tick) — 외곽 |y|=∞ 표현. ROI 상수는 percept_topic_matcher.cpp와 동일 값으로 인라인. 두 파일 동시 갱신 필수.
- 우측 측면 cut: `percept_topic_matcher.cpp` + `rviz_filter.cpp` 둘 다 자차 우측(`y < -5`) 객체 전체 제거. 측면 조건 `abs_y > LATERAL` → `curr_y > LATERAL` (좌측만). 박스 A(`|y|≤5`)는 영향 없음 — 우측 5m 이내는 그대로 포함. rviz_filter 우측 측면 마커(id 2, 4) 제거.
- `hmi_state.py` 신호등 phase 매핑 보강: stat_display.cpp와 정렬해 case 5(permissive movement allowed)→1=초록, case 7(permissive clearance)→2=주황 추가. 기존엔 case 6/8/3만 인식해 phase 5/7 들어올 때 color=0으로 빠져 web_hmi 신호등 색상 미표시.

### siheung_dev 브랜치 작업 (2026-05-12)
- **bridge-cpp-port 하네스 신규** (`.claude/skills/bridge-cpp-port/` + agents 3 (bridge-port-analyst/coder/verifier)): Python ROS bridge → C++ 외과적 분리 파이프라인. 워크스페이스 `_bridge_cpp_workspace/`.
- `web_hmi_threejs_bridge.py` 의 `/hmi/threejs/tracks` 발행을 신규 C++ 노드 `web_hmi_threejs_tracks_cpp` 로 분리 (옵션 2 — map latched 발행은 Python 잔존). hz 0.78 → **9.98 Hz** (~13×), 1초 점프 해결. 페이로드 51 KB/msg × 10 Hz = 503 KB/s.
- C++ 노드 핵심 (`src/visualization/web_hmi/src/web_hmi_threejs_tracks_node.cpp`, 339 lines): PointCloud2 `ConstPtr` 보관 + `reinterpret_cast<const float*>(data())` zero-copy + `point_step/4` 스트라이드 인덱싱. `std::partial_sort` dist² ASC top 6 → resize → 생존자만 `slicePoints` (cap 256 먼저 → NaN/Inf 드롭 → mm round). jsoncpp `StreamWriterBuilder` indentation="" 압축. 3개 구독 모두 `transport_hints().tcpNoDelay()`. 단일 스레드 `ros::spin()` (mutex 없음).
- **dual-publisher 회피 패턴** (외과적 분리 일반화): 같은 토픽 두 노드 advertise 시 마지막 advertise wins + queue 충돌 — 한쪽 반드시 비활성화. `web_hmi_threejs_bridge.py` 에 `~publish_tracks` ROS param (기본 False) 가드 — False 면 `_pub_tracks` advertise + percept/cloud/local 구독 + 5s 로그 타이머 전부 스킵, map latched 발행만 유지. launch 에서 Python 노드에 `<param name="publish_tracks" value="false"/>` 명시 주입.
- 메시지 오타 필드 (perception_ros_msg): `CoreInfo.trakcer_id` (sic), `Object.hassupplmentinfo` (sic), `SupplementInfo.cloud_indices[k].data` (Int32 객체 배열). 포팅 시 그대로 유지 필수.
- **Python → C++ 콜백 비용 함정**: `std_msgs/Int32 .data` attribute access 가 Python 콜백 시간 지배 (cloud_indices 만개 단위). C++ 직접 멤버 access 로 사실상 0 비용. cap 먼저 (`indices[:256]`) 원칙은 그대로 유지 (페이로드 크기 한도용).
- 빌드 의존성: `libjsoncpp-dev` (시스템 패키지) + `pkg_check_modules(JSONCPP REQUIRED jsoncpp)` — nlohmann vendoring 회피. `find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs perception_ros_msg mmc_msgs)` (rospy 기존 라인에 확장).
- 검증 순서: (1) `rostopic info /hmi/threejs/tracks` Publishers 단일 cpp 확인 → (2) `rosnode info /web_hmi_threejs_bridge` Subscriptions 비었는지 → (3) `rostopic hz/bw` 측정.
- **6개 cap 일괄 제거** (cpp 포팅으로 콜백 여유 확보된 뒤): `web_hmi_threejs_tracks_node.cpp`의 `TRACKS_MAX_RENDERED=6` 상수 + `partial_sort` cap 분기 제거 → 단순 `std::sort(dist² ASC)`. `web_hmi_threejs_bridge.py` 동일 상수/슬라이스 제거 (가드 비활성이라도 일관성). `web_hmi_bridge.py`의 `OBJECTS_MAX=6` 제거, 정렬은 유지. 상한은 이제 `percept_topic_matcher.cpp`의 14개 cap이 결정. 점군 cap `PERCEPT_MAX_POINTS_PER_TRACK=256`은 페이로드 안정성용으로 유지. 라이브 적용은 `roslaunch web_hmi web_hmi.launch` 재기동 필요 (cpp 노드 `required="true"`).

### siheung_dev 브랜치 작업 (2026-05-18)
- `lanelet_marker.py` POINTZ shapefile 크래시 수정 (`'float' object is not subscriptable`): fiona 가 Point geometry 를 `coordinates = (x,y,z)` **단일 튜플** 로 반환하는데 기존 코드는 LineString/Polygon 처럼 좌표 목록으로 가정 → `[coord[0] for coord in coords]` 에서 float 인덱싱 시도. 해결: `geom_type in ('Point', 'MultiPoint')` 면 continue (LINE_STRIP 마커는 ≥2 점 필요하므로 어차피 못 그림). HDMap_Oido_New 의 A1_NODE, B1_SAFETYSIGN, C1_TRAFFICLIGHT 등 POINTZ 레이어가 원인이었음.
- `lanelet_marker.py` A2_LINK 전용으로 좁힘: `load_multiple_shapefiles(shp_map_path, "*.shp")` → `"A2_LINK.shp"`. HDMap_Oido_New 전체 레이어(11개) 로드 시 마커 9226개 → RViz 성능 경고(임계값 5000). 차선만 그리면 충분 + 성능 여유. `lanelet_marker.py:271`. 다른 레이어 시각화는 `/hmi/threejs/map` (web_hmi) 또는 향후 별도 노드에서 처리.
- **일반 패턴 — fiona shapefile geometry 분기**: Point/MultiPoint/LineString/MultiLineString/Polygon/MultiPolygon 모두 `coordinates` 구조가 다름. Point=`(x,y,z)` 단일 튜플, LineString=`[(x,y,z),...]`, Polygon=`[ring1, ring2, ...]` (ring 이 LineString 형태). `len(coordinates)==0` 체크만으로는 Point 는 못 거름 (튜플 len=3). geom_type 분기 필수.

### siheung_dev 브랜치 작업 (2026-06-22)
- **SPaT 병합 노드 신규** `spat_merge_node` (`src/v2x/siheung_v2x/src/spat_merge_node.cpp`): OBU(`/siheung_spat`) + MQTT(`/siheung_v2x/mqtt_spat`) 를 교차로(IID) 단위 OBU 우선으로 병합 → 단일 `/spat_merged` 10Hz 재발행. OBU 가 안 싣는 MQTT-only 교차로(특히 **IID 302**, 링크 417 타깃)를 HMI/CAN 에 전달. 상세: [spat-merge-obu-mqtt](spat_merge_obu_mqtt.md)
- 기존 문제: HMI 3곳(web_hmi/pyqt_hmi `hmi_state.py`·`main_window.py`, stat_display)이 `/siheung_spat`(OBU)만 구독 → MQTT-only 교차로 미표시. `spat_CAN_writer` 의 OBU 우선 게이트가 전역(all-or-nothing)이라 OBU 가 살아있으면 302 MQTT 전체 skip.
- 병합 키 `(IID, SignalGroupID, MovementStateName)` — `v2x_msgs/intersection_msg.Movements` 는 단일 movement_msg(배열 아님)라 같은 IID 가 movement 마다 data[] 원소로 분리. 발행 시점에 IID 별 승자 소스 결정(OBU fresh `obu_priority_timeout`=2s 면 OBU, 아니면 MQTT)으로 소스 혼합 방지, `entry_ttl`=3s 로 죽은 신호 제거.
- 소비자 4곳 `/siheung_spat` → `/spat_merged` 재지정: `spat_CAN_writer.cpp`(sub3 mqtt 직접구독 제거), `stat_display.cpp:44`, pyqt_hmi `main_window.py:602`·`utils/hmi_state.py:144`(web_hmi 공용). `v2x_diagnostic` 은 OBU 건전성용이라 raw `/siheung_spat` 유지. CMake 타겟 + `launch/siheung.launch` 노드 기동 추가.
- 링크→IID 매핑은 `link_*.mat` 의 `look_at_IntersectionID/look_at_signalGroupID/MANUAVER`. 링크 **417 → IID 302, SG 70, MANUAVER -1(LEFT), is_stop_line 1**. 소비자(CAN·HMI) 모두 ego 타깃 IID 하나만 필터.

## Diagnostic 구조
| 토픽 | 메시지 타입 | 소스 노드 | 판단 기준 |
|------|-------------|-----------|-----------|
| `/diagnostic/vcu` | `vcu_diagnostic_msg` | vcu_diagnostic | V_CAN GearInfo life_count |
| `/diagnostic/adcu` | `k_adcu_diagnostic_msg` | chassis_CAN_reader | AD_CAN BrainState life_count |
| `/diagnostic/cpt7_gps` | `cpt7_gps_diagnostic_msg` | cpt7_gps_diagnostic | ublox NavPVT carrSoln |
- 공통 패턴: 콜백에서 msg_received 플래그 설정, 타이머에서 플래그 확인 후 리셋
- GPSRTK_StatCode: carrSoln 값 (0=No RTK, 1=Float, 2=Fixed)
- stat_display GPS 색상: carrSoln>=2(Fixed) 초록, <2(Float/No RTK) 주황, 통신끊김 빨강
- GPS fixType: 0=NO_FIX, 2=2D, 3=3D, 4=GNSS+DR (GPS_INS_SolutionStat에 사용)

## 추가 메모리 파일
- [Project Build & Branch Status](project_build.md) — 빌드 경로, 브랜치 구조, 최근 변경
- [User Communication Style](user_style.md) — 한국어 짧은 명령 선호, 간결 응답
- [siheung_dev 활성 맵](siheung_map_senario3.md) — shp_map 루트 두 .shp(POLYLINEZ 1770 + POLYGONZ 372), EPSG:32652→5179 변환
- [web-hmi-adapt harness](web_hmi_adapt_harness.md) — 브랜치 간 web_hmi 어댑트 파이프라인 (.claude/agents+skills)
- [qt_hmi + web_hmi rviz-grade follow](project_qt_hmi.md) — 50Hz ego pose 패턴(둘 다); ego-frame 트랙은 emit-time ego 스냅샷(`ego_at_emit`)과 페어링 필수
- [bridge cpp 포팅 패턴](project_bridge_cpp_port.md) — Python `/hmi/threejs/tracks` 0.78 Hz → C++ 9.98 Hz. zero-copy PointCloud2 + dual-publisher 회피(`~publish_tracks` 가드) 일반 패턴
- [senario-gps-pub harness](senario_gps_pub_harness.md) — senario HTML 주행 link 시퀀스 → 40 km/h GPS 시뮬레이션 publisher 신규 개발 파이프라인
- [spat-merge OBU+MQTT](spat_merge_obu_mqtt.md) — `spat_merge_node` 교차로 단위 OBU 우선 병합(/spat_merged). MQTT-only 교차로(302)를 HMI/CAN 에 전달
- [spat_viewer 실행](spat_viewer_run.md) — MQTT/OBU SPaT 라이브 Leaflet 뷰어. 별도 터미널 `roslaunch spat_viewer spat_viewer.launch` + `http://localhost:8080/` (rosbridge 9090/http 8080)

## 피드백 메모리
- [일본어 사용 금지](feedback_no_japanese.md) — 응답에 일본어(한자) 금지, 한국어만 사용
- [존댓말만 사용](feedback_honorific.md) — 응답 종결은 "~합니다/입니다" 존댓말로 통일, 반말 금지
- [web_hmi 어댑트 함정](web_hmi_adapt_pitfalls.md) — LAYER_STYLE 동기화 누락 / bag /hmi/* 충돌 (ROS만 패치하면 화면 안 나옴)
