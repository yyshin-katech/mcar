# MCAR Project Memory Index

## Memory Files
- [build-deps.md](build-deps.md) — 빌드 의존성 (apt, Kvaser SDK, CMake 노트)
- [feedback_no_tmux_panes.md](feedback_no_tmux_panes.md) — ROS 에이전트팀 실행/종료 패턴 (백그라운드, 좀비 프로세스 정리)
- [can_channel_map.md](can_channel_map.md) — CAN 채널 할당표 (노드별 채널, DBC, R/W)
- [kvaser_driver_kernel.md](kvaser_driver_kernel.md) — Kvaser canlib 채널 0개 진단. 5.15.0-139가 정규 커널(GRUB/apt hold 고정, 2026-04-23)
- [wifi_ax211.md](wifi_ax211.md) — Intel AX211(51f1) Wi-Fi 복구: linux-modules-iwlwifi-*-generic 사용 (backport-dkms는 5.15 미지원)
- [audio_setup.md](audio_setup.md) — NUC13ANH-B 오디오(ALC256 analog 잭만, HDMI 미지원), PulseAudio 카드 프로파일 off 복구 절차
- [project_ioniq5_work.md](project_ioniq5_work.md) — ioniq5 브랜치 2026-04 작업 이력

### 개발 머신(WSL) auto-memory (yuyeong)
- [user_profile.md](user_profile.md) — 카텍 자율주행 연구원, 시흥+K-City 시나리오, 브랜치 ioniq5_hmi_dev
- [v2x_package.md](v2x_package.md) — siheung_v2x 패키지 구조, SPaT/SDSM 디코딩
- [reference_spat_timestamp.md](reference_spat_timestamp.md) — OBU는 MOY+DSecond 있으나 9h 오프셋, MQTT는 MOY=0 (절대시각 없음)
- [environment.md](environment.md) — WSL2 개발환경, mirrored 네트워킹 가이드
- [map_data.md](map_data.md) — senario / senario3 mat 구조, MANUAVER 라벨(curve_lane.md) 원천
- [senario_mat_viewer.md](senario_mat_viewer.md) — ~/temp 하네스, cs2cs로 EPSG:5179→WGS84
- [reference_kcity_mat_viewer.md](reference_kcity_mat_viewer.md) — K_CITY mat 편집/뷰어. ~/kcity_map/regen_kcity_viewer.py(pyproj, var DATA 1줄 교체, build_feature 재사용). mat east/north는 row(1,N) 필수(to_control_team ['east'][0]). 2026-06-10 link_79/83 끝40m 단순절삭 + link_81/82/83 row 통일
- [canoe_tool.md](canoe_tool.md) — CAN 송신 검증에 Vector CANoe 사용, rate/timing 디버깅의 1차 근거
- [feedback_can_request_signals.md](feedback_can_request_signals.md) — MD_AD_Req 같은 request 신호는 1초 펄스, 활성 모드면 재요청 버튼 비활성화
- [project_gps_hardware.md](project_gps_hardware.md) — ZED-F9K USB serial(`/dev/ttyACM0`), 이더넷/NTRIP 미사용. cpt7 인터넷 ping 의도적 비활성
- [feedback_root_permissions.md](feedback_root_permissions.md) — root 필요한 SOCK_RAW 대신 `system("ping ...")` + worker thread 패턴 선호
- [feedback_disable_undo.md](feedback_disable_undo.md) — "무력화 해제" 요청은 literal revert가 아니라 가려져있던 로직 버그까지 정상화하는 의미
- [project_pyqt_hmi_primary.md](project_pyqt_hmi_primary.md) — 모니터링 화면은 pyqt_hmi가 메인, 빌드 2개(기본/A-1) 둘 다 반영, 진단 status 0/1/2 규약
- [project_active_branch_map.md](project_active_branch_map.md) — ioniq5_hmi_dev + K_CITY_20260608, to_control_team_demo가 mat 값을 코드에서 덮어쓰는 gotcha
- [project_v2x_spat_topic.md](project_v2x_spat_topic.md) — SPaT 활성 토픽 브랜치마다 정반대: ioniq5_hmi_dev=`/katri_v2x_node/katri_spat`(katri_obu_interface 활성), siheung_dev=`/siheung_spat`. 점검 전 git branch 확인. MovementStateName은 소비자 없음
- [reference_decode_md_spat_patch.md](reference_decode_md_spat_patch.md) — ~/decode_md/spat_udp_decode.py preprocess 보강 (ENUMERATED 두 번째 ... + trailing comma) + Windows Python 실행 필요
- [feedback_wsl_rostopic_hz.md](feedback_wsl_rostopic_hz.md) — hz 가 Terminated 만 떠도 토픽은 흐를 수 있음. echo -n N 카운트 / tcpdump 로 cross-check
- [project_k_city_20260618.md](project_k_city_20260618.md) — shp→mat 변환(EPSG:32652→5179), link_61교체+86/87/88추가, MAX_LANE_ID=88. gotcha: convert_links.py가 이전 링크 NEXT 미갱신→link_60 NEXT 61→88 수동수정(2026-06-18, commit 9729337). 학교구역 경고 61→61/86/87/88 확대(LINK_ID 분기, Road_State==1보다 우선, 음성은 stat_display GUARDZONE만)
- [feedback_wslg_wheel_input.md](feedback_wslg_wheel_input.md) — WSLg는 wheel/클릭을 커서 위치 기준 전달. PyQt wheelEvent에 isActiveWindow 가드 필요(다른 창 선택해도 줌 먹는 증상)
- [reference_diag_replay_harness.md](reference_diag_replay_harness.md) — ~/diag_replay_sample, CANoe mat(v7.3/h5py)→rosbag, vcu_diagnostic 검증(make_vcan_bag.py=구 v_can / make_adcan_bag.py=현행 AutonomousState) + 슈퍼바이저/종/횡 고장구간 라벨링
- [project_control_fault_not_displayed.md](project_control_fault_not_displayed.md) — 슈퍼바이저/횡/종 error_code 표출 경로. 2026-06-09 vcu_diagnostic을 AutonomousState(life_count staleness OR error_code≠0) 기반으로 교체 → VCU 슬롯 표출
- [project_v2x_spat_takeover_decoupled.md](project_v2x_spat_takeover_decoupled.md) — V2X SPaT↔TOR. AliveCount(=SPaT 수신 카운터) 경로로 SPaT 갭이 TOR 유발. 2026-06-09 stat_display 게이팅: 신호등 필요(look_at≠0)+SPaT 0.5s 미수신만 v2x_status=2→TOR, SPaT 없으면 주황. off-intersection false TOR 제거
- [project_blackbox_recorder.md](project_blackbox_recorder.md) — pyqt_hmi blackbox_recorder 노드. TOR 트리거 시 [이전10s,이후2s] 저장(LiDAR/인지 13토픽 제외). 경량 모니터+C++ rosbag record 롤링버퍼(Python 전토픽 버퍼링은 실차부하에 막힘). 합성 하네스 PASS, 실차 종단검증 권장
- [project_altitude_can_flow.md](project_altitude_can_flow.md) — 2026-06 hMSL→host_altitude→CAR_EGO_A_Ex.ALTITUDE(id1830, 32bit float byte4-7, DLC4→8). DBC v7 신설(파일별 분리 관례), writer+reader 둘 다 v7. local_CAN_writer는 dlc=8 고정 송신. 실차/CANoe rate·timing 미검증
- [project_gps_status_takeover.md](project_gps_status_takeover.md) — 2026-06-10 GPS 고장 기준 통일(RTK Fixed 아님/std>15cm=고장, >5cm=경고; pyqt 기본·A-1 + stat_display). TOR는 stat_display→/diagnostic/system→to_control_team_demo 경유(pyqt는 표시 전용). any(s!=0)이라 경고도 TOR 유발(현행 유지). 발행 1s 주기·RTK 플랩핑 false TOR 주의
- [project_spat_can_writer_switch.md](project_spat_can_writer_switch.md) — 2026-06-17 spat_CAN_writer 교차로 처리 구조. switch(intersection_id)는 temp_intersection_id_msg=교차로ID/100 설정하나 dead store(미사용). 실제 (교차로,그룹) 발화는 look_at_IntersectionID/signalGroupID(to_control_team) 매칭으로 결정. case(100)→1, case(1500)→15 추가. signalGroup 16/10은 switch 무관, look_at 경유

### 차량 머신(katech NUC) auto-memory
`siheung_dev` 계열에서 축적된 메모리. 2026-08-07 `ioniq5_siheung_dev` 로 동기화.
- [project_build.md](project_build.md) — 빌드 경로, **브랜치 계열=차량 플랫폼(siheung_*=아이오닉 EV / ioniq5_*=IONIQ 5)**, 2026-08-07 리네임 이력 / 브랜치 전환 직후 첫 빌드 msg 헤더 경합 실패(재빌드로 해소)
- [project_ioniq5_siheung_port.md](project_ioniq5_siheung_port.md) — **이 브랜치의 현행 작업**: web_hmi + senario 맵(293링크 417~3877 비연속) + to_control_team_demo.py 를 siheung_release 에 정렬. K-City 하드코딩 제거로 `else: Speed_Limit=30` 맵값 파괴 버그 해소. msg int8→int32 + MANUAVER, DBC v8 LOCAL_MAP_INFO 12bit 전체 repack(제어팀 동기화 필요)
- [can_package.md](can_package.md) — sensing/can 패키지 구조, 실행파일/DBC/채널 매핑
- [user_style.md](user_style.md) — 한국어 짧은 명령 선호, 간결 응답
- [feedback_honorific.md](feedback_honorific.md) — 응답 종결은 "~합니다/입니다" 존댓말로 통일
- [feedback_can_frozen_additive.md](feedback_can_frozen_additive.md) — 차량 CAN-feeding 토픽은 수정 말고 additive 신규 토픽으로 우회(무변경 증명 git diff ∅/sha256)
- [siheung_map_senario3.md](siheung_map_senario3.md) — shp_map 루트 두 .shp(POLYLINEZ 1770 + POLYGONZ 372), EPSG:32652→5179 변환
- [reference_a2_link_shp.md](reference_a2_link_shp.md) — senario_shp_20260623 A2_LINK, UTM52N(32652)→5179, 좌/중/우 3차선, ITSLinkID 1:N, ToNode→FromNode 위상. web_hmi 지도/경로 원천
- [stopline_adj.md](stopline_adj.md) — senario mat 링크를 B2_SURFACELINEMARK 정지선까지 연장 + 다음 링크 트림(station 재계산, 공유정점). mat=5179/shp=32652
- [lanelet_fiona_geometry.md](lanelet_fiona_geometry.md) — POINTZ `'float' not subscriptable` 크래시, A2_LINK 전용화, geom_type별 coordinates 구조 차이
- [senario_gps_pub_harness.md](senario_gps_pub_harness.md) — senario HTML 주행 link 시퀀스 → 40 km/h GPS 시뮬레이션 publisher
- [web_hmi_adapt_harness.md](web_hmi_adapt_harness.md) — 브랜치 간 web_hmi 어댑트 파이프라인. chassis_msg Curr_gear 필드+HMI 매핑, V2X 미수신 threshold 3s/StatCode≥2 WARN
- [web_hmi_adapt_pitfalls.md](web_hmi_adapt_pitfalls.md) — LAYER_STYLE 동기화 누락 / polyline `alpha` 무시 / bag `/hmi/*` 충돌 (ROS만 패치하면 화면 안 나옴)
- [project_qt_hmi.md](project_qt_hmi.md) — 50Hz ego pose 패턴; ego-frame 트랙은 emit-time ego 스냅샷(`ego_at_emit`)과 페어링 필수
- [project_bridge_cpp_port.md](project_bridge_cpp_port.md) — Python `/hmi/threejs/tracks` 0.78→9.98 Hz. zero-copy PointCloud2 + dual-publisher 회피(`~publish_tracks` 가드)
- [project_global_nav_hmi.md](project_global_nav_hmi.md) — web_hmi 주행 예정 경로 중앙차선 리본 + TIM 트리거 latch. 오프라인 route JSON
- [project_bag_replay_hmi.md](project_bag_replay_hmi.md) — bag 에 `/hmi/*` 까지 녹화돼 있어 live 브리지와 dual-publisher. `/hmi/*:=/sink/*` remap 필수
- [reference_tim_pedes_bag_replay.md](reference_tim_pedes_bag_replay.md) — `web_hmi_replay.launch`(bag→web_hmi threejs_f1). 구 bag 으론 퓨전 own 경로 검증 불가
- [percept_filter_policy.md](percept_filter_policy.md) — percept_topic_matcher.cpp+rviz_filter.cpp 3-zone OR, 우측컷, 14개 cap. 두 파일 ROI 동시 갱신
- [perception_object_type.md](perception_object_type.md) — RS class enum `1=CONE,2=PED,3=BIC,4=CAR,5=TRUCK_BUS,6=ULTRA`. object_msg.status=class(type). 실데이터 type3(BIC)이 차량크기 → 보행자 판정은 `type==1` 로 원복
- [crosswalk_position.md](crosswalk_position.md) — 횡단보도 1~18 폴리곤(WGS84→EPSG:5179 리터럴), CW_LINKS 게이팅, OBU 는 #1/#2 전용
- [ped_detector_cpp.md](ped_detector_cpp.md) — on_crosswalk 검출 Python→C++ 포팅. 게이트 타입→멤버십→방향(PCA 길이축 ≤40°). 객체 vx,vy=ego-상대
- [spat_merge_obu_mqtt.md](spat_merge_obu_mqtt.md) — `spat_merge_node` 교차로(IID) 단위 OBU 우선 병합(`/spat_merged`), MQTT-only 교차로 302 전달
- [spat_dir_match.md](spat_dir_match.md) — MANUAVER -1/0/1 ↔ MovementStateName(LEFT/STR/RIGHT). HMI 방향필터 부재로 알파벳순 LEFT 오선택 버그
- [spat_viewer_run.md](spat_viewer_run.md) — SPaT ① 라이브 Leaflet 뷰어 ② offline bag 리플레이 뷰어(ego 방향매칭 신호등 재생/스크럽)
- [spat_viewer_build_harness.md](spat_viewer_build_harness.md) — spat_viewer 패키지 신규(rosbridge:9090 + http.server:8080). 데이터는 `mapfiles/senario/link_*.mat` 직접 추출
- [project_mqtt_v2n_spat.md](project_mqtt_v2n_spat.md) — 시흥 V2N MQTT payload 앞 16 byte = 경찰청 V2N container. fid/psid 는 메시지 타입별 상이
- [reference_v2n_fid_psid.md](reference_v2n_fid_psid.md) — 경찰청 V2N 메시지별 fid/psid/방향/토픽 xlsx 위치 (16-byte 헤더 단일 출처)
- [mqtt_vpn_setup_harness.md](mqtt_vpn_setup_harness.md) — MQTT V2N 트래픽만 시흥시 VPN split-tunnel. 실환경은 SecuwaySSL, 하네스 사양은 OpenVPN 기준
- [project_vehicle_tracker.md](project_vehicle_tracker.md) — BSM 기반 차량 위치 관제, EC2 배포 구성
- [gps_warning_criteria.md](gps_warning_criteria.md) — rviz `/rviz/jsk/gps_stat` 색: 주황=`StatCode!=0x38 || std>5cm`, 빨강=`Network_Status`(ping 8.8.8.8, GPS 무관)
- [reference_sdsm_bag_data.md](reference_sdsm_bag_data.md) — SDSM(J3224) 수집 데이터는 `~/20251128/sdsm_data/` 가 유일(bag 7개 `/obu/sdsm` 1,890 msgs + pcapng + unified CSV 3,722행). objType 전량 Unknown, RSU sourceID 시나리오별 상이. `~/bag`·`~/bag_data` bag 73개엔 SDSM 없음

## Project Overview
- ROS Noetic catkin workspace for autonomous driving (KATECH)
- Location: `/home/katech/mcar/`
- Main branch: `main` (2026-01-07 이후 변경 없음), active branches: `ioniq5_siheung_dev`(2026-08 현행), `ioniq5_release`, `ioniq5_kcity_tested`, `siheung_dev`, `siheung_release`
- sudo password: `1`

## 브랜치 계열 = 차량 플랫폼 (2026-08-07 확정)
**브랜치 이름의 계열이 곧 차량 버전이다.** 코드나 git 이력만으로는 알 수 없음.

| 계열 | 차량 |
|------|------|
| `siheung_*` (`siheung_dev`, `siheung_release`) | **아이오닉 EV** |
| `ioniq5_*` (`ioniq5_release`, `ioniq5_kcity_tested`, `ioniq5_siheung_dev`) | **IONIQ 5** |

두 계열은 차량 자체가 달라 DBC·CAN 시그널·`.msg` 정의·맵이 갈라진다. 브랜치 간 코드를 옮기거나 비교할 때 CAN/DBC·`.msg` 필드 차이는 버그가 아니라 **플랫폼 차이**일 수 있음. 공통 조상은 `main`(`cbe9d3c`).

**2026-08-07 리네임 (로컬·원격 모두, 구 원격 브랜치 삭제됨):**
- `ioniq5` → **`ioniq5_release`**
- `ioniq5_hmi_dev` → **`ioniq5_kcity_tested`** (K-City 검증 완료 스냅샷)
- 신규 `ioniq5_siheung_dev` = `ioniq5_release`(`9831727`)에서 분기한 IONIQ 5 개발 브랜치
- `ioniq5_hmi_dev` 의 88 커밋은 리네임 전 `ioniq5` 에 fast-forward 병합됨 → ioniq5 계열 3개 모두 `9831727` 동일

아래 문서 중 `ioniq5_hmi_dev` / `ioniq5` 로 적힌 것은 각각 `ioniq5_kcity_tested` / `ioniq5_release` 로 읽을 것.

## Branch별 지도/설정
- **ioniq5_siheung_dev** (현행): MAPFILE_PATH=`mapfiles/senario` (`mat_scenario` arg default, 2026-08-07 K_CITY_20260618→senario). web_hmi `map_shp`/`threejs_mapdir`=`src/shp_map/senario_shp_20260623`. 모두 `siheung_release` 와 동일 셋
- **ioniq5_kcity_tested** (구 `ioniq5_hmi_dev`, 2026-06 HMI 작업 브랜치): MAPFILE_PATH=`mapfiles/K_CITY_20260618` (katech_test.launch, 2026-06-18 갱신). `ioniq5_release` 도 병합 후 동일 내용
- **ioniq5_release** (구 `ioniq5`): 병합 전 기준 MAPFILE_PATH=`mapfiles/K_CITY_20251201` (K-City 지도)
- **siheung_dev**: MAPFILE_PATH=`mapfiles/$(arg scenario)` (senario1/senario3 선택), SHP_MAP_PATH=`src/shp_map/$(arg scenario)`
  - `scenario` arg: `senario1`(기본) 또는 `senario3`
  - senario3 mat: link_233, link_653, link_783 삭제됨 (2026-04 pull 반영)

## Build
- Build command: `source /opt/ros/noetic/setup.bash && catkin_make`
- 35+ ROS packages (sensing, localization, diagnostic, v2x, visualization, msgs)
- See [build-deps.md](build-deps.md) for details

## Key Packages
- **sensing/can**: CAN communication (Kvaser canlib + kvadblib), DBC-based signal encode/decode
- **localization/gps_system_localizer**: GPS-based localization, publishes to control team
- **diagnostic/**: System health monitoring (GPS, lidar, radar, camera, V2X, VCU, HMI, IPC)
  - vcu_diagnostic: VCU Info 6종 life_count(/sensors/v_can) staleness(0.5s) 추적, 하나라도 미갱신 시 VCU_StatCode=1 (2026-06-02)
- **v2x/siheung_v2x**: V2X communication (j3224_decode에서 리네임, KSR1600 추가)
  - **SPaT 활성 토픽은 브랜치마다 정반대 — 점검 전 `git branch` 확인** (2026-06-04):
    - `ioniq5_hmi_dev`: `/katri_v2x_node/katri_spat` (katri_obu_interface 활성, katech_test.launch:35). `/siheung_spat` 없음. 구독: spat_CAN_writer(→차량 CAN V2X_SPaT_1)/v2x_diagnostic/stat_display/pyqt 양빌드
    - `siheung_dev`: `/siheung_spat` (siheung_v2x_node), katri_v2x_node 주석처리
    - katri 경로: `~/katri_dsrc/decodeSample.c`가 SIG_SPAT[5]를 1024B UDP(:50000)로 송신→katri_obu_interface 수신, struct byte-identical(포맷 일치). `MovementStateName`은 발행만 되고 읽는 소비자 없음(spat_CAN_writer는 movementName_1 신호에 상수 0)
- **visualization/pyqt_hmi**: PyQt HMI (vehicle view, diagnostic, rosbag 녹화 등)
  - 빌드 2개: 기본 `main_display.py`→`widgets/main_window.py`, A-1 `main_display_a1.py`→`utils/hmi_state.py`+`widgets_a1/`. 진단/표시 로직 변경 시 둘 다 반영
  - 진단 status 색: 0=정상(green) 1=경고(orange) 2=에러(red). 규약: 토픽끊김→2, StatCode 도메인→1. 단 VCU는 StatCode==1(life_count 결손=device fault)을 error(2)로 표시 (2026-06-02)
  - 기본 빌드(main_window.py): 좌측 하단 "주행경로 ON/OFF" 토글(arc 표출 제어), shp 지도 경로는 rospkg(gps_system_localizer)로 해석(절대경로 하드코딩 제거) (2026-06-04)
  - **WSLg gotcha**: VehicleViewWidget.wheelEvent 줌은 `isActiveWindow()` 가드 필수. WSLg/XWayland는 wheel을 키보드 포커스가 아닌 커서 위치 기준으로 전달해, 다른 창 선택 후 스크롤해도 커서가 HMI 위면 줌이 먹었음 (2026-06-04). 클릭 라우팅 이상도 같은 WSLg 입력 특성으로 추정(코드상 grabMouse/eventFilter 없음)

## CAN 구성 (ioniq5 브랜치)
- 전체 AD CAN 노드 DBC v6 통일 (CANdb_IONIQ5_AD_CAN_v6.dbc, ioniq5_hmi_dev 2026-06-04 v5→v6). v6 = from_Control(512)에 arc_len/arc_kappa/arc_ds + AEB_flag/LC_flag, Mornitoring(513) obj 신호 추가
- ch0: chassis_CAN_reader + local/spat/ped_detector/diagnostic writers
  - chassis_CAN_reader: from_Control(512) arc_len/kappa/ds + AEB_flag/LC_flag 디코딩 → /sensors/ioniq5_ad_can. arc는 pyqt_hmi 주행경로 표출에 사용
- ch1: track_CAN_writer_no_grid (PCAN2)
- ch2: IONIQ5_CAN_reader (V_CAN_Release.dbc) → /sensors/v_can. 추가로 **/sensors/chassis(mmc_msgs/chassis_msg) 단독 owner 발행** (V_CAN 필드 합성 + /sensors/ioniq5_ad_can 구독으로 vcu_ADMDStatus=autonomous_mode/AEB/LC, 50Hz timer+mutex, 2026-06-04)
- ch3: DTG_CAN_writer (2gen-2ch-C_IoniqEV_v2.dbc)
- See [can_channel_map.md](can_channel_map.md) for full details

## Localization Details
- **통일 좌표계: EPSG:5179** (Korea 2000 / Unified CS)
- `to_control_team_demo.py`: .mat file based, Frenet coordinate, ODD 판정
  - **2026-08-07 (ioniq5_siheung_dev)**: K-City LINK_ID 하드코딩 분기 전면 제거 → mat 의 Speed_Limit·신호정보·is_stop_line 이 그대로 나간다. 구 `else: Speed_Limit = 30` 이 맵값을 덮어쓰던 버그도 함께 해소. `ioniq5_kcity_tested`/`ioniq5_release` 에는 아직 구 로직이 남아 있음
  - **로더도 siheung 방식**: `sorted(glob('link_*.mat'))` → `MAX_LANE_ID`/`road_{i}` 동적 속성 없음. `lane_id` 는 사전순 인덱스라 LINK_ID 와 무관(링크 417 → `road_210`/`lane_id 211`). 상세 [project_ioniq5_siheung_port.md](project_ioniq5_siheung_port.md)
  - 링크 매칭은 frenet min-|d|. 물리적 겹침 링크(78↔79, 78 끝 s≈27·36~37m)는 old_lane_id 기반 hysteresis로 조기전환 방지 (2026-06-02)
- `to_control_team_demo_shp.py`: .shp file based (siheung_dev)
- Key msgs: `localization2D_msg`, `to_control_team_from_local_msg`, `chassis_msg`

## Launch Files
- **katech_test.launch**: 메인 런치 (브랜치별 설정 다름 — Branch별 지도/설정 참조)
- **diagnostic_only.launch**: diagnostic 노드 + lanelet_marker + rviz_filter + model_publisher

## File Conventions
- C++ nodes in sensing/can use Kvaser canlib API + kvaDbLib for DBC parsing
- Python nodes use `#!/usr/bin/env python3.8`
- Map files: `.mat` (MATLAB) in mapfiles/ or `.shp` (Shapefile) in shp_map/
