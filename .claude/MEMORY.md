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
- [feedback_wslg_wheel_input.md](feedback_wslg_wheel_input.md) — WSLg는 wheel/클릭을 커서 위치 기준 전달. PyQt wheelEvent에 isActiveWindow 가드 필요(다른 창 선택해도 줌 먹는 증상)
- [reference_diag_replay_harness.md](reference_diag_replay_harness.md) — ~/diag_replay_sample, CANoe mat(v7.3/h5py)→rosbag, vcu_diagnostic 검증(make_vcan_bag.py=구 v_can / make_adcan_bag.py=현행 AutonomousState) + 슈퍼바이저/종/횡 고장구간 라벨링
- [project_control_fault_not_displayed.md](project_control_fault_not_displayed.md) — 슈퍼바이저/횡/종 error_code 표출 경로. 2026-06-09 vcu_diagnostic을 AutonomousState(life_count staleness OR error_code≠0) 기반으로 교체 → VCU 슬롯 표출
- [project_v2x_spat_takeover_decoupled.md](project_v2x_spat_takeover_decoupled.md) — V2X SPaT↔TOR. AliveCount(=SPaT 수신 카운터) 경로로 SPaT 갭이 TOR 유발. 2026-06-09 stat_display 게이팅: 신호등 필요(look_at≠0)+SPaT 0.5s 미수신만 v2x_status=2→TOR, SPaT 없으면 주황. off-intersection false TOR 제거
- [project_blackbox_recorder.md](project_blackbox_recorder.md) — pyqt_hmi blackbox_recorder 노드. TOR 트리거 시 [이전10s,이후2s] 저장(LiDAR/인지 13토픽 제외). 경량 모니터+C++ rosbag record 롤링버퍼(Python 전토픽 버퍼링은 실차부하에 막힘). 합성 하네스 PASS, 실차 종단검증 권장
- [project_altitude_can_flow.md](project_altitude_can_flow.md) — 2026-06 hMSL→host_altitude→CAR_EGO_A_Ex.ALTITUDE(id1830, 32bit float byte4-7, DLC4→8). DBC v7 신설(파일별 분리 관례), writer+reader 둘 다 v7. local_CAN_writer는 dlc=8 고정 송신. 실차/CANoe rate·timing 미검증

## Project Overview
- ROS Noetic catkin workspace for autonomous driving (KATECH)
- Location: `/home/katech/mcar/`
- Main branch: `main` (2026-01-07 이후 변경 없음), active branches: `ioniq5_hmi_dev`(2026-06 현행), `ioniq5`, `siheung_dev`
- sudo password: `1`

## Branch별 지도/설정
- **ioniq5_hmi_dev** (2026-06 현행 HMI 작업 브랜치): MAPFILE_PATH=`mapfiles/K_CITY_20260608` (katech_test.launch)
- **ioniq5**: MAPFILE_PATH=`mapfiles/K_CITY_20251201` (K-City 지도)
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
- `to_control_team_demo.py`: .mat file based, Frenet coordinate, ODD判定
  - **gotcha**: mat의 Speed_Limit·신호정보·is_stop_line을 읽은 뒤 LINK_ID별 하드코딩 분기로 덮어씀 → mat만 바꿔선 안 바뀜. 속도제한 등은 코드 분기 먼저 확인 (else 기본 30, 2026-06-01)
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
