# MCAR Project Memory Index

## Memory Files
- [build-deps.md](build-deps.md) — 빌드 의존성 (apt, Kvaser SDK, CMake 노트)
- [feedback_no_tmux_panes.md](feedback_no_tmux_panes.md) — ROS 에이전트팀 실행/종료 패턴 (백그라운드, 좀비 프로세스 정리)
- [can_channel_map.md](can_channel_map.md) — CAN 채널 할당표 (노드별 채널, DBC, R/W)
- [kvaser_driver_kernel.md](kvaser_driver_kernel.md) — Kvaser canlib 채널 0개 진단. 5.15.0-139가 정규 커널(GRUB/apt hold 고정, 2026-04-23)
- [wifi_ax211.md](wifi_ax211.md) — Intel AX211(51f1) Wi-Fi 복구: linux-modules-iwlwifi-*-generic 사용 (backport-dkms는 5.15 미지원)
- [project_ioniq5_work.md](project_ioniq5_work.md) — ioniq5 브랜치 2026-04 작업 이력

## Project Overview
- ROS Noetic catkin workspace for autonomous driving (KATECH)
- Location: `/home/katech/mcar/`
- Main branch: `main` (2026-01-07 이후 변경 없음), active branches: `ioniq5`, `siheung_dev`
- sudo password: `1`

## Branch별 지도/설정
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
- **v2x/siheung_v2x**: V2X communication (j3224_decode에서 리네임, KSR1600 추가)
- **visualization/pyqt_hmi**: PyQt HMI (vehicle view, diagnostic, rosbag 녹화 등)

## CAN 구성 (ioniq5 브랜치)
- 전체 AD CAN 노드 DBC v5 통일 (CANdb_IONIQ5_AD_CAN_v5.dbc)
- ch0: chassis_CAN_reader + local/spat/ped_detector/diagnostic writers
- ch1: track_CAN_writer_no_grid (PCAN2)
- ch2: IONIQ5_CAN_reader (V_CAN_Release.dbc)
- ch3: DTG_CAN_writer (2gen-2ch-C_IoniqEV_v2.dbc)
- See [can_channel_map.md](can_channel_map.md) for full details

## Localization Details
- **통일 좌표계: EPSG:5179** (Korea 2000 / Unified CS)
- `to_control_team_demo.py`: .mat file based, Frenet coordinate, ODD判定
- `to_control_team_demo_shp.py`: .shp file based (siheung_dev)
- Key msgs: `localization2D_msg`, `to_control_team_from_local_msg`, `chassis_msg`

## Launch Files
- **katech_test.launch**: 메인 런치 (브랜치별 설정 다름 — Branch별 지도/설정 참조)
- **diagnostic_only.launch**: diagnostic 노드 + lanelet_marker + rviz_filter + model_publisher

## File Conventions
- C++ nodes in sensing/can use Kvaser canlib API + kvaDbLib for DBC parsing
- Python nodes use `#!/usr/bin/env python3.8`
- Map files: `.mat` (MATLAB) in mapfiles/ or `.shp` (Shapefile) in shp_map/
