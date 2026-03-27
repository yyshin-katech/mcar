# MCAR Project Memory

## Project Overview
- ROS Noetic catkin workspace for autonomous driving (KATECH)
- Location: `/home/katech/mcar/`
- Main branch: `main`, active branches: `ioniq5`, `siheung_dev`, `dev`, `katech_aeb`, `koras`
- sudo password: `1`

## Build
- Build command: `source /opt/ros/noetic/setup.bash && catkin_make`
- 35 ROS packages (sensing, localization, diagnostic, v2x, visualization, msgs)
- Dependencies installed: `ros-noetic-nmea-msgs`, `ros-noetic-jsk-rviz-plugins`, `kvlibsdk` (Kvaser SDK for kvaDbLib)
- See [build-deps.md](build-deps.md) for details

## Key Packages
- **sensing/can**: CAN communication (Kvaser canlib + kvadblib), DBC-based signal encode/decode
- **localization/gps_system_localizer**: GPS-based localization, publishes to control team
- **diagnostic/**: System health monitoring (GPS, lidar, radar, camera, V2X, VCU, HMI, IPC)
- **v2x/**: V2X communication (J3224 SDSM, KATRI OBU interface)

## Localization Details
- **통일 좌표계: EPSG:5179** (Korea 2000 / Unified CS) — 모든 노드가 동일 좌표계 사용
- `to_control_team_demo.py`: .mat file based, Frenet coordinate, ODD判定
  - mapfiles 디렉토리: senario1 (248 links), senario3 (242 links) 등 시나리오별 mat 파일
- `to_control_team_demo_shp.py`: .shp file based, Frenet coordinate, ODD判定
  - 현재 SHP: `shp_map/senario1/TB_senario_map_senario1.shp` (248 links)
  - SHP 원본 CRS: EPSG:32652 → 로드 시 pyproj로 EPSG:5179 변환
  - `densify_coords`: SHP 웨이포인트를 2m 간격으로 보간 (SHP가 mat보다 포인트 적어 매칭 실패 방지)
  - Attributes: ID, MaxSpeed, LaneNo, R_LinkID, L_LinkID, FromNodeID, ToNodeID, LinkType, Length, TO_LinkID, mc_MANEUVE, mc_SIG_GR, mc_INT_ID, is_stop_ln, have_to_le, have_to_ri
  - mc_SIG_GR → look_at_signalGroupID, mc_INT_ID → look_at_IntersectionID, is_stop_ln → is_stop_line
  - have_to_le → have_to_LangeChange_left, have_to_ri → have_to_LangeChange_right
  - TO_LinkID: 콤마 구분 다음 링크 ID 목록 (복수 시 다음 링크의 mc_MANEUVE 매칭으로 선택)
  - Uses cython functions: `compute_current_lane`, `xy2frenet_with_closest_waypoint`
  - dist threshold: 3.0m
- `test_loop_publisher.py`: 루프 경로 pose 퍼블리시 테스트 노드 (30km/h, 20Hz, SHP 기반 루프)
- `test_senario1_publisher.py`: senario1 mat 기반 1회 주행 테스트 (74링크 3779m, 북→동남, 갭 순간이동)
- `ublox_navpvt_to_pose2d.py`: /ublox/navpvt → localization2D_msg 변환 (EPSG:4326→5179, pyproj)
- `lanelet_marker.py`: shp_map/ 및 하위 디렉토리 내 모든 SHP 로드 (LineString + Polygon 지원), /rviz/lanelet_marker 퍼블리시
- `gps_world_tf_node_send_host_gps.cpp`: /ublox/fix → localization2D_msg (EPSG:5179, libproj)
- Key msgs: `localization2D_msg`, `to_control_team_from_local_msg` (LINK_ID/NEXT_LINK_ID/lane_id → int32로 변경), `chassis_msg`

## Launch Files
- **katech_test.launch**: 메인 런치 (siheung_dev)
  - tester_for_usb_eq 주석처리 (ublox GPS 직접 사용)
  - `gps_world_tf_node_send_host_gps` → `/localization/pose_2d_gps` (EPSG:5179)
  - `to_control_team_demo_shp.py` → `/localization/to_control_team`
- **test_shp_bag.launch**: bag 테스트용 (ublox_navpvt_to_pose2d + to_control_team_demo_shp)

## SHP 테스트 파이프라인 (siheung_dev)
- bag: `~/senario3-1.bag` (58s, /ublox/navpvt @40Hz)
- bag 재생: `rosbag play ~/senario3-1.bag --topics /ublox/navpvt -r 1.0 -l`
- sensing/gps/ublox: ioniq5 브랜치에서 가져옴 (ublox_gps, ublox_msgs 등 빌드 완료)
- `shapefile` 모듈 미설치 → fiona 사용으로 대체

## File Conventions
- C++ nodes in sensing/can use Kvaser canlib API + kvaDbLib for DBC parsing
- Python nodes use `#!/usr/bin/env python3.8`
- Map files: `.mat` (MATLAB) in mapfiles/ or `.shp` (Shapefile) in shp_map/
