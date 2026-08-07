# mcar_v13 프로젝트 메모리

## 프로젝트 개요
- ROS (noetic) 기반 자율주행 시스템
- 경로: `/home/ads/mcar_v13`
- 메인 브랜치: `main`. **브랜치 계열 = 차량 플랫폼**: `siheung_*`(siheung_dev/siheung_release)=**아이오닉 EV**, `ioniq5_*`(ioniq5_release/ioniq5_kcity_tested/ioniq5_siheung_dev)=**IONIQ 5**. 상세 [project_build.md](project_build.md)
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
- 대상 파일: 이 디렉토리의 모든 `*.md` (`MEMORY.md` 포함). 메모리 파일 추가 시 자동 포함.

## 핵심 참조 사실 (durable — 나머지 변경 이력은 git log)
- **DBC 기어 매핑**: v_can `gear_status` 0=N/A,1=P,2=R,3=N,4=D (ioniq5 브랜치) / PCAN1 `Curr_gear`(MOTOR_RPM) 0=P,5=D,6=N,7=R (siheung_dev 브랜치).
- **v_can wheel_speed 단위 = m/s** (V_CAN_Release.dbc scale 0.01). pyqt 현재속도 = wheel_speed 4개 평균 × 3.6.
- **신호등 phase→color**: stat_display phase 3→빨강 / 8→주황 / 6→초록. hmi_state case 5(perm movement)→초록, 7(perm clearance)→주황 추가(기존 3/6/8만 인식 → 5/7 미표시 버그).
- **GPS 소스**: 메인은 Novatel(`/sensors/gps/novatel|bestpos|inspva`). ublox `/ublox/navpvt`(carrSoln)는 일부 진단·web_hmi 보조. carrSoln=`(flags>>6)&0x03` (0=No RTK,1=Float,2=Fixed).
- **ublox 설정**: `zed_f9k.yaml` tmode3=0(로버), config_on_startup=false. `node.cpp` rtcm 구독 제거(장치 자체 RTCM 수신).
- **to_control_team_demo.py 완화 임계**: ODD_YAW_ERR 30°, ODD_OCCUPIED_OFFSET 2.0m.
- **perception_ros_msg 오타 필드(포팅 시 그대로 유지)**: `CoreInfo.trakcer_id`, `Object.hassupplmentinfo`, `SupplementInfo.cloud_indices[k].data`(Int32 객체 배열).
- **diagnostic_only.launch**: `lateral_offset_relay.py`/`gps_std_relay.py` = rviz overlay text 전용(web_hmi 무관), `ioniq_statdisplay.rviz` 가 구독하므로 유지 필요. base2ego TF frame_id 는 `/ego_frame`(슬래시 필수, rviz 매칭).
- **bag 검증 함정**: `/hmi/state`/`/hmi/*` 녹화 bag 재생 시 live 브리지(web_hmi_bridge/qt bridge) 동시 기동 금지 → dual-publisher 진동. roscore + bag + 뷰어 노드만. bag 재생 시 `/hmi/map`·`/hmi/threejs/map` 을 `/dev/null/*` 로 remap.
- **SPaT 활성 토픽은 브랜치마다 정반대** — siheung_dev `/siheung_spat`(→ 병합 `/spat_merged`) vs ioniq5_kcity_tested(구 `ioniq5_hmi_dev`) `/katri_v2x_node/katri_spat`. 점검 전 `git branch` 확인.
- **crosswalk 좌표/CAN/게이팅**: katech_ped_detector.py `crosswalk_data`=EPSG:5179 폴리곤(오프라인 pyproj 4326→5179 리터럴). 검출 `/katech_msg/crosswalk_detection` → `katech_ped_detector_can_writer` CAN `Pedestrian_Stat`(528/529) `on_crosswalk=1`(crosswalk_id 없음·객체 4 cap). **검출을 ego LINK_ID(`CW_LINKS`, detector cpp·py·fusion 공유)로 게이팅 → 접근 크로스워크만**. occupancy_msg `uint8[] occupied_ids` 추가, fusion active(own=occupied_ids, obu #1·#2만). **크로스워크 1~18**(2026-07-27, 10~18 은 md 링크표 없어 mat `is_stop_line`+선행 20m 규칙으로 도출; #15/#16 만 접근링크 661/662 공유). web_hmi CrosswalkZones=[E,N] 1~18 점멸, mat 뷰어 var CROSSWALK=[lat,lon]. **CAN writer 는 자체 검출 외에 `/katech_msg/crosswalk_ped_fusion` 추가 구독 → RSU present(source 2/3) 시 `on_crosswalk_1 = max(자체, RSU)` 로 OR(2s staleness). 자체=0 이어도 RSU=1 이면 CAN on_crosswalk=1 (2026-08-05).**
- **perception ObjectType(RS)**: enum `0=UNKNOW,1=CONE,2=PED,3=BIC,4=CAR,5=TRUCK_BUS,6=ULTRA_VEHICLE`. `/track_Multi_RS` object_msg.**status = class(type)**(트래킹상태 아님, `lidar_object_publisher_v2.py:282`). ⚠️ **실데이터에선 type3(BIC) 이 ≈5m 차량크기** → {2,3}=보행자로 두면 web_hmi 가 차량을 사람으로 표시. 그래서 **차량/사람 분류는 이전버전 `type==1`→보행자 로 원복(2026-07-21)**. 상세 [perception ObjectType](perception_object_type.md).
- **객체 속도 vx,vy = ego-상대**(perception object_msg, flat 구조). 절대 이동방향 = `R(host_yaw)·(vx,vy) + v_ego_map`(ego 가산 필수; to_control_team 에 host 속도 없어 위치 유한차분). 횡단보도 보행자 검출(on_crosswalk)은 C++ `katech_ped_detector.cpp`(Python 대체, katech_test.launch line57) — 게이트 **타입{1,2}(이전버전)→멤버십(LINK)→방향**(크로스워크 PCA 길이축 사이각≤40°, 정지 스킵). **크기 게이트는 제거됨**(사용자: 진행방향만 고려). 상세 [ped-detector-cpp](ped_detector_cpp.md).

## Diagnostic 구조
| 토픽 | 메시지 타입 | 소스 노드 | 판단 기준 |
|------|-------------|-----------|-----------|
| `/diagnostic/vcu` | `vcu_diagnostic_msg` | vcu_diagnostic | V_CAN GearInfo life_count |
| `/diagnostic/adcu` | `k_adcu_diagnostic_msg` | chassis_CAN_reader | AD_CAN BrainState life_count |
| `/diagnostic/cpt7_gps` | `cpt7_gps_diagnostic_msg` | cpt7_gps_diagnostic | **Novatel** `/sensors/gps/bestpos`·`/inspva` (ublox 아님) |
- 공통 패턴: 콜백에서 msg_received 플래그 설정, 타이머에서 플래그 확인 후 리셋
- GPSRTK_StatCode: **0x38(INS_RTKFIXED) 또는 0** 두 값뿐 (carrSoln 아님). GPS_INS_SolutionStat: `solution_status=="SOL_COMPUTED"`→0x00, else 0x01 (fixType 아님)
- stat_display GPS 색상 / 경고 기준 상세 → [gps_warning_criteria.md](gps_warning_criteria.md) (빨강은 GPS 아닌 ping 8.8.8.8 실패)

## 추가 메모리 파일 (작업 이력 상세 = 각 토픽 파일)
- [Project Build & Branch Status](project_build.md) — 빌드 경로, **브랜치 계열=차량 플랫폼(siheung_*=아이오닉 EV / ioniq5_*=IONIQ 5)**, 2026-08-07 리네임 이력 / 브랜치 전환 직후 첫 빌드 msg 헤더 경합 실패(재빌드로 해소)
- [User Communication Style](user_style.md) — 한국어 짧은 명령 선호, 간결 응답
- [siheung_dev 활성 맵](siheung_map_senario3.md) — shp_map 루트 두 .shp(POLYLINEZ 1770 + POLYGONZ 372), EPSG:32652→5179 변환
- [web-hmi-adapt harness](web_hmi_adapt_harness.md) — 브랜치 간 web_hmi 어댑트 파이프라인. chassis_msg Curr_gear 필드+HMI 매핑(0=P→1,5=D→4,6=N→3,7=R→2), V2X 미수신 threshold 3s/StatCode≥2 WARN
- [qt_hmi + web_hmi rviz-grade follow](project_qt_hmi.md) — 50Hz ego pose 패턴(둘 다); ego-frame 트랙은 emit-time ego 스냅샷(`ego_at_emit`)과 페어링 필수
- [bridge cpp 포팅 패턴](project_bridge_cpp_port.md) — Python `/hmi/threejs/tracks` 0.78→9.98 Hz. zero-copy PointCloud2 + dual-publisher 회피(`~publish_tracks` 가드). 6-cap 제거 후 상한은 percept 14-cap
- [percept 오브젝트 필터 정책](percept_filter_policy.md) — percept_topic_matcher.cpp+rviz_filter.cpp 3-zone OR(박스 `|y|≤5`/전·후방 좌측면), 우측컷, 14개 cap. 두 파일 ROI 동시 갱신
- [lanelet fiona geometry 함정](lanelet_fiona_geometry.md) — POINTZ `'float' not subscriptable` 크래시, A2_LINK 전용화, geom_type별 coordinates 구조 차이
- [senario-gps-pub harness](senario_gps_pub_harness.md) — senario HTML 주행 link 시퀀스 → 40 km/h GPS 시뮬레이션 publisher 신규 개발 파이프라인
- [spat-merge OBU+MQTT](spat_merge_obu_mqtt.md) — `spat_merge_node` 교차로(IID) 단위 OBU 우선 병합(/spat_merged). MQTT-only 교차로 302 를 HMI/CAN 에 전달. 링크 417→IID 302/SG 70/MANUAVER -1(LEFT)
- [SPaT 방향 매칭](spat_dir_match.md) — MANUAVER -1/0/1 ↔ MovementStateName(LEFT/{STR,STRAIGHT}/RIGHT). HMI 방향필터 부재로 병합 알파벳순 LEFT 오선택 버그. spat_CAN_writer/hmi_state.py/stat_display 수정, 하네스 spat-dir-match
- [spat_viewer 실행](spat_viewer_run.md) — MQTT/OBU SPaT ① 라이브 Leaflet 뷰어(`roslaunch spat_viewer spat_viewer.launch` + `:8080/`) ② offline bag 리플레이 뷰어(`extract_spat_replay.py`→`replay.html`, ego 방향매칭 신호등 재생/스크럽)
- [tim-pedes bag 재생](reference_tim_pedes_bag_replay.md) — `web_hmi_replay.launch`(bag→web_hmi threejs_f1, /hmi/* remap 토글). 구 bag 으론 퓨전 own 경로 검증 불가(occupancy/track_Multi_RS 없음 → OBU 경로만 간헐)
- [A2_LINK shp 구조](reference_a2_link_shp.md) — senario_shp_20260623 A2_LINK, UTM52N(32652)→5179, 좌/중/우 3차선(중앙=R·L 양쪽), ITSLinkID 1:N, ToNode→FromNode 위상. web_hmi 지도/경로 원천
- [mat 정지선 연장/트림](stopline_adj.md) — senario mat 링크를 B2_SURFACELINEMARK 정지선까지 연장 + 다음 링크 트림(L=L+N[1:k+1]+P / N=P+N[k+1:], station 재계산, 공유정점). mat=5179/shp=32652. 뷰어 var DATA 6 feature 패치. 하네스 stopline-adj
- [global-nav-hmi 경로표시](project_global_nav_hmi.md) — web_hmi 주행 예정 경로 중앙차선 리본 + TIM(`on_block_link&&do_not_go_forward`) 트리거 old→new latch 전환. 오프라인 route JSON. 795117 절단(후반 ITS 미구현)
- [crosswalk-position 횡단보도 좌표](crosswalk_position.md) — WGS84 폴리곤 → detector(cpp+py) EPSG:5179 crosswalk_data(오프라인 pyproj 리터럴) + mat 뷰어 var CROSSWALK([lat,lon] swap) + CrosswalkZones.jsx([E,N]). **1~18**, CW_LINKS 게이팅, 10~18 링크 도출규칙(is_stop_line+선행 20m), OBU 는 #1/#2 전용. 하네스 crosswalk-position
- [perception ObjectType](perception_object_type.md) — RS perception class enum `1=CONE,2=PED,3=BIC,4=CAR,5=TRUCK_BUS,6=ULTRA`. object_msg.status=class(type). 코드 1=보행자 오분류 5곳 정정(PED+BIC 보행자), main_window.py 대기
- [ped-detector-cpp 검출 포팅](ped_detector_cpp.md) — Python on_crosswalk 검출→C++(`katech_ped_detector.cpp`) + 크기(≤2m)·방향(PCA 길이축 vs 절대속도) 게이트, 순서 타입→크기→멤버십→방향. 객체 vx,vy=ego-상대(절대=R(yaw)·v+v_ego). 하네스 ped-detector-cpp
- [GPS 경고 판정 기준](gps_warning_criteria.md) — rviz `/rviz/jsk/gps_stat` 색: 주황=`StatCode!=0x38 || std>5cm` / 빨강=`Network_Status`(ping 8.8.8.8 실패, GPS 무관). AliveCnt 검사 무력화(GPS 끊기면 색 동결), rviz `==0x38` vs HMI `<2` 판정 불일치, GPS_Over 죽은 필드(2026-07-27 트리거 제거, msg/CAN 유지)

## 피드백 메모리
- [일본어 사용 금지](feedback_no_japanese.md) — 응답에 일본어(한자) 금지, 한국어만 사용
- [존댓말만 사용](feedback_honorific.md) — 응답 종결은 "~합니다/입니다" 존댓말로 통일, 반말 금지
- [web_hmi 어댑트 함정](web_hmi_adapt_pitfalls.md) — LAYER_STYLE 동기화 누락 / polyline `alpha` 무시(buildPolyline 패치로 지원, A2_LINK 0.2) / bag /hmi/* 충돌 (ROS만 패치하면 화면 안 나옴)
- [CAN 완전 동결 additive](feedback_can_frozen_additive.md) — 차량 CAN-feeding 토픽은 수정 말고 additive 신규 토픽으로 우회(무변경 증명 git diff ∅/sha256). tim-pedes-display 에서 /katech_msg/crosswalk_occupancy 신설
