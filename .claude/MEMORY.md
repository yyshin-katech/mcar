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
- 대상 파일: `MEMORY.md`, `can_package.md`, `project_build.md`, `user_style.md`, `siheung_map_senario3.md`, `web_hmi_adapt_harness.md`, `web_hmi_adapt_pitfalls.md`, `feedback_no_japanese.md` (메모리 파일 추가 시 갱신)

## 최근 작업 이력
- DBC 파일을 `CANdb_IONIQev_PCAN1.dbc` → `CANdb_IONIQ5_AD_CAN_v3.dbc`로 변경 (7개 src 파일)
- `IONIQ_CAN_reader.cpp`: `V_CAN_Release.dbc` 사용, CAN FD 지원 (500K/1M), 14개 전체 메시지 수신
- `CHASSIS_CAN_READER`: 기존 VCU 메시지 → IONIQ5 AD CAN 메시지로 교체
- launch에서 `vision_CAN_reader`, `front_RADAR_CAN_reader` 제거
- `vcu_diagnostic`: `/sensors/v_can` 구독, GearInfo `life_count`로 VCU 상태 체크
- `chassis_CAN_reader`: BrainState `life_count`로 ADCU diagnostic 추가 (`/diagnostic/adcu` 퍼블리시)
- launch: `chassis_CAN_reader`, `IONIQ_CAN_reader` 활성화
- `cpt7_gps_diagnostic`: Novatel → ublox NavPVT(`/ublox/navpvt`) 구독으로 변경
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

## 피드백 메모리
- [일본어 사용 금지](feedback_no_japanese.md) — 응답에 일본어(한자) 금지, 한국어만 사용
- [web_hmi 어댑트 함정](web_hmi_adapt_pitfalls.md) — LAYER_STYLE 동기화 누락 / bag /hmi/* 충돌 (ROS만 패치하면 화면 안 나옴)
