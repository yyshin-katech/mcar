# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Full workspace build
cd /home/ads/mcar_v13 && catkin_make

# Single package build
catkin_make --pkg <package_name>

# Source workspace
source devel/setup.bash

# Main launch files
roslaunch launch/katech_test.launch        # Primary: GPS, CAN, localization, V2X
roslaunch launch/diagnostic_only.launch    # Diagnostics, HMI, stat_display, rviz
```

## Architecture

ROS Noetic 자율주행 시스템 (IONIQ 5). 주요 데이터 흐름:

```
GPS(ublox) → localization → to_control_team → CAN writers → vehicle
CAN readers → /sensors/* topics → diagnostic nodes → /diagnostic/* → stat_display/pyqt_hmi
```

### Package Groups

- **sensing/can/**: Kvaser canlib/kvaDbLib 기반 CAN 통신. DBC 파일(`dbc/` 디렉토리)로 메시지 매핑. C++ 21개 실행파일
- **sensing/gps/ublox/**: u-blox ZED-F9K GPS 드라이버. config: `config/zed_f9k.yaml`, device: `/dev/ttyACM0`
- **localization/gps_system_localizer/**: 핵심 경로 계획 (`to_control_team_demo.py`). Shapefile(EPSG:5179) 맵, ODD 판단
- **diagnostic/**: 8개 진단 노드. 공통 패턴: 콜백에서 msg_received 플래그 설정, 타이머에서 확인 후 리셋
- **visualization/pyqt_hmi/**: PyQt5 GUI. pyqtSignal로 ROS 콜백 스레드 안전 처리
- **visualization/stat_display/**: C++ + Boost GIL 이미지 오버레이
- **msgs/**: katech_custom_msgs(차량 CAN), katech_diagnostic_msgs(진단), mmc_msgs(제어/인지), v2x_msgs

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/v_can` | `v_can_msg` | VCU CAN (steering, wheel speed, gear) |
| `/sensors/ioniq5_ad_can` | `ioniq5_ad_can_msg` | AD CAN (autonomous mode, brain state) |
| `/localization/to_control_team` | `to_control_team_from_local_msg` | 경로/ODD/속도제한 |
| `/diagnostic/*` | `*_diagnostic_msg` | 센서별 진단 상태 |
| `/ublox/navpvt` | `NavPVT` | GPS 위치/RTK 상태 (carrSoln: 0=No RTK, 1=Float, 2=Fixed) |
| `/rviz/lanelet_marker` | `MarkerArray` | 차선 시각화 (frame: gps) |

### DBC Files

- `CANdb_IONIQ5_AD_CAN_v4.dbc` — AD CAN (chassis_CAN_reader)
- `V_CAN_Release.dbc` — VCU CAN (IONIQ_CAN_reader)

### TF Frames

`/base_link` → `/ego_frame` (frame_id에 슬래시 필수, rviz 매칭용)

## Conventions

- 사용자 언어: 한국어
- 응답에 일본어(한자) 사용 금지. 반드시 한국어로만 응답할 것
- 좌표계: EPSG:5179 (Korean TM)
- GPS RTK 상태: carrSoln 기반 (`(flags >> 6) & 0x03`)
- CAN: Kvaser canlib (not SocketCAN)

## Memory Sync (필수)

로컬 메모리와 git 메모리를 항상 최신 상태로 유지해야 한다.

- **로컬 메모리 (원본)**: `/home/ads/.claude/projects/-home-ads-mcar-v13/memory/`
- **git 메모리 (동기화 대상)**: `/home/ads/mcar_v13/.claude/`
- **대상 파일**: `MEMORY.md`, `can_package.md` (메모리 파일 추가 시 목록 갱신)

### 커밋/푸시 시

커밋 전 반드시 로컬 → git으로 최신 메모리 복사 후 함께 커밋:

```bash
cp /home/ads/.claude/projects/-home-ads-mcar-v13/memory/MEMORY.md /home/ads/mcar_v13/.claude/
cp /home/ads/.claude/projects/-home-ads-mcar-v13/memory/can_package.md /home/ads/mcar_v13/.claude/
git add .claude/MEMORY.md .claude/can_package.md
```

### 새 대화 시작 시

git 메모리가 로컬보다 최신일 수 있으므로 (다른 환경에서 커밋된 경우) 비교 후 최신 버전으로 동기화:

```bash
# 날짜 비교하여 최신 파일로 동기화
diff /home/ads/.claude/projects/-home-ads-mcar-v13/memory/MEMORY.md /home/ads/mcar_v13/.claude/MEMORY.md
```

- git 쪽이 최신이면: git → 로컬로 복사
- 로컬이 최신이면: 유지 (커밋 시 git에 반영)
- 양쪽 모두 변경되었으면: 내용을 병합

## 하네스: web-hmi-adapt

**목표:** ioniq5_hmi_dev 기준으로 작성된 web_hmi를 현재 브랜치(siheung_dev 등)의 토픽/메시지/launch/맵 구조에 맞춰 어댑트.

**트리거:** "web_hmi 데이터 매칭", "web_hmi 어댑트", "브랜치 매칭", "다시 어댑트" 요청 시 `web-hmi-adapt` 스킬 사용. 단순 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-07 | 초기 구성 | agents 3 (match-detective/bridge-adapter/adapt-verifier) + skills/web-hmi-adapt | siheung_dev에서 web_hmi 데이터 매칭 요청 |

## 하네스: percept-filter-tune

**목표:** `src/sensing/can/src/percept_topic_matcher.cpp` 의 perception 오브젝트 필터 정책(우선순위·개수 cap) 튜닝.

**트리거:** "perception 필터", "오브젝트 필터", "percept_topic_matcher 수정", "object_filter.md 작업" 요청 시 `percept-filter-tune` 스킬 사용. 단순 코드 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-11 | 초기 구성 | agents 3 (percept-filter-analyst/coder/verifier) + skills/percept-filter-tune | `claude_work_list/object_filter.md` 사양 (전방 100m 우선 → 좌/우 차순 → 14개 cap) 적용 요청 |

## 하네스: bridge-cpp-port

**목표:** `src/visualization/web_hmi/scripts/web_hmi_threejs_bridge.py` 의 트랙 발행 경로(`/hmi/threejs/tracks`)를 동등 기능의 C++ ROS 노드로 포팅하여 콜백 지연 (Python 0.95 Hz → 목표 10 Hz) 을 해소.

**트리거:** "브리지 cpp 포팅", "브릿지 파이썬을 cpp 로", "web_hmi_threejs_bridge cpp", "트랙 박스 1초 점프 해결" 요청 시 `bridge-cpp-port` 스킬 사용. 단순 코드 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-12 | 초기 구성 | agents 3 (bridge-port-analyst/coder/verifier) + skills/bridge-cpp-port | `/hmi/threejs/tracks` 가 콜백 지연으로 0.95 Hz 까지 떨어져 web_hmi 트랙 박스가 1초마다 점프 — Python → C++ 포팅 요청 |

## 하네스: senario-gps-pub

**목표:** `mapfiles/senario/mat_viewer_senario_*.html` 주행 link 시퀀스를 따라 EPSG:5179 GPS 데이터를 40 km/h 일정 + 간단 동역학으로 시뮬레이션 발행하는 ROS publisher 신규 개발. 기존 `test_senario3_publisher.py` 와 동일 토픽/메시지 (`/localization/pose_2d_gps`, `mmc_msgs/localization2D_msg`) 패턴.

**트리거:** "senario gps publisher 만들어", "mat 시나리오 시뮬", "senario_260514c 주행 시뮬", "40km/h GPS 시뮬", "senario gps pub 다시 만들어", "route 바꿔서 다시" 요청 시 `senario-gps-pub` 스킬 사용. 단순 코드 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-15 | 초기 구성 | agents 3 (senario-sim-analyst/coder/verifier) + skills/senario-gps-pub | senario_260514c HTML 주행 시퀀스 기반 40 km/h GPS 시뮬레이터 publisher 신규 개발 요청 |

## 하네스: v2x-signal-verify

**목표:** `mapfiles/senario/` mat 의 신호등 매핑 ↔ `siheung_v2x` SPaT 디코딩(`/katri_v2x_node/katri_spat` 등) ↔ `to_control_team_demo.py` 신호등 매칭 로직의 정합성을 코드 변경 없이 정적 검증. 라이브 ROS 토픽 echo 는 SKIP (사용자 정책).

**트리거:** "v2x 신호등 검증", "senario 신호등 매칭 확인", "SPaT 파싱 검증", "신호등 디코딩 흐름 점검" 요청 시 `v2x-signal-verify` 스킬 사용. 패치가 필요한 경우 별도 하네스/직접 수정으로 위임. 단순 코드 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-18 | 초기 구성 | agents 2 (v2x-signal-analyst/verifier) + skills/v2x-signal-verify | senario 경로 mat 적용 시 v2x SPaT 파싱·매칭 정합성 검증 요청 |

## 하네스: protocol-spec-check

**목표:** `~/protocol/[ITSK-00150-2]V2N규격-제2부 V2N정보연계-경찰청연구과제규격.hwpx` 의 MQTT 인터페이스 규격을 추출하여 `siheung_v2x` 의 `mqtt_spat_rx_node` / `mqtt_bsm_tx_node` / `launch/siheung.launch` 가 규격에 정합한지 코드 변경 없이 정적 검증. 라이브 ROS 토픽 echo SKIP.

**트리거:** "프로토콜 규격 확인", "V2N 규격 검증", "hwpx 규격 비교", "MQTT 인터페이스 정합성 확인", "프로토콜 문서랑 코드 비교" 요청 시 `protocol-spec-check` 스킬 사용. 패치 필요 시 별도 하네스/직접 수정으로 위임. 단순 코드 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-23 | 초기 구성 | agents 2 (protocol-spec-analyst/verifier) + skills/protocol-spec-check | 경찰청 V2N 정보연계 hwpx 규격 ↔ 현재 MQTT 구현 정합성 검증 요청 |

## 하네스: mqtt-vpn-setup

**목표:** MQTT V2N 인터페이스 트래픽 (prod 브로커 `192.168.255.173:10044`) 만 시흥시 VPN 터널 로 보내고, 그 외 인터넷·NTRIP·로컬 LAN 트래픽은 default route 를 유지하는 split-tunnel 을 외과적으로 구성·검증. 실 환경 VPN 은 **시흥시 시범운행 인프라의 SecuwaySSL (Secuwiz)** — `~/sslvpn/SecuwaySSLU_client` + `conf/client.info` (`27.101.133.111:443`). OpenVPN 가정의 초기 사양서는 무시.

**트리거:** "vpn 설정", "mqtt vpn", "split-tunnel", "시흥시 vpn", "mqtt 인터페이스 vpn", "vpn 다시 설정", "vpn 재구성", "secuwayssl", "openvpn 분리 터널" 요청 시 `mqtt-vpn-setup` 스킬 사용. 단순 ROS/launch 질문이나 일반 네트워크 진단은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-26 | 초기 구성 | agents 3 (vpn-net-analyst/configurator/verifier) + skills/mqtt-vpn-setup | 시흥시 VPN 게이트웨이 통해 prod MQTT 브로커만 분리 터널 구성 요청 (자격증명 시스템 경로 보관, git 평문 금지) |
| 2026-05-26 | 실 환경 검증 + 사실 보정 | memory/mqtt_vpn_setup_harness.md + 본 항목 | 실 VPN 은 OpenVPN 아닌 SecuwaySSL (Secuwiz) 로 판명. PDF 가이드 + `~/sslvpn/SecuwaySSLU_client` 로 27.101.133.111:443 접속, tun0=172.18.113.51 동적 할당. split push 자동 → 핵심 2조건 통과. prod MQTT → IID 517 (시화) SPaT 5 Hz 수신·디코드 검증 (signalGroup 60/80, phase GO). `publishSpat` 가 `/localization/to_control_team` 필터 사용 — 시연 시 dummy publish 필요 |
| 2026-05-26 | 다른 PC 셋업 산출물 추가 | tools/sslvpn/{SSU21-2.1.0.2-20230331.tgz, README.md} | 다른 PC 에서도 PDF 없이 동일 절차로 VPN 셋업·접속 가능하도록 클라이언트 tgz 와 설치/검증/트러블슈팅 가이드를 git 추적 |
| 2026-05-26 | BSM 송신 검증 추가 | memory/mqtt_vpn_setup_harness.md + 본 항목 | `~/bag/20260508/*_0.bag` 리플레이 → `mqtt_bsm_tx_node` (prod 인자) → prod 브로커 `V2N/1321103202/bsm` 토픽 10 Hz 송신 확인. V2N header `04 00 ff 10` (BSM fid=FF10), PSID `00 01 40 82`, inner 40 byte J2735 BSM UPER 정합. v_can 미포함 bag 이라 steering/accel/yaw/brake 는 0 으로 채워짐. `use_sim_time=true` + `--clock` 미사용 시 timer 동결 함정 기록 |
| 2026-05-26 | VPN 명칭 정정 (카네비 → 시흥시) | CLAUDE.md, MEMORY.md, memory/*, .claude/{agents,skills,mqtt_vpn_setup_harness.md}, tools/sslvpn/README.md | "kanavi VPN" 으로 부르던 것은 실제로 시흥시 시범운행 인프라의 VPN. 표기 통일을 위해 모든 문서·에이전트·스킬에서 "kanavi" → "siheung" (파일명 `kanavi-mqtt.conf` → `siheung-mqtt.conf`, `tun-kanavi` → `tun-siheung` 등) 정정. ID 마스킹 `kana***1` 은 실제 계정 패턴이므로 유지 |
| 2026-05-26 | launch 자격증명 평문 정책 완화 + prod 기본화 | launch/siheung.launch, memory/mqtt_vpn_setup_harness.md | 다른 PC 셋업 편의를 위해 시흥시 VPN ID/PW 와 서버 IP/포트를 `launch/siheung.launch` arg 로 평문 포함 (`vpn_id`/`vpn_pass`/`vpn_server_ip`/`vpn_server_port`). `mqtt_server` default 를 `test` → `prod` 로 변경 (인터넷 테스트 시만 명시 override). 메모리 자격증명 보안 섹션을 launch 예외 명시로 갱신 |

## 하네스: spat-viewer-build

**목표:** VPN 통해 prod 브로커 (`192.168.255.173:10044`) 에서 들어오는 MQTT SPaT (`/siheung_v2x/mqtt_spat`) + OBU SPaT (`/siheung_spat`) 를 라이브로 보여주는 self-contained Leaflet HTML 뷰어 신규 개발. `mat_viewer_senario_*` 패턴으로 지도 위에 shp_map 도로 link, 교차로별 신호등 색/잔여시간, ego 마커(real-time), target intersection 강조. 메인 launch (`siheung.launch`) 와 동시 실행 가능 (별도 rosbridge_websocket).

**트리거:** "spat 뷰어 만들어", "신호등 뷰어", "spat 시각화", "MQTT spat 보는 페이지", "vpn spat 확인 페이지", "뷰어 다시 만들어", "spat-viewer", "교차로 색 잘못 나옴" 요청 시 `spat-viewer-build` 스킬 사용. 단순 코드 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-26 | 초기 구성 | agents 3 (spat-viewer-{analyst,coder,verifier}) + skills/spat-viewer-build | OBU>MQTT fallback 정책 적용 후 (commit 80624de — spat_CAN_writer 에서 ego 매칭, 발행자측 필터 제거) 라이브 SPaT 흐름을 메인 시스템 실행 중에도 브라우저로 확인할 수 있는 뷰어 요청 |
| 2026-05-26 | 라이브 검증 + 데이터 소스 mat 재설계 | src/visualization/spat_viewer/* + memory/spat_viewer_build_harness.md | VPN+mqtt_spat_rx_node+roslaunch spat_viewer 라이브 검증 (브로커 IID 매치 13/15). 초기 shp 추출 (HDMap_Oido_New C1_TRAFFICLIGHT.shp, 2544 feature) 을 `mapfiles/senario/link_*.mat` 직접 추출로 교체 — mat 이 IID/SG/stop_line/MANUAVER 키를 직접 carry. 결과 308 link / 15 IID `[134,136,165,168,201,203,302,504,507,508,509,516,517,518,519]`. is_stop_line=1 링크 `#fab387` 강조 + legend 갱신. extract_map_data.py 는 pyproj 만 의존 (pyshp 제거) |

## 하네스: hmi-block-display

**목표:** 차량이 link 548/550/552/417 위에 있거나 `/v2x/tim_message/can_go_status` 의 `do_not_go_forward=True` 수신 시, web_hmi(Three.js) 지도에 붉은 30% 반투명 박스 + "전방 직진 주행 금지" 배너 + 좌/우 진행 화살표를 표시하고, rviz(stat_display)에 "전방 직진 주행 금지" OverlayText 팝업을 띄운다. 원천: `_work_item/hmi_block.md`.

**트리거:** "hmi 블로킹", "직진 금지 표시", "hmi_block", "전방 직진 주행 금지", "블로킹 박스", "주행 경로 블로킹", "블로킹 다시" 요청 시 `hmi-block-display` 스킬 사용. 단순 코드 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-06-24 | 초기 구성 + 구현 + 라이브 검증 | agents 3 (hmi-block-{analyst,coder,verifier}) + skills/hmi-block-display + web_hmi(web_hmi_bridge.py/BlockZones.jsx/index_threejs_f1.html) + stat_display(.h/.cpp/CMake/package.xml) + workspace_config/ioniq_statdisplay.rviz | `_work_item/hmi_block.md` 요구 구현. 트리거 `on_block_link`(LINK_ID∈{548,550,552,417}) + `do_not_go_forward`(can_go_status True 2.0s staleness). 박스 좌표 WGS84 1e7 7점→EPSG:5179 2폴리곤(Box A/B). bag0 라이브 검증: on_block_link=1 166회(=link417), go_ahead_popup "전방 직진 주행 금지" ADD 229회. stat_display 별도 토픽 `/rviz/jsk/go_ahead_popup`(시스템 popup 비충돌). 추가-온리(기존 동작 보존) |

## 하네스: tim-pedes-display

**목표:** TIM(V2X) 보행자(`/obu/v2x_pedes_assistance`)와 차량 자체 횡단보도 판단(`/katech_msg/crosswalk_detection`)을 퓨전하여 소스(자체/OBU) 필드 포함 새 토픽(`/katech_msg/crosswalk_ped_fusion`)으로 발행하고, web_hmi 지도의 횡단보도 #1/#2 위에 보행자 유무를 붉은 점멸 + "전방 보행자 주의" 팝업(소스별 색상)으로 표시. 원천: `claude_work_list/tim_pedes_display.md`.

**트리거:** "tim pedes", "보행자 퓨전", "횡단보도 보행자 표시", "tim_pedes_display", "보행자 퓨전 다시", "횡단보도 hmi" 요청 시 `tim-pedes-display` 스킬 사용. 단순 코드 질문은 직접 응답.

**확정 결정:** #1=south_pedes / #2=east_pedes · 새 퓨전 노드(CAN 무손상, 검출 노드는 additive `crosswalk_id`만) · 팝업 색상 자체=주황#ff9800/OBU=빨강#ff3030/둘다=자홍#ff30ff · active LINK_ID {1239,1238}→#1, 1205→#2.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-07-07 | 초기 구성 | agents 3 (tim-pedes-{analyst,coder,verifier}) + skills/tim-pedes-display + `_tim_pedes_workspace/00_constraints.md` | `claude_work_list/tim_pedes_display.md` 보행자 퓨전+HMI 표시 요청. 사용자 결정 3건 반영(#2=east_pedes, 새 퓨전 노드 CAN 무손상, 팝업 색상) |

## 하네스: global-nav-hmi

**목표:** web_hmi(Three.js) 지도에 주행 예정 경로(`A2_LINK.shp` 링크 시퀀스)를 자차 앞 굵은 선(중앙 차선 대표 1선)으로 표시하고, OBU TIM(`do_not_go_forward`) 팝업/배너 트리거 시 기존 경로를 삭제하고 신규(우회) 경로로 전환·latch. 지나간 구간은 페이드. 원천: `claude_work_list/global_nav_hmi.md`.

**트리거:** "주행 예정 경로", "global_nav", "경로 표시", "경로 전환", "우회 경로 안내", "네비 라인", "경로 다시", "경로 하네스" 요청 시 `global-nav-hmi` 스킬 사용. 단순 경로/링크 조회는 직접 응답.

**확정 결정:** 중앙 차선 대표 1선 · 오프라인 사전 추출 JSON · 범위=분기 직후 795116/795118까지(후반 ITSLinkID 구간 제외) · 전환=`on_block_link AND do_not_go_forward`(기존 `/hmi/state` 재사용) 후 latch · 분기점 좌차선 `A222BF785188`→old `785058`(직진)/new `785057`(좌분기) · 빈구간 `785106→785109` BFS 연결 · CAN 무손상 추가-온리.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-07-08 | 초기 구성 | agents 3 (global-nav-{analyst,coder,verifier}) + skills/global-nav-hmi + `_global_nav_workspace/00_constraints.md` | `claude_work_list/global_nav_hmi.md` 주행 예정 경로 표시+전환 요청. 사용자 결정 4건 반영(중앙차선 1선, 오프라인 JSON, 795까지, latch 전환) |

## 하네스: spat-dir-match

**목표:** MQTT/OBU SPaT 신호를 ego 진행방향(local `MANUAVER` -1/0/1)에 해당하는 movement(`MovementStateName`)로 매칭해 CAN·HMI 에 반영. 원천 확정사실: `_spat_dir_workspace/00_constraints.md`.

**트리거:** "spat 방향 매칭", "신호등 방향 매칭", "MANUAVER 매칭", "MovementStateName 매칭", "좌회전 신호 매칭 안됨", "방향 신호등 잘못 나옴", "spat 방향 다시" 요청 시 `spat-dir-match` 스킬 사용. 단순 조회는 직접 응답.

**확정 결정:** 방향 매핑 -1→LEFT / 0→{STR,STRAIGHT} / 1→RIGHT (MQTT 약어+OBU 풀네임 혼재) · PED/BUS/BYC 배제 · signalGroup 단독매칭 금지(같은 SG 다방향, 방향 문자열이 최종 판별자) · 매칭 실패 시 안전 리셋(신호 0) · CAN 무손상(spat_CAN_writer 선택 로직만).

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-07-14 | 초기 구성 + 1회 실행 | agents 3 (spat-dir-{analyst,coder,verifier}) + skills/spat-dir-match + `_spat_dir_workspace/` | HMI 3경로 방향필터 부재로 병합노드 알파벳순(LEFT 최선) 오선택 버그. web_hmi(hmi_state.py)+stat_display+CAN 수정, bag 실증(516/518 OLD LEFT→NEW STR). pyqt main_window 제외(사용자 스코프) |

## 하네스: stopline-adj

**목표:** senario mat 도로링크 끝을 노면선표시(B2_SURFACELINEMARK) 정지선까지 연장하고 다음 링크를 교차점부터 트림. 웨이포인트/station(길이)/is_stop_line 갱신 + HTML mat 뷰어 반영. 원천: `claude_work_list/stop_line_adj_*.md`, 확정사실: `_stopline_adj_workspace/00_constraints.md`.

**트리거:** "링크 정지선 매칭", "정지선까지 연장", "링크 끝 정지선", "stop_line_adj", "링크 길이 수정 정지선", "mat 정지선 다시" 요청 시 `stopline-adj` 스킬 사용. 단순 조회는 직접 응답.

**확정 결정:** 편집 대상 6개 mat만(연장 L + 트림 N 세트) · 공유정점 L.end==N.start==교차점 P · L=L점+N[1:k+1]+P / N=P+N[k+1:] · station 0부터 재계산 · 위상(NEXT/L/R_LINK)·IID/SG/MANUAVER 등 불변 · is_stop_line 연장링크=1/트림링크=0 · 원본 백업 · 뷰어 var DATA 6 feature만 byte-safe 패치(regen_mat_viewer.py repo외부라 직접).

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-07-15 | 초기 구성 + 1회 실행 | agents 3 (stopline-adj-{analyst,coder,verifier}) + skills/stopline-adj + `_stopline_adj_workspace/` | `stop_line_adj_20260715.md`: 링크 871/870/877 을 B2209W001421(Kind530 정지선)까지 +2.9m 연장(66→68.9m), 3871/3870/3877 트림. verifier 6항목 PASS(정지선 수직거리 0m, 287 mat 불변) |

## 하네스: crosswalk-position

**목표:** `claude_work_list/crosswalk_position.md` 의 횡단보도 폴리곤(WGS84)을 `src/sensing/can/src/katech_ped_detector.py` 의 EPSG:5179 `crosswalk_data` 로 교체(오프라인 pyproj 변환 후 리터럴)하고 senario mat 뷰어(`mapfiles/senario/mat_viewer_senario_260514c 1.html`)에 각 횡단보도 다각형(채움+토글+범례)을 표시. 원천: `claude_work_list/crosswalk_position.md`, 확정사실: `_crosswalk_position_workspace/00_constraints.md`.

**트리거:** "횡단보도 좌표", "crosswalk_position", "횡단보도 다각형", "ped_detector 좌표", "횡단보도 뷰어 표시", "횡단보도 좌표 다시" 요청 시 `crosswalk-position` 스킬 사용. 단순 조회는 직접 응답.

**확정 결정 (Phase1):** 범위=좌표+뷰어+검증만(occupancy_msg·CAN·HMI 무변경, 검출토픽 순회로 자동 커버) · 변환 `pyproj EPSG:4326→5179 always_xy=True` 입력(lon,lat) · .py 는 `crosswalk_data` 딕셔너리만 교체(리터럴) · 뷰어 추가-온리 · 원본 백업.

**확정 결정 (Phase2, 사용자 "둘 다"):** Phase1 의 CAN/occupancy 무변경 제약 **해제** · 크로스워크→접근 LINK_ID `CW_LINKS`(원천 `crosswalk_position.md` §134, detector·fusion 공유 dict) · detector `find_crosswalks_containing_object` 를 ego LINK_ID 로 **게이팅**(on_crosswalk·occupancy 동시, 접근 크로스워크만) · occupancy_msg `uint8[] occupied_ids` 추가(기존 2필드 보존) · fusion active=CW_LINKS 조회(1~9), own=`active in occupied_ids`, obu=#1·#2만(3~9 OBU 없음) · CAN writer/web_hmi(CrosswalkZones) 무변경(3~9 점멸은 자동).

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-07-15 | 초기 구성 | agents 3 (crosswalk-position-{analyst,coder,verifier}) + skills/crosswalk-position + `_crosswalk_position_workspace/00_constraints.md` | `crosswalk_position.md` 9개 횡단보도(1·2 기존=동일, 3~9 신규) 좌표를 ped_detector 에 반영 + mat 뷰어 표시 요청. 사용자 결정 2건(범위=좌표+뷰어만, 뷰어=채움 다각형+토글+범례) |
| 2026-07-15 | Phase1 실행 | katech_ped_detector.py(crosswalk_data 1~9) + mat 뷰어 var CROSSWALK + CrosswalkZones.jsx(표시 1~9) | 9개 좌표 반영·뷰어·web_hmi 표시(표시 전용). verifier 7/7 (aaefdc4/fd1baee) |
| 2026-07-15 | Phase2 (LINK 게이팅+HMI 경보, "둘 다") | katech_ped_detector.py(LINK 게이팅+occupied_ids) + crosswalk_occupancy_msg(`uint8[] occupied_ids`) + crosswalk_ped_fusion.py(active 1~9) | §134 크로스워크→LINK 매핑으로 on_crosswalk(CAN) 링크 게이팅 + fusion active 3~9 → web_hmi 3~9 점멸. catkin_make EXIT0, verifier 7/7. CAN writer/web_hmi 무변경 |
