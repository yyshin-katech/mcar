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
