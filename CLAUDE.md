# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 작업 원칙

LLM 코딩 실수를 줄이기 위한 행동 지침. 안정성을 속도보다 우선시한다. 사소한 작업은 판단껏 적용.

### 1. 코딩 전에 생각하라

**가정하지 말 것. 혼란을 숨기지 말 것. 트레이드오프를 드러낼 것.**

구현 전에:
- 가정은 명시적으로 말한다. 불확실하면 묻는다.
- 해석이 여러 갈래면 모두 제시한다 — 혼자 결정하지 않는다.
- 더 단순한 방법이 있으면 말한다. 필요하면 사용자 의견에 반론한다.
- 불명확하면 멈추고, 무엇이 혼란스러운지 짚고, 묻는다.

이 프로젝트 특수 사례:
- CAN/DBC 시그널 변경은 차량 거동에 직결된다. 시그널 이름·스케일·바이트 위치를 추측하지 말고 `dbc/` 파일 또는 CANoe 결과로 확인한다.
- 좌표계(EPSG:5179 vs WGS84)·TF frame(`/base_link` vs `/ego_frame`)·시간 단위(ms/s/ns) 가정은 명시.

### 2. 단순함 우선

**문제를 해결하는 최소 코드. 추측성 금지.**

- 요구되지 않은 기능 추가 금지.
- 1회성 코드에 추상화 금지.
- 요청되지 않은 "유연성"·"설정 가능성" 금지.
- 일어날 수 없는 시나리오에 대한 에러 처리 금지.
- 200줄로 짠 게 50줄로 가능하면 다시 짠다.

자문: "시니어가 보면 과설계라고 할까?" → 그렇다면 단순화.

ROS 진단 노드 같이 패턴이 정해진 영역(콜백→플래그→타이머)에서는 기존 패턴을 따르고, 새로운 추상 클래스를 만들지 않는다.

### 3. 외과적 변경

**필요한 부분만 건드린다. 본인이 만든 잔해만 치운다.**

기존 코드 편집 시:
- 인접 코드·주석·포맷을 "개선"하지 않는다.
- 망가지지 않은 것을 리팩토링하지 않는다.
- 본인이 다르게 짰을 스타일이라도 기존 스타일에 맞춘다.
- 무관한 데드 코드를 발견하면 언급만, 삭제는 하지 않는다.

변경으로 고아가 생기면:
- *내* 변경이 미사용으로 만든 import/변수/함수만 제거한다.
- 기존부터 있던 데드 코드는 요청 없이는 제거하지 않는다.

테스트: 변경된 모든 줄은 사용자 요청으로 직접 추적되어야 한다.

특히 `to_control_team_demo.py`, `chassis_CAN_reader`, `IONIQ_CAN_reader` 같은 핵심 경로는 요청 범위 밖 수정 금지. 진단 노드 8개도 한 묶음이라 한 노드만 바꿀 때 다른 노드까지 "통일"하지 않는다.

### 4. 목표 기반 실행

**성공 기준을 정의하고, 검증될 때까지 반복한다.**

작업을 검증 가능한 목표로 변환:
- "검증 추가" → "잘못된 입력에 대한 테스트를 작성하고 통과시킨다"
- "버그 수정" → "버그를 재현하는 테스트를 작성하고 통과시킨다"
- "X 리팩토링" → "전후로 테스트가 통과함을 확인한다"

다단계 작업은 짧은 계획을 명시:
```
1. [단계] → 검증: [확인 방법]
2. [단계] → 검증: [확인 방법]
```

이 프로젝트의 검증 수단:
- CAN 송신: Vector CANoe 캡처 (rate/timing은 CANoe가 1차 근거)
- ROS 토픽: `rostopic echo`, `rostopic hz`
- 빌드: `catkin_make --pkg <package_name>` 단일 패키지부터 확인
- HMI/시각화: 실제 rviz/PyQt 화면에서 확인 (타입 체크만으로 부족)

---

**지침이 작동하는 신호:** 불필요한 변경이 줄어든다, 과설계로 인한 재작업이 줄어든다, 실수 후가 아닌 구현 전에 질문이 나온다.

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

## 하네스: web-hmi-review

**목표:** `src/visualization/web_hmi`의 launch + bridge + frontend + variant 4영역 정합성을 다각도 검토.

**트리거:** "web_hmi 검토", "HMI 점검", "HMI 코드 리뷰", "다시 검토", "재실행" 요청 시 `web-hmi-review` 스킬 사용. 단순 질문은 직접 응답.

**변경 이력:**
| 날짜 | 변경 내용 | 대상 | 사유 |
|------|----------|------|------|
| 2026-05-07 | 초기 구성 | agents 4 (launch/bridge/frontend/variant-auditor) + skills/web-hmi-review | 하네스 검토 요청 |
