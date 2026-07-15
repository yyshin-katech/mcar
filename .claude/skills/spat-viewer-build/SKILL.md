---
name: spat-viewer-build
description: VPN 통해 prod 브로커에서 들어오는 MQTT SPaT (`/siheung_v2x/mqtt_spat`) + OBU SPaT (`/siheung_spat`) 를 라이브로 보여주는 self-contained Leaflet HTML 뷰어 (mat_viewer 패턴) 를 신규 개발한다. 지도 위에 shp_map 도로 link, 교차로별 신호등 색/잔여시간, ego 마커(real-time), target intersection 강조를 표시. 메인 launch (siheung.launch) 와 동시 실행 가능. 사용자가 "spat 뷰어 만들어", "신호등 뷰어", "spat 시각화", "MQTT spat 보는 페이지", "vpn spat 확인 페이지", "뷰어 갱신", "뷰어 다시 만들어", "spat-viewer", "교차로 색 잘못 나옴" 등을 요청하면 반드시 이 스킬을 사용. 단순 코드 질문은 직접 응답.
---

# spat-viewer-build — SPaT Live Viewer (Orchestrator)

prod 브로커에서 VPN 으로 들어오는 SPaT 데이터를 (OBU 우선, MQTT fallback 정책 포함) 라이브 시각화하는 self-contained Leaflet 뷰어를 새로 만든다. 메인 노드들 (`launch/siheung.launch`) 이 떠있는 상태에서 별도 launch 로 띄워 브라우저로 확인할 수 있어야 한다.

## 실행 모드
**에이전트 팀 (파이프라인)**: analyst → coder → verifier. 사양서/구현/검증을 3단계로 명확히 분리해 코드 품질과 검증 추적성을 확보. 다른 하네스 (senario-gps-pub, vpn-net) 와 동일 패턴.

## Phase 0: 컨텍스트 확인

작업 시작 전 `_workspace_spat_viewer/` 상태로 실행 모드 결정.

| 상태 | 모드 |
|------|------|
| `_workspace_spat_viewer/` 없음 + `src/visualization/spat_viewer/` 없음 | **초기 실행** — 전체 파이프라인 |
| 둘 다 존재 + 사용자가 부분 수정 요청 (예: "색만 바꿔", "ego 마커 안 보임") | **부분 재실행** — 해당 에이전트만 재호출 |
| 사용자가 새 요구 (다른 shp, 다른 토픽) | **새 실행** — 기존을 `_workspace_spat_viewer_prev/` 로 mv 후 처음부터 |

## Phase 1: 분석 (spat-viewer-analyst)

목표: 7가지 입력을 정밀 분석해 코더가 의문점 없이 구현할 수 있는 사양서 1개.

핵심 입력:
- 참조 뷰어: `src/localization/gps_system_localizer/mapfiles/senario/mat_viewer_senario_260514c 1.html` (파일명 공백 주의)
- SPaT msg: `src/v2x/v2x_msgs/msg/intersection_array_msg.msg` (+ 하위 msg 들)
- 발행자 코드: `src/v2x/siheung_v2x/src/{j2735_decode,mqtt_spat_rx_node}.cpp` (현재 ego 필터 *제거됨*, commit 80624de)
- ego 매칭: `src/sensing/can/src/spat_CAN_writer.cpp` (OBU>MQTT fallback)
- shp_map: `src/localization/gps_system_localizer/src/shp_map/HDMap_Oido_New/C1_TRAFFICLIGHT.shp` (+ .prj)
- IID↔SigGrp: `src/localization/gps_system_localizer/mapfiles/senario3/intersection_signal_links.csv`
- ego pose 토픽 후보 (`/localization/pose_2d_gps`, `/sensors/gps/inspva` 등)

산출물: `_workspace_spat_viewer/01_analyst_spec.md`

## Phase 2: 코딩 (spat-viewer-coder)

목표: 사양서대로 `src/visualization/spat_viewer/` 패키지 신규 생성.

산출물:
- `package.xml`, `CMakeLists.txt`
- `launch/spat_viewer.launch` (rosbridge_websocket + 정적 웹 서버)
- `web/index.html` (Leaflet + roslibjs self-contained)
- `web/data/intersections.json`, `road_links.json`
- `scripts/extract_map_data.py`
- `_workspace_spat_viewer/02_coder_report.md`

## Phase 3: 검증 (spat-viewer-verifier)

목표: 정적 (파일/json/HTML/build) + 가능 시 동적 (launch parse, port listen) 검증.

검증 항목:
- A. 파일 존재 + json/HTML 정합 (CDN URL, 색상 hex, 토픽 이름)
- B. `catkin_make --pkg spat_viewer` PASS
- C. roslaunch 파싱, rosbridge port 9090 listen
- D. 메인 launch (siheung.launch) 와 노드 충돌 없음 확인

산출물: `_workspace_spat_viewer/03_verifier_report.md` + PASS/PARTIAL/FAIL.

FAIL 시 coder 재호출 1회. 2회 FAIL 이면 사용자에게 보고 후 중단.

## 데이터 흐름

```
analyst ──spec──> coder ──package──> verifier ──verdict──> 사용자
                            │                    │
                            └── (FAIL 시 재작업) ─┘

[런타임]
brower(index.html) ──ws://9090──> rosbridge ──ROS──> {/siheung_v2x/mqtt_spat,
                                                       /siheung_spat,
                                                       /localization/to_control_team,
                                                       ego pose 토픽}
```

전달 방식: 파일 기반 (`_workspace_spat_viewer/`) + 메시지 기반 (SendMessage 알림).

## SPaT phase → 색 매핑 (J2735 표준; analyst 가 사양서에 동일 값 명시)

| phase | 의미 | 색 | 비고 |
|-------|------|-----|------|
| 0 | unavailable | #808080 | 회색 |
| 1 | dark | #303030 | 거의 검정 |
| 2 | stop-Then-Proceed | #ff0000 | 점멸 |
| 3 | stop-And-Remain | #ff0000 | 빨강 |
| 4 | pre-Movement | #ff8800 | 적+황 |
| 5 | permissive-Movement-Allowed | #00cc00 | 진초록 |
| 6 | protected-Movement-Allowed | #00ff00 | 초록 |
| 7 | permissive-clearance | #ffff00 | 황 |
| 8 | protected-clearance | #ffcc00 | 황 (protected) |
| 9 | caution-Conflicting-Traffic | #ffff00 | 점멸 황 |

## OBU/MQTT 우선순위 (뷰어에서도 동일 정책)

`spat_CAN_writer.cpp` 와 동일하게 OBU 우선, 2초 stale 이면 MQTT.
- OBU 메시지 도착 시각 기록
- MQTT 도착 시 `now - obu_last_seen > 2s` 일 때만 표시
- 화면에 현재 표시 소스 (OBU/MQTT/NONE) 배지

## 에러 핸들링

| 단계 | 실패 유형 | 대응 |
|------|----------|------|
| 분석 | shp/.prj EPSG 불명 | 사양서에 "EPSG 확인 필요" 명시, 사용자 질의. 추측 금지. |
| 분석 | intersection_signal_links.csv 컬럼 불일치 | 사양서에 fallback (signalGroup 표시 생략) 명시 |
| 분석 | rosbridge_server 미설치 | analyst 가 사양서에 apt 명령 안내. (이미 설치 확인됨) |
| 코딩 | pyshp 미설치 | `pip install pyshp` 안내 후 재시도. 보고서에 기록. |
| 코딩 | catkin_make 실패 | 1회 자동 fix. 2회 실패면 사용자 보고. |
| 검증 | 동적 검증 불가 (roscore 충돌) | 정적만 PASS, 동적은 SKIP 명시. |
| 검증 | port 9090 점유 | 사용자에게 port 변경 안내, 보고서에 alternative 제안. |

## 에이전트 호출 패턴

오케스트레이터는 다음 순서로 Agent 도구를 호출. 모두 `model: "opus"` 명시.

```
Phase 1:
  Agent(subagent_type="spat-viewer-analyst", model="opus",
        prompt="<사용자 요청 요지 + 모드(initial/partial/new) + 핵심 입력 경로>")
  → 결과 메시지에서 spec 경로 확인

Phase 2:
  Agent(subagent_type="spat-viewer-coder", model="opus",
        prompt="<spec 경로 + analyst 가 발견한 주요 결정사항>")
  → 결과 메시지에서 coder report 경로 + 빌드 PASS/FAIL 확인

Phase 3:
  Agent(subagent_type="spat-viewer-verifier", model="opus",
        prompt="<coder report 경로 + 검증 범위 (정적+가능시 동적)>")
  → 결과에서 verdict 확인. FAIL 이면 Phase 2 재실행 1회.
```

## 사용자 사용법 (verifier 가 보고서에 정리)

```bash
# 1. 메인 노드 (이미 떠있다고 가정)
roslaunch launch/siheung.launch    # mqtt_spat_rx_node 등 띄움

# 2. 별도 터미널에서 뷰어 launch
source /home/katech/mcar_v13/devel/setup.bash
roslaunch spat_viewer spat_viewer.launch

# 3. 브라우저 열기
xdg-open http://localhost:8080/    # 또는 file:///home/katech/mcar_v13/src/visualization/spat_viewer/web/index.html
```

## 테스트 시나리오

**정상 흐름**:
1. analyst → spec.md 작성 (intersections N≥1, road_links M≥1 추출 성공)
2. coder → catkin_make PASS + 8개 파일 생성
3. verifier → 모든 정적 항목 PASS, 동적 항목 PASS (또는 SKIP 이라도 명시)
4. 사용자 라이브 실행 → 브라우저에 지도 + 신호등 색 표시 확인

**에러 흐름 (shp EPSG 불명)**:
1. analyst 가 .prj 읽었더니 EPSG 코드 불명 → 사양서 작성 중단, 사용자 질의
2. 사용자가 EPSG 확정 → analyst 재호출, 사양서 완성
3. 이후 정상 진행

## 산출물 경로 컨벤션

- 작업 디렉토리: `_workspace_spat_viewer/` (다른 하네스의 `_workspace*` 와 분리)
- 신규 코드: `src/visualization/spat_viewer/`
- `.gitignore` 갱신 불필요 (json/launch/html 모두 git 추적; 사용자가 결정한 평문 정책 일관성)
