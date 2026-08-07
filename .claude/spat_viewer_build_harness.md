---
name: spat-viewer-build 하네스 + mat 데이터 소스 라이브 검증
description: src/visualization/spat_viewer 패키지 신규 (rosbridge_websocket:9090 + http.server:8080 + Leaflet/proj4js/roslibjs CDN). 데이터 소스는 mapfiles/senario/link_*.mat 직접 추출 (shp 아님) — IID/SG/stop_line/MANUAVER 가 mat 키로 들어있음. 15 IIDs, 308 link. 라이브 브로커 매치 13/15. is_stop_line=1 링크 강조 (#fab387)
type: project
originSessionId: 8099a2fb-c590-474a-b3ff-3eb9f9dff9be
---
# spat-viewer-build 하네스 — 라이브 검증 + mat 기반 추출

## 핵심 사실

**패키지 위치**: `src/visualization/spat_viewer/` (3-phase 하네스 산출)
- `launch/spat_viewer.launch` — rosbridge_websocket(9090) + http.server(8080) 노드. 메인 `siheung.launch` 와 동시 실행 가능 (포트/노드 충돌 없음).
- `web/index.html` — self-contained. CDN: Leaflet 1.9.4 + proj4js 2.9.2 + roslibjs 1.4.1.
- `web/data/intersections.json`, `web/data/road_links.json` — 빌드 산출물 (git 추적).
- `scripts/extract_map_data.py` — mat → json 변환 (EPSG:5179 → 4326, pyproj).

## 데이터 소스: shp 아닌 mat

처음에 `C1_TRAFFICLIGHT.shp` (HDMap_Oido_New) 로 추출했지만, `mapfiles/senario/link_*.mat` 가 IID/SG 정보를 **직접** carry 한다는 게 더 정확한 소스로 판명.

mat 파일 핵심 필드:
- `east`, `north` — EPSG:5179 vertex 배열
- `LINK_ID`, `LINK_ID_string`, `NEXT_LINK_ID`
- `look_at_IntersectionID`, `look_at_signalGroupID`
- `is_stop_line` (1 = 정지선 링크)
- `MANUAVER`, `Speed_Limit`

**결과**: 308 link, 8024 vertex, 15 IID — IIDs = `[134, 136, 165, 168, 201, 203, 302, 504, 507, 508, 509, 516, 517, 518, 519]`. 그 중 12 IID 는 stop_line 평균 좌표, 3 IID (508/509/516) 는 stop_line 없어 link endpoint 평균 fallback.

shp 기반은 2544 feature / 1008KB 였는데 mat 기반은 308 link / 262KB — 시각화 부담 크게 감소.

## 라이브 검증 (2026-05-26)

순서: VPN → mqtt_spat_rx_node → roslaunch spat_viewer → 브라우저 → 검증.

```
sudo /home/katech/sslvpn/SecuwaySSLU_client     # tun0 = 172.18.113.51
rosrun siheung_v2x mqtt_spat_rx_node \
  _broker_host:=192.168.255.173 _broker_port:=10044 \
  _username:=xcms-mtqq _password:='xcms123!' \
  _topic:=V2N/1321103202/trf_drct/spat
roslaunch spat_viewer spat_viewer.launch
xdg-open http://localhost:8080/
```

브로커가 보내는 IID 26개 중 mat extract 15 IID 와 **13 매치** (이전 shp 추출은 9 IID 중 6 매치 = 67%, mat 은 87%). 남은 미매치는 비-시화 권역 SPaT.

## SPaT/뷰어 정책

| 항목 | 값 |
|------|------|
| OBU 우선 timeout | `SPAT_STALE_MS = 2000` (index.html L80) — `spat_CAN_writer.cpp` 와 동일 정책 (클라이언트측 재현) |
| phase→color | `PHASE_COLOR` map (index.html L90~) — J2735 0~9 표준 hex |
| 토픽 구독 | `/localization/pose_2d_gps` (100ms throttle), `/siheung_spat`, `/siheung_v2x/mqtt_spat`, `/localization/to_control_team` (200ms) |
| stop_line 강조 | `is_stop_line === 1` → `#fab387` (오렌지, weight=4). 일반 링크 `#4fc3f7` weight=1 |
| 캐시 구조 | `Map<IID, Map<sg\|name, {phase, minEnd_ds, recv_ts}>>` |

## 동시 실행 안전성

- 포트: 9090 (rosbridge), 8080 (http) — 메인 launch 들과 충돌 안 함.
- 노드 중복 없음: spat_viewer.launch 에 `mqtt_spat_rx_node` / `bsm_tx_node` 등 메인 노드 포함 안 함.
- 메인 (`launch/siheung.launch`) 가 SPaT 를 발행하고, spat_viewer 는 구독만 하는 일방향 의존.

## 셋업 산출물 (3-phase 하네스)

- 분석: `_workspace_spat_viewer/01_analyst_spec.md` (703줄, 초기 shp 사양; 추후 mat 으로 변경)
- 구현: `_workspace_spat_viewer/02_coder_report.md` (catkin_make PASS, 8개 파일)
- 검증: `_workspace_spat_viewer/03_verifier_report.md` (GO 판정)

## 주의

`extract_map_data.py` 는 mat 직접 읽음 → shp/csv 의존 제거됨. mat 갱신되면 스크립트 한 번 다시 실행하면 json 도 갱신.

쌍방향 매핑 (`look_at_signalGroupID` 와 SPaT `MovementStateName` 의 SG 매치) 은 mat → intersections.json 의 `signal_groups` 리스트가 정답지 역할.
