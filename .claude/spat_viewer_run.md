---
name: spat-viewer-run
description: spat_viewer 실행 — MQTT/OBU SPaT 라이브 Leaflet 뷰어 + offline bag 리플레이 뷰어(replay.html)
metadata: 
  node_type: memory
  type: project
  originSessionId: 9aaa157b-1f5d-435d-b05a-5d5073737072
---

`spat_viewer` 패키지 (`src/visualization/spat_viewer/`) — OBU + MQTT SPaT 를 지도 위에 라이브 표시하는 self-contained Leaflet HTML 뷰어.

## 실행

siheung.launch / katech_test.launch 와 **별도 터미널**에서 (포트 충돌 없음 — rosbridge 9090, http 8080):

```bash
roslaunch spat_viewer spat_viewer.launch
xdg-open http://localhost:8080/
```

- siheung.launch 가 이미 떠 있어야 SPaT 가 흐른다 (`mqtt_spat_rx_node` 가 `/siheung_v2x/mqtt_spat` 발행). prod MQTT 는 [[mqtt-vpn-setup-harness]] 의 SecuwaySSL VPN 연결 필요.
- 포트 변경: `roslaunch spat_viewer spat_viewer.launch port:=9091 http_port:=8081`

## 구독 토픽 (web/index.html)

- `/siheung_v2x/mqtt_spat` (MQTT), `/siheung_spat` (OBU) → `ingestSpat`
- `/localization/pose_2d_gps` (ego 마커), `/localization/to_control_team` (타깃 IID/SG 강조)

## 데이터

- `web/data/intersections.json` 교차로 15개: `[134,136,165,168,201,203,302,504,507,508,509,516,517,518,519]`
- `web/data/road_links.json` 도로 link 308개
- json 재생성: `scripts/extract_map_data.py` (mapfiles/senario/link_*.mat 직접 추출, pyproj 만 의존)

## 리플레이 뷰어 (offline, bag 재생) — replay.html

라이브 뷰어와 **별개**로, 저장된 주행 bag 을 ego 위치별 **방향매칭 신호등**과 함께 재생·스크럽하는 self-contained 뷰어. 수정된 SPaT 방향매칭([[spat-dir-match]]) 검증/시연용. roscore 불필요.

- 데이터 추출(1회): `source devel/setup.bash && python3 scripts/extract_spat_replay.py [bag...]`
  → `web/data/replay_timeline.json`. 기본 `~/bag_data/2026-06-24-14-37-33_2026-06-24*.bag` 3개 = **2427 샘플 / 512s / 5Hz**. ego `host_east/north`→WGS84, `/spat_merged` 에서 **수정 매칭 헬퍼(hmi_state 동일 복제)** 로 ego 방향(MANUAVER -1/0/1) 신호 선택.
- 실행: `bash scripts/serve_http.sh 8080` → `http://localhost:8080/replay.html`.
- 화면: 지도 배경(road_links/intersections.json 재사용) + ego 궤적·방향화살표(yaw) + 우상단 신호등 패널(색/방향/IID/SG/movement/LINK·제한속도) + 재생·일시정지·0.5~4×·타임라인 스크럽·ego 따라가기. 타깃 교차로 마커가 신호 색으로 하이라이트. 지도 타일 CDN 인터넷 필요.
- `replay_timeline.json` 은 파생물 — bag 바뀌면 추출 재실행.

관련: [[spat-merge-obu-mqtt]] (소비자측 /spat_merged 병합), [[spat-dir-match]] (방향 매칭 로직).
