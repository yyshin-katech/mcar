---
name: spat-viewer-run
description: MQTT/OBU SPaT 라이브 Leaflet 뷰어(spat_viewer) 실행 방법
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

관련: [[spat-merge-obu-mqtt]] (소비자측 /spat_merged 병합).
