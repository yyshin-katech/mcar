# mcar — 시흥 자율주행 시스템 (IONIQ EV)

ROS Noetic 기반 자율주행 시스템 (KATECH). 현재 작업 브랜치: **`siheung_dev`** (시흥 시범운행).

```
GPS(Novatel) → localization → to_control_team → CAN writers → 차량
CAN readers → /sensors/* → diagnostic 노드 → stat_display / web_hmi
V2X: OBU(/siheung_spat) + MQTT SPaT(/siheung_v2x/mqtt_spat) → /spat_merged → CAN·HMI
```

- **지도**: mat = `mapfiles/senario` (EPSG:5179), shp = `shp_map/HDMap_Oido_New` (EPSG:32652)
- **좌표계**: EPSG:5179 (Korea TM) · **CAN**: Kvaser canlib

---

## 1. 빌드

워크스페이스 루트에서:

```bash
catkin_make
source devel/setup.bash
```

- 단일 패키지: `catkin_make --pkg <package_name>`
- ⚠️ **브랜치 전환 직후 첫 빌드**가 msg 헤더 경합으로 실패(`... has no member named ...`)할 수 있음 → **한 번 더 `catkin_make`** 하면 통과 (두 브랜치의 `.msg` 정의가 다르고 `devel/`은 공유되기 때문).

---

## 2. 전체 시스템 실행

주행용 `katech_test.launch` 와 진단/HMI용 `diagnostic_only.launch` 를 **별도 터미널에서 함께** 실행한다.

### 2-0. NTRIP RTK 보정 (먼저 실행)
GPS RTK 보정 스트림(RTCM)을 받기 위해 NTRIP 클라이언트를 **먼저** 기동한다:
```bash
roslaunch launch/launch/katech_ntrip.launch
```
`ntripclient`(ntrip_node.py) 실행 → RTK Fix 확보 후 아래 주행 launch 실행.

### 2-1. 주행 (`katech_test.launch`)
```bash
roslaunch launch/katech_test.launch
```
GPS(Novatel)·localization(`to_control_team_demo.py`)·CAN(readers/writers)·V2X(`siheung.launch` 포함)·보행자 퓨전 기동.

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `mat_scenario` | `senario` | mat 지도 폴더 (`mapfiles/<name>`) |
| `shp_scenario` | `HDMap_Oido_New` | shp 지도 폴더 (`src/shp_map/<name>`) |
| `mqtt_server` | `prod` | `prod`=시흥 VPN 사설망(브로커 실서버) / `test`=인터넷 테스트 |

예) `roslaunch launch/katech_test.launch mqtt_server:=test`

### 2-2. 진단 + HMI + rviz (`diagnostic_only.launch`)
```bash
roslaunch launch/diagnostic_only.launch
```
8개 diagnostic 노드 + `stat_display`(rviz 오버레이) + `lanelet_marker` + **web_hmi(threejs_f1, 포트 8088 자동 기동)**.

---

## 3. V2X / VPN (MQTT SPaT · BSM)

`prod` MQTT 브로커(`192.168.255.173:10044`)는 **시흥시 SecuwaySSL VPN** 을 통해서만 접속된다.

```bash
# VPN 접속 (별도) — 클라이언트: ~/sslvpn/SecuwaySSLU_client, 게이트웨이 27.101.133.111:443
# (자격증명·서버정보는 launch/siheung.launch 및 ~/sslvpn/conf/client.info)
```
- `siheung.launch` 는 `katech_test.launch` 에 포함되어 있음 (OBU UDP 수신 `0.0.0.0:9999`, TIM, `spat_merge_node`, BSM 업로드).
- 인터넷 테스트 브로커만 쓰려면 `mqtt_server:=test` (VPN 불필요).

---

## 4. 뷰어 / 리플레이 도구

### A. web_hmi — 실시간 HMI (Three.js F1 대시보드 + 3D 씬)
`diagnostic_only.launch` 가 자동 기동. 단독 실행:
```bash
roslaunch web_hmi web_hmi.launch variant:=threejs_f1
# → 브라우저: http://localhost:8088/index_threejs_f1.html
```
variant: `threejs_f1`(기본) · `threejs` · `f1` · `default`.

### B. web_hmi 리플레이 — bag 재생 HMI
```bash
roslaunch web_hmi web_hmi_replay.launch bag:=2026-07-02-17-58-41_2026-07-02-17-58-42_0.bag run_fusion:=true
# bag 은 ~/bag_data 안의 "파일명"만. → http://localhost:8088/index_threejs_f1.html
```
옵션: `rate:=0.5`(배속) · `run_fusion:=true`(횡단보도 보행자 퓨전 live 생성) · `open_browser:=false`.

### C. spat_viewer — 실시간 SPaT 신호등 뷰어 (Leaflet)
```bash
roslaunch spat_viewer spat_viewer.launch                    # prod (VPN 필요)
roslaunch spat_viewer spat_viewer.launch mqtt_server:=test  # 인터넷 테스트 브로커
roslaunch spat_viewer spat_viewer.launch run_mqtt_rx:=false # siheung.launch 동시 실행 시(중복 수신 방지)
# → 브라우저: http://localhost:8080/
```
rosbridge(9090) + http(8080). 지도 위 도로링크·교차로 신호색/잔여시간·ego 마커.

### D. spat_viewer 리플레이 — 오프라인 bag 재생 뷰어 (roscore 불필요)
저장된 주행 bag 을 **ego 위치별 방향매칭 신호등**과 함께 재생·스크럽.
```bash
# 1) bag → JSON 추출 (기본 ~/bag_data/2026-06-24-14-37-33_2026-06-24*.bag 3개)
source devel/setup.bash
python3 src/visualization/spat_viewer/scripts/extract_spat_replay.py            # 또는 [bag ...] 명시
# 2) http 서버
bash src/visualization/spat_viewer/scripts/serve_http.sh 8080
# → 브라우저: http://localhost:8080/replay.html
```
ego 궤적·방향화살표 + 신호등 패널(색/방향/IID/SG) + ▶재생·0.5~4×·타임라인 스크럽. bag 바뀌면 추출 재실행.

### E. senario mat 뷰어 — 정적 HTML (self-contained)
브라우저로 파일을 직접 연다 (지도 타일만 인터넷 필요):
```
src/localization/gps_system_localizer/mapfiles/senario/mat_viewer_senario_260514c 1.html
```
도로링크(link_*.mat) + 노면표시 **Type5**(B3_SURFACEMARK) + 노면선표시 **B2**(B2_SURFACELINEMARK) + 링크별 신호/MANUAVER 표시(토글). 경로 미리보기는 `route_viewer_senario_260514c.html`.

### F. qt_hmi — 네이티브 HMI (Qt/C++)
```bash
roslaunch qt_hmi qt_hmi.launch
```

### G. pyqt_hmi — PyQt5 HMI
```bash
roslaunch pyqt_hmi pyqt_hmi.launch
```

---

## 5. 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/localization/to_control_team` | `to_control_team_from_local_msg` | 경로/ODD/속도제한/링크(IID·SG·MANUAVER) |
| `/sensors/v_can` | `v_can_msg` | VCU CAN (조향·휠속·기어) |
| `/spat_merged` | `intersection_array_msg` | OBU+MQTT SPaT 병합(IID 단위) |
| `/diagnostic/*` | `*_diagnostic_msg` | 센서별 진단 상태 |
| `/hmi/state`, `/hmi/threejs/*` | — | web_hmi 표시 데이터 |

⚠️ **SPaT 활성 토픽은 브랜치마다 다름** — `siheung_dev`: `/siheung_spat`→`/spat_merged`, `ioniq5_hmi_dev`: `/katri_v2x_node/katri_spat`. 점검 전 `git branch` 확인.

---

## 6. 참고

- 개발 하네스(`.claude/skills`, `.claude/agents`): spat-dir-match(신호 방향매칭)·stopline-adj(정지선 연장)·global-nav-hmi(경로표시)·spat-viewer-build·mqtt-vpn-setup 등. 상세는 `CLAUDE.md`.
- 사용자 언어: 한국어.
