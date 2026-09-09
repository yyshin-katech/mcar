---
name: reference-sdsm-bag-data
description: SDSM(J3224) 수집 데이터 위치 — ~/20251128/sdsm_data 의 bag 7개(/obu/sdsm)·pcapng·디코딩 CSV. ~/bag, ~/bag_data 에는 SDSM 없음
metadata:
  type: reference
---

# SDSM(J3224) 데이터 소재 (katech NUC, 2026-08-31 확인)

**SDSM 이 들어있는 유일한 경로 = `~/20251128/sdsm_data/`** (= `/home/katech/20251128/sdsm_data/`).
토픽 `/obu/sdsm`, 타입 `j3224_msgs/sdsm` (md5 `dbab0794cc21415012f99fa173c1f856`).
수집 2025-11-28 12:53~13:49 (시나리오 1-1 ~ 3-3, 각 30~55초).

## bag 7개 (bag 안에는 `/obu/sdsm` + `/rosout` 뿐 — 센서/localization 토픽 없음)

| bag | 길이 | SDSM msgs | 시작(현지) | unified CSV 행수 |
|---|---|---|---|---|
| senario_1-1.bag | 33.1s | 210 | 12:53:59 | 493 |
| senario_1-2.bag | 49.5s | 340 | 13:06:14 | 170 |
| senario_2-1.bag | 50.4s | 259 | 13:13:55 | 569 |
| senario_2-2.bag | 49.2s | 215 | 13:21:13 | 540 |
| senario_3-1.bag | 54.7s | 499 | 13:28:16 | 1071 |
| senario_3-2.bag | 29.2s | 114 | 13:41:29 | 331 |
| senario_3-3.bag | 34.8s | 253 | 13:48:37 | 548 |
| **합계** | | **1,890** | | **3,722** |

## 같은 디렉토리의 부속 파일
- `senario_*.pcapng` 7개 — OBU UDP 원본 캡처 (bag 과 1:1)
- `senario_*_sdsm_unified.csv` 7개 — 디코딩 결과. **1행 = 메시지 내 객체 1개**, 51컬럼
  (refPos lat/lon/elev + raw, refPosConf, objectID, objType/objTypeCfd, offsetX/Y/Z(cm·m),
   speed(raw/mps/kmh), heading(raw/deg/direction), abs_lat/lon_deg, distance_from_ref_m …)
- `sdsm_bag_to_unified_csv.py` (bag→CSV), `analyze_unified_csv.py` (통계·플롯)
- `*_summary.txt`, `*_analysis.png`, `*_position_heatmap.png`
- `SDSM_coordinate_analysis_report.pdf`, `senario_3-{2,3}_*_map.html`(Leaflet/Google 좌표 검증 뷰어), `senario_compare_map.html`

## 데이터 특성 (사용 전 주의)
- **objType 이 전량 `Unknown(0)`** (3,722행 100%). RSU 가 객체 분류를 안 실어보냄 → 이 데이터만으로 보행자/차량 구분 불가.
- **sourceID(RSU) 가 시나리오마다 다름**: 1-1=`234-251-201-172`, 1-2=`161-115-251-164`,
  2-1=`161-115-251-164`(38)+`208-202-209-92`(531), 2-2=`208-202-209-92`,
  3-1=`48-34-86-76`(102)+`208-202-209-92`(969), 3-2=`48-34-86-76`, 3-3=`81-224-96-241`.
  → 2-1, 3-1 은 RSU 전환(핸드오버) 구간이 섞여 있음.
- 객체 위치는 refPos(RSU 기준점) + offsetX/Y 상대좌표. CSV 의 `abs_lat/lon_deg` 는 스크립트가 환산한 값.

## SDSM 이 **없는** 경로 (2026-08-31, bag 73개 `rosbag info` 전수 확인 — `sdsm`/`j3224` 0건)
- `~/20251128/*.bag` (senario2-1_fixed / 2-2 / 2-2_fixed, 각 3.4GB) — 주행 센서 bag, V2X 없음
- `~/bag/` 32개 261GB — 3~5월 K-City 계열. V2X 는 `/katri_v2x_node/katri_spat` 뿐 (20260318 15/15, 20260423 4/12, 20260508 0/5)
- `~/bag_data/` 41개 109GB — 7월 시흥 계열. `/spat_merged`, `/siheung_spat`, `/siheung_v2x/mqtt_spat`,
  `/siheung_v2x/bsm_tx`, `/v2x/tim_message`, `/obu/v2x_pedes_assistance`(`v2x_msgs/v2x_pedes_assist_msg`, 33개 bag).
  `/obu/*` 중 존재하는 건 `v2x_pedes_assistance` 하나뿐 — `/obu/sdsm` 없음.

브랜치 SPaT 토픽 대응은 [[spat-merge-obu-mqtt]], [[project-v2x-spat-topic]] 참조. bag 재생 주의사항은 [[reference-tim-pedes-bag-replay]].
