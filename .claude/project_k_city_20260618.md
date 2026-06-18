---
name: project-k-city-20260618
description: "K_CITY_20260618 맵 생성 이력 — shp→mat 변환 규칙, 신규 링크 구성, link_60 NEXT 갱신 누락 gotcha, 학교구역 경고 확대"
metadata: 
  node_type: memory
  type: project
  originSessionId: 75e36484-068a-4b93-87f0-e653085a596c
---

K_CITY_20260618 은 K_CITY_20260608 을 복제하여 link_61 교체 + link_86/87/88 신규 추가한 맵.

**Why:** ~/shp_file/A2_LINK shp 파일(A2256W000085 등 4개 링크)을 기존 경로 구간에 삽입하여 교차로 진입 구간을 세분화.

**How to apply:** 이후 shp→mat 추가 변환 시 동일 규칙 적용.

## shp 변환 규칙

- shp 파일 좌표계: **EPSG:32652** (UTM zone 52N) → mat 저장 좌표계: **EPSG:5179** (Korean TM)
- 변환 스크립트: `mapfiles/K_CITY_20260618/convert_links.py`
- 포인트 간격: 2m (시작·종점 항상 포함)
- 라이브러리: `pyshp(shapefile)` + `pyproj Transformer`

## 링크 구성 (chain: 60→88→86→87→61→62)

| 파일 | shp ID | NEXT | is_stop | IID | SGID | 비고 |
|------|--------|------|---------|-----|------|------|
| link_61.mat | A2256W000085 | 62 | 0 | 0 | 0 | 기존 61 교체 |
| link_86.mat | A2256W000229 | 87 | 0 | 0 | 0 | 신규 |
| link_87.mat | A2256W000231 | 61 | 1 | 1500 | 10 | 신규, 교차로 정지선 |
| link_88.mat | A2256W000327 | 86 | 1 | 100 | 16 | 신규, 교차로 정지선 |

## 연동 파일 수정

- `launch/katech_test.launch`: MAPFILE_PATH → `K_CITY_20260618`
- `to_control_team_demo.py`: `MAX_LANE_ID = 88` (구 85)

## gotcha: 분할 시 이전 링크 NEXT 포인터 갱신 누락 (2026-06-18, commit 9729337)

`convert_links.py`는 **대상 4개 링크만 재생성**하고 그 체인으로 진입하는 **이전 링크의 NEXT_LINK_ID는 갱신하지 않는다.** 기존 단일 link_61을 88→86→87→61 체인으로 분할했는데, 예전에 61로 진입하던 `link_60.mat`이 `NEXT_LINK_ID=61` 그대로 남아 60→61 라우팅이 **52.3m 점프**했다. 체인의 실제 진입점은 link 88(60.last ↔ 88.first gap 0.4mm 일치). `link_60.mat`의 NEXT를 61→88로 수정해 해결(scipy.io 로 값만 교체, dtype/타 필드 보존).

**검증법:** 모든 link_*.mat 로드 → NEXT_LINK_ID로 그래프 구성 → 각 링크 last↔next.first, prev.last↔first 의 유클리드 거리(EPSG:5179, m 단위)가 ~0 인지 확인. 분할/삽입 작업 후 항상 양 끝 인접 링크 연속성 체크.

## 학교구역(어린이 보호구역) 경고 — 61→61/86/87/88 확대 (commit f4471f1)

원래 link 61 진입 시만 나오던 "어린이 보호구역입니다! 주의하세요!" 지속 팝업+음성 경고를 61/86/87/88 전부에서 발화하도록 확대.

- 음성: `stat_display.cpp` `sound_play("GUARDZONE")`(guardzonewarnning.mp3 반복) — 음성 발화는 stat_display 한 곳뿐
- 팝업: `stat_display.cpp` + pyqt `main_window.py` / `hmi_state.py` (pyqt엔 사운드 분기 없음, 팝업만)
- **이 경고는 `LINK_ID` 키 분기이며 `Road_State==1`("전방 ODD 이탈 경고") 분기보다 우선**. to_control_team_demo 에서 61/86/87/88 을 Road_State=1 로도 설정했지만 LINK_ID 분기에 가려져 실제로는 학교구역 문구가 표출됨. 관련 [[project-control-fault-not-displayed]] [[project-gps-status-takeover]]
