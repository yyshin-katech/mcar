---
name: project-sdsm-web-hmi
description: SDSM(J3224) 오브젝트를 web_hmi threejs_f1 에 표출 — offsetX/Y 단위는 cm 아닌 dm + X 부호반전, 헤딩은 사이트 상수 220도. rviz_filter.cpp 의 변환은 틀렸으나 의도적 보존
metadata:
  type: project
---

# SDSM(J3224) → web_hmi threejs_f1 표출 (2026-09-09, commit afc8377, siheung_dev)

`/obu/sdsm` (`j3224_msgs/sdsm`) 을 `web_hmi_threejs_bridge.py` 에서 절대 EPSG:5179 로 변환해
**`/hmi/threejs/sdsm`** 으로 발행하고, `web/threejs/SdsmObjects.jsx` 가 `map.origin` 시프트 후 렌더.
맵프레임 규약은 CrosswalkZones/BlockZones 와 동일(`+X=east_delta`, `+Z=north_delta`, `scene.scale.z=-1`).

## 좌표 변환 (원천 = `~/노바코스GPS변환.txt`, RSU 송신측 C++ 코드)

송신측이 **`noffX = gpsX * -10` / `noffY = y_point * 10`** 으로 채운다. 따라서:

| 항목 | 값 | 주의 |
|---|---|---|
| offsetX/Y 단위 | **dm** (0.1 m) | cm 아님 |
| offsetX 부호 | **반전** | `local_x = -offX*0.1` |
| offsetY | 그대로 | `local_y = offY*0.1` |
| 회전 | `dE = lx·cos h + ly·sin h` / `dN = -lx·sin h + ly·cos h` | h = RSU 설치 방위각 |
| h | **220도** (북=0, 시계방향) | 메시지에 없는 **사이트 상수** → launch arg `sdsm_heading_deg` |
| speed | **km/h 그대로** | J2735 표준 0.02 m/s 아님 |
| heading 필드 | **항상 ~0, 무의미** | 프레임 간 변위(≥0.3 m)로 `atan2(dy,dx)` 추정 |

검증: `~/20251128/sdsm_data` 3,722 샘플 재생 시 A2_LINK 도로망까지 중앙거리 **1.49 m**,
yaw 추정 성공률 **91.4%**. 스케일 후보 중 dm+km/h 만 변위/speed 비 1.021 (dm+0.02m/s 는 14.19).

## rviz_filter.cpp 의 변환은 틀렸다 (의도적 미수정)

`src/visualization/rviz_filter/lib/rviz_filter.cpp:344` 는 `*0.01 // cm to m` + `ref_heading = 0`.
**둘 다 오류**지만 사용자가 "rviz 디스플레이는 사용 안 하게 됐으니 그대로 두라"고 지시 → 무변경 보존.
SDSM 좌표를 다른 곳에 쓸 때 이 파일을 참고 삼지 말 것.

**Why:** 단위/부호/헤딩 셋 다 표준 J2735 해석과 어긋나서, 규격서나 기존 코드만 보고는 절대 못 맞춘다.
근거는 오직 RSU 벤더 송신 코드 한 파일뿐이라 출처를 잃으면 재현 불가.

**How to apply:**
- RSU 가 교체·재설치되면 `sdsm_heading_deg` 를 다시 맞춰야 한다. 튜닝 기준 = 오브젝트 궤적이 A2_LINK 도로망에 얹히는지.
- `objType` 은 이 RSU 가 전량 `Unknown(0)` 으로 보낸다([[reference-sdsm-bag-data]]) → 타입별 색/치수는 표시용 기본값일 뿐 신뢰 금지.
- bag 재생 시 `--topics` 는 가변인자라 **bag 경로를 먼저** 둘 것: `rosbag play -l <bag> --topics /obu/sdsm`.

## web_hmi ego yaw 규약 (이번에 확인)

`/hmi/ego_pose` · `/hmi/state.ego` 의 `yaw` = **정동(+East) 기준 반시계 라디안** (나침반 방위각 아님).
원천 `to_control_team_from_local_msg.host_yaw`, 근거 `to_control_team_demo.py:351 path_yaw = arctan2(dy_ds, dx_ds)`,
화면단 `CameraController.jsx:55 Heading_world = (cos yaw, 0, -sin yaw)`.

```
yaw[rad] = radians(90 - 방위각[deg])      # 예: 방위각 212도 → -2.1293
```

관련: [[reference-sdsm-bag-data]] (수집 데이터 소재), [[web_hmi 어댑트 알려진 함정]] (표출 안 될 때).
