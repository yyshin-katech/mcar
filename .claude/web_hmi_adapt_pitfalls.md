---
name: web_hmi 어댑트 알려진 함정
description: web_hmi 어댑트 시 ROS 측 패치만으로 화면이 안 나오는 두 가지 함정 (LAYER_STYLE 동기화 + bag /hmi/* 충돌)
type: feedback
originSessionId: 3ea36247-9ca8-4af1-aa8b-d69a44426fbf
---
web_hmi 어댑트는 ROS 측 LAYERS_ALL/launch만 패치하면 화면이 안 나온다. 두 가지 추가 검증 필수:

## 1. 프론트 LAYER_STYLE 동기화 누락

`web_hmi_threejs_bridge.py:LAYERS_ALL`에 새 layer 키를 추가하면 `web/threejs/types.js:LAYER_STYLE` + `DEFAULT_LAYER_VIS`에도 반드시 같은 키를 추가해야 한다. 누락 시 `web/threejs/MapLayers.jsx:60`의 `if (!style) return null` 분기로 빠져 아무것도 그려지지 않는다 (ROS 페이로드는 정상이지만 브라우저는 빈 화면).

**Why:** LAYER_STYLE은 K-City NGII 표준 13개 layer (A1_NODE, A2_LINK, B3_SURFACEMARK 등)만 정의. siheung_dev처럼 새 layer 키(TB_senario_map, TB_senario_surfaceMARK)는 별도 등록 필요. 2026-05-08 2차 어댑트에서 이 함정에 빠짐.

**How to apply:**
- web_hmi 어댑트 시 `LAYERS_ALL`과 `LAYER_STYLE` 동기화를 검증 단계의 필수 체크로 둘 것.
- types.js는 정적 파일이라 수정 후 노드 재기동 불필요. **브라우저 강제 새로고침 (Ctrl+Shift+R)** 필요 — index_threejs_*.html에 cache-busting 쿼리스트링이 없어 일반 새로고침은 캐시 그대로 받음.
- match-detective가 LAYERS_ALL만 보고 LAYER_STYLE은 안 보므로 `bridge-adapter` 또는 `adapt-verifier`에 LAYER_STYLE 매칭 검사를 추가하는 게 좋음.

## 2. bag 재생 시 /hmi/* latched 토픽 충돌

bag 파일에 `/hmi/map`, `/hmi/threejs/map` (등 다른 `/hmi/*`) 토픽이 함께 녹화되어 있으면 재생 시 `/play_*` 노드가 옛 페이로드를 latched로 publish해 web_hmi 노드의 새 페이로드를 덮어쓴다. 결과: 새 노드 띄워도 /hmi/threejs/map echo가 옛 layer 키 그대로.

**Why:** `/hmi/*`는 latched 토픽이라 가장 최근 publish가 살아남는다. bag publish 빈도가 web_hmi의 1회성 publish보다 자주면 항상 옛 데이터로 덮임. 2026-05-08 2차 어댑트 검증에서 bag publisher가 `/hmi/threejs/map`을 publish 중인 사실을 토픽 publisher 목록(`rostopic info`)으로 확인.

**How to apply:**
- web_hmi 검증 시 `rostopic info /hmi/threejs/map` 등으로 publisher가 web_hmi 노드만인지 확인. `/play_*`가 보이면 bag 재생 옵션 수정 필요.
- 권장 옵션: `rosbag play <bag> /hmi/map:=/dev/null/hmi_map /hmi/threejs/map:=/dev/null/threejs_map` (output 토픽 remap으로 bag publish 무력화).
- 더 깨끗한 옵션: `rosbag play --topics <입력 토픽만>` (예: `/siheung_spat /sensors/v_can /sensors/ioniq5_ad_can /ublox/navpvt /percept_topic /track_Multi_RS /fusion_lidar_points`).
- adapt-verifier가 publisher 목록 검사를 검증 항목에 포함하면 이 함정 자동 감지 가능.
