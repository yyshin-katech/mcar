---
name: web_hmi 어댑트 알려진 함정
description: web_hmi 화면이 안 나올 때의 함정 모음 (LAYER_STYLE 동기화 / polyline alpha 무시 / bag /hmi/* 충돌 / 캐시된 구 HTML → React #130 / ego_pose JSON 깨짐 → 카메라 이탈)
type: feedback
originSessionId: 3ea36247-9ca8-4af1-aa8b-d69a44426fbf
modified: 2026-09-09T00:00:00.000Z
---
web_hmi 어댑트는 ROS 측 LAYERS_ALL/launch만 패치하면 화면이 안 나온다. 두 가지 추가 검증 필수:

## 1. 프론트 LAYER_STYLE 동기화 누락

`web_hmi_threejs_bridge.py:LAYERS_ALL`에 새 layer 키를 추가하면 `web/threejs/types.js:LAYER_STYLE` + `DEFAULT_LAYER_VIS`에도 반드시 같은 키를 추가해야 한다. 누락 시 `web/threejs/MapLayers.jsx:60`의 `if (!style) return null` 분기로 빠져 아무것도 그려지지 않는다 (ROS 페이로드는 정상이지만 브라우저는 빈 화면).

**Why:** LAYER_STYLE은 K-City NGII 표준 13개 layer (A1_NODE, A2_LINK, B3_SURFACEMARK 등)만 정의. siheung_dev처럼 새 layer 키(TB_senario_map, TB_senario_surfaceMARK)는 별도 등록 필요. 2026-05-08 2차 어댑트에서 이 함정에 빠짐.

**How to apply:**
- web_hmi 어댑트 시 `LAYERS_ALL`과 `LAYER_STYLE` 동기화를 검증 단계의 필수 체크로 둘 것.
- types.js는 정적 파일이라 수정 후 노드 재기동 불필요. **브라우저 강제 새로고침 (Ctrl+Shift+R)** 필요 — index_threejs_*.html에 cache-busting 쿼리스트링이 없어 일반 새로고침은 캐시 그대로 받음.
- match-detective가 LAYERS_ALL만 보고 LAYER_STYLE은 안 보므로 `bridge-adapter` 또는 `adapt-verifier`에 LAYER_STYLE 매칭 검사를 추가하는 게 좋음.

## 1b. LAYER_STYLE `alpha` 는 polygon 전용이었음 (polyline 무시)

지도 레이어 색/두께/투명도의 **단일 원천은 `web/threejs/types.js:LAYER_STYLE`** 이다. 단, `alpha` 는 원래 `buildPolygonOutline` 만 읽었고 `buildPolyline` 은 `color`/`width` 만 받아 **polyline 레이어에 `alpha` 를 써도 아무 효과가 없었다**. 2026-07-27 `MapLayers.jsx:buildPolyline` 에 `alpha` 인자를 추가(`alpha != null && < 1.0` 일 때만 `transparent+opacity`)해 지원. `alpha` 미지정 레이어는 종전대로 불투명.

**Why:** "A2 Lane lines 를 연하게" 요청 시 types.js 값만 바꾸면 반영될 것처럼 보이지만 빌더가 무시한다. 실제로 두 파일을 함께 고쳐야 한다.

**How to apply:**
- polyline 레이어(A2_LINK, B2_SURFACELINEMARK, TB_senario_map, C3, C5) 농도 조절 = `types.js` `alpha` 값 하나. 현재 `A2_LINK: alpha 0.2`(연함), 나머지는 미지정=불투명.
- `width`(linewidth) 는 대부분의 WebGL 환경에서 1px 로 고정되어 굵기 조절이 안 되므로, 강조/약화는 색상+`alpha` 로 한다.
- 정적 파일이라 catkin 빌드·노드 재기동 불필요. **Ctrl+Shift+R** 만 하면 됨(위 1번과 동일).

## 2. bag 재생 시 /hmi/* latched 토픽 충돌

bag 파일에 `/hmi/map`, `/hmi/threejs/map` (등 다른 `/hmi/*`) 토픽이 함께 녹화되어 있으면 재생 시 `/play_*` 노드가 옛 페이로드를 latched로 publish해 web_hmi 노드의 새 페이로드를 덮어쓴다. 결과: 새 노드 띄워도 /hmi/threejs/map echo가 옛 layer 키 그대로.

**Why:** `/hmi/*`는 latched 토픽이라 가장 최근 publish가 살아남는다. bag publish 빈도가 web_hmi의 1회성 publish보다 자주면 항상 옛 데이터로 덮임. 2026-05-08 2차 어댑트 검증에서 bag publisher가 `/hmi/threejs/map`을 publish 중인 사실을 토픽 publisher 목록(`rostopic info`)으로 확인.

**How to apply:**
- web_hmi 검증 시 `rostopic info /hmi/threejs/map` 등으로 publisher가 web_hmi 노드만인지 확인. `/play_*`가 보이면 bag 재생 옵션 수정 필요.
- 권장 옵션: `rosbag play <bag> /hmi/map:=/dev/null/hmi_map /hmi/threejs/map:=/dev/null/threejs_map` (output 토픽 remap으로 bag publish 무력화).
- 더 깨끗한 옵션: `rosbag play --topics <입력 토픽만>` (예: `/siheung_spat /sensors/v_can /sensors/ioniq5_ad_can /ublox/navpvt /percept_topic /track_Multi_RS /fusion_lidar_points`).
- adapt-verifier가 publisher 목록 검사를 검증 항목에 포함하면 이 함정 자동 감지 가능.

## 3. 새 JSX 컴포넌트 추가 + 캐시된 구 HTML → React #130 으로 **트리 전체 사망**

`index_threejs_*.html` 에 `<script>` 태그를 추가하고 화면 JSX 에서 `window.NewComp` 를 마운트했는데
브라우저가 **구 HTML 을 캐시**하고 있으면, 스크립트 태그 없이 새 JSX 만 로드돼 `window.NewComp === undefined`.
결과는 그 컴포넌트만 빠지는 게 아니라 `Minified React error #130 (element type is invalid ... got: undefined)` 로
**렌더 트리 전체가 죽는다 — 지도까지 통째로 사라진다.** 증상이 "지도 shp 로딩 안 된 것 같다" 로 보여서 백엔드를 엉뚱하게 파게 된다.

**Why:** `web_server.py` 가 `SimpleHTTPRequestHandler` 라 `Cache-Control` 을 안 붙였고, HTML 에 cache-busting 쿼리스트링도 없어
브라우저 휴리스틱 캐싱이 걸린다. JSX 는 `text/babel` 로 매번 새로 받는데 HTML 만 옛것이라 **조합이 어긋난다**.

**How to apply:**
- 2026-09-09 근본 조치 2건 적용됨(commit afc8377): `web_server.py` 응답에 `Cache-Control: no-store` 추가,
  `ThreejsF1Screen.jsx` 의 신규 마운트를 `{window.NewComp && <window.NewComp ... />}` 존재 가드로 감쌈.
- **앞으로 `window.*` 컴포넌트를 마운트할 때는 항상 존재 가드를 붙일 것.** 한 컴포넌트 누락이 화면 전체를 끄는 걸 막는다.
- 진단법: 헤드리스로 `--dump-dom` 떠서 DOM 크기를 정상본과 비교(죽으면 눈에 띄게 작다) + console 에 `#130` 있는지 확인.
- `web_dir` 은 `$(find web_hmi)/web` = **소스 디렉토리**이고 `devel/lib/web_hmi/*.py` 는 소스를 exec 하는 릴레이 스텁이라,
  HTML/JSX/파이썬 스크립트 수정에 catkin 빌드는 불필요하다(노드 재기동만).

## 4. `/hmi/ego_pose` JSON 이 깨져도 **조용히 무시** → ego (0,0) → 카메라가 지도에서 2,000 km 밖

`ros_bridge.jsx:143 useJsonTopic` 은 `JSON.parse` 실패를 `catch {}` 로 삼킨다. 그래서 페이로드가 깨지면
에러 없이 기본값 `{east:0, north:0, yaw:0}` 이 유지되고, 맵 원점은 `[931291.7, 1929703.19]` 이라
**카메라가 지도에서 약 2,000 km 떨어진 곳을 본다. 지도도 오브젝트도 화면 밖 = 아무것도 안 보인다.**

실제 사고: 수동 테스트 시 쉘 이스케이프가 깨져 `{\east":931307.1,...}` 가 발행됨(여는 따옴표 소실).

**How to apply:**
- 수동 ego 주입은 작은따옴표로 전체를 감쌀 것:
  `rostopic pub -r 50 /hmi/ego_pose std_msgs/String 'data: "{\"east\": 931307.1, \"north\": 1931256.8, \"yaw\": -2.1293}"'`
- 발행 후 **반드시 `rostopic echo -n1 /hmi/ego_pose` 로 `\"east\"` 가 온전한지 눈으로 확인.**
- 화면이 비면 백엔드 의심 전에 `/hmi/ego_pose` 페이로드부터 볼 것. `/hmi/threejs/map` 바이트 수(정상 약 2 MB)가 나오면 백엔드는 무죄다.
- yaw 규약은 정동 기준 CCW 라디안 → [[project-sdsm-web-hmi]] 참조.
