---
name: project-web-hmi-camera
description: web_hmi Three.js 카메라 — vendor three r160 UMD 라 OrbitControls 조달 불가(자체 orbit 컨트롤러 유지), 씬 좌표 규약, lookAt 행렬 함정, 헤드리스 검증 레시피
metadata:
  type: project
---

`web/threejs/CameraController.jsx` (모드 `iso` / `top` / `orbit`).

**OrbitControls 를 새로 가져오려 하지 말 것.** `web/vendor/three.min.js` 는 **r160 UMD 레거시 빌드**
(`THREE.REVISION === 160`) 이고, three 는 **r148 에서 `examples/js` 를 삭제**해서 UMD 용
OrbitControls 자체가 존재하지 않는다. `examples/jsm` 을 쓰려면 three ES 모듈 사본이 추가로 필요해
THREE 인스턴스가 이중화된다(빌드 파이프라인 없는 in-browser Babel 구조라 더 위험). 그래서
**자체 컴팩트 orbit 컨트롤러를 직접 구현해 유지**하는 것이 확정 방침이다 (2026-09-09).

**바인딩·감도는 rviz `OrbitViewController` 와 동일하게 맞춰 둠** — 좌드래그 회전
(`yaw += dx*0.005`, `elevation += dy*0.005` rad/px), 중클릭·Shift+좌 = pan(지면을 잡고 끄는 방향),
우클릭·Ctrl+좌 = dolly(아래로 = 멀어짐), 휠 = dolly, 더블클릭 = 직전 프리셋 복귀.
캔버스를 드래그하면 **현재 프리셋에서 seed 되어 orbit 으로 자동 진입**(시점 점프 0),
초점은 rviz target frame 처럼 자차를 계속 추종한다. ego 는 `/hmi/state`(10 Hz) 가 아니라
`/hmi/ego_pose`(~50 Hz) 를 쓴다 — [[project-qt-hmi]] 의 50 Hz ego pose 패턴과 동일.

**씬 좌표 규약 (카메라·오버레이 수학의 전제):** `+X=east`, `+Z=north`, `+Y=up` 이되
`scene.scale.z = -1` 이고 **카메라는 씬의 자식이 아니라 월드 공간**에 있다. 따라서 카메라 좌표에서는
`worldZ = -(north delta)`. orbit 의 `panN` 갱신이 `o.panN -= wz` 인 이유가 이것이다.

**three 함정:** `Object3D.lookAt()` 은 **쿼터니언만** 쓰고 `obj.matrix` 는 갱신하지 않는다.
렌더 루프의 `renderer.render()` 가 `updateMatrixWorld()` 를 부를 때까지 `cam.matrix` 는 낡은 값이라,
포인터 핸들러에서 `setFromMatrixColumn(cam.matrix, 0)` 으로 right 축을 읽으면 한 프레임 밀리거나
첫 렌더 전에는 단위행렬이 나와 pan 방향이 어긋난다. → **`cam.updateMatrixWorld()` 후
`cam.matrixWorld` 에서 축을 읽을 것.**

**헤드리스 검증 레시피** (합성 PointerEvent 로 실제 드래그 → 카메라 좌표 수치 검증. 26 항목 PASS):
`web/` 아래에 임시 테스트 HTML 을 두고 `window.useThree/useJsonTopic/useEgoPose` 를 스텁으로 덮은 뒤
`CameraController.jsx` 만 로드하면 ROS 없이 검증된다. ego 추종 검증에는 스텁을 **React state 훅**으로
만들어야 한다(객체만 mutate 하면 리렌더가 없어 `useEffect(apply)` 가 안 돈다).

```bash
python3 -m http.server 8099 --bind 127.0.0.1     # web/ 에서
google-chrome --headless=new --no-sandbox \
  --use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader \
  --virtual-time-budget=8000 --dump-dom "http://127.0.0.1:8099/<page>"
```
- swiftshader 3종 플래그가 없으면 `Error creating WebGL context` 로 ThreeScene 이 죽어 canvas 가 안 생긴다.
- `--virtual-time-budget` 이 없으면 `--dump-dom` 이 load 직후 떠서 비동기 테스트 결과를 못 받는다.
- 실제 페이지(`index_threejs_f1.html` / `index_threejs.html`) 회귀 확인에도 그대로 쓴다 —
  JS 치명 오류 0건 + `<canvas>` 존재를 보면 [[web-hmi-adapt-pitfalls]] 의 React #130 트리 사망을 잡을 수 있다.
