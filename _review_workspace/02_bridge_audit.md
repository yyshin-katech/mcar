# bridge-auditor 보고서

검토 대상:
- `/home/katech/mcar_v13/src/visualization/web_hmi/scripts/web_hmi_bridge.py`
- `/home/katech/mcar_v13/src/visualization/web_hmi/scripts/web_hmi_threejs_bridge.py`
- 보조: `/home/katech/mcar_v13/src/visualization/web_hmi/scripts/web_server.py`
- 보조: `/home/katech/mcar_v13/src/visualization/pyqt_hmi/scripts/utils/hmi_state.py` (BaseHmiStateController)
- 보조: `/home/katech/mcar_v13/src/msgs/**/*.msg`

라이브 ROS 동작 중 확인 — `/hmi/state 10 Hz`, `/hmi/diagnostics 10 Hz`, `/hmi/objects ~10 Hz`, `/hmi/threejs/tracks ~7.3 Hz`, `/hmi/topic_hz 1 Hz` 정상 발행.

---

## 1. 토픽 매트릭스

### `web_hmi_bridge.py` (WebHmiBridge — BaseHmiStateController 상속)

| 토픽 | 방향 | 메시지 타입 | source/sink | rate | queue_size | latch |
|------|------|-----------|-------------|------|-----------|-------|
| `/hmi/state` | pub | std_msgs/String (JSON) | `_publish_periodic` 10 Hz Timer | 10 Hz | 2 | no |
| `/hmi/diagnostics` | pub | std_msgs/String (JSON) | `_publish_periodic` | 10 Hz | 2 | no |
| `/hmi/objects` | pub | std_msgs/String (JSON) | `_emit('objects_changed')` (10 Hz cap) + late drain in `_publish_periodic` | ≤10 Hz | 2 | no |
| `/hmi/popup` | pub | std_msgs/String (JSON) | `_emit('popup_changed')` (dedup) | event | 4 | yes |
| `/hmi/traffic` | pub | std_msgs/String (JSON) | `_emit('traffic_changed')` (dedup) | event | 4 | yes |
| `/hmi/bag` | pub | std_msgs/String (JSON) | `_emit('bag_state_changed')` (dedup) | event | 4 | yes |
| `/hmi/topic_hz` | pub | std_msgs/String (JSON) | `_publish_hz` 1 Hz Timer | 1 Hz | 2 | no |
| `/hmi/map` | pub | std_msgs/String (JSON) | `_publish_map_once` (init only) | once | 1 | yes |
| `/hmi/cmd/mode_request` | sub | std_msgs/Bool | `_on_mode_request` → `request_mode` | event | (default) | — |
| `/hmi/cmd/bag_toggle` | sub | std_msgs/Empty | `_on_bag_toggle` → `toggle_bag` | event | (default) | — |
| `/diagnostic/cpt7_gps` | sub (Base) | cpt7_gps_diagnostic_msg | `_cb_gps` | event | default | — |
| `/diagnostic/adcu` | sub (Base) | k_adcu_diagnostic_msg | `_cb_adcu` | event | default | — |
| `/diagnostic/lidar` | sub (Base) | lidar_diagnostic_msg | `_cb_lidar` | event | default | — |
| `/diagnostic/radar` | sub (Base) | radar_diagnostic_msg | `_cb_radar` | event | default | — |
| `/diagnostic/v2x` | sub (Base) | v2x_diagnostic_msg | `_cb_v2x` | event | default | — |
| `/diagnostic/hmi` | sub (Base) | hmi_diagnostic_msg | `_cb_hmi` | event | default | — |
| `/diagnostic/vcu` | sub (Base) | vcu_diagnostic_msg | `_cb_vcu` | event | default | — |
| `/diagnostic/cam` | sub (Base) | cam_diagnostic_msg | `_cb_cam` | event | default | — |
| `/diagnostic/ipc` | sub (Base) | ipc_diagnostic_msg | `_cb_ipc` | event | default | — |
| `/sensors/chassis` | sub (Base) | mmc_msgs/chassis_msg | `_cb_chassis` (AEB only) | event | default | — |
| `/sensors/ioniq5_ad_can` | sub (Base) | katech_custom_msgs/ioniq5_ad_can_msg | `_cb_ad_can` | event | default | — |
| `/sensors/v_can` | sub (Base) | katech_custom_msgs/v_can_msg | `_cb_v_can` | event | default | — |
| `/localization/to_control_team` | sub (Base) | mmc_msgs/to_control_team_from_local_msg | `_cb_local` | event | default | — |
| `/katri_v2x_node/katri_spat` | sub (Base) | v2x_msgs/intersection_array_msg | `_cb_traffic` | event | default | — |
| `/track_Multi_RS` | sub (Base) | perception_ros_msg/object_array_msg | `_cb_objects` | event | default | — |
| `/vehicle/mode_command` | pub (Base) | std_msgs/UInt8 | `_periodic_update` 100 ms tick | 10 Hz | 1 | no |

### `web_hmi_threejs_bridge.py`

| 토픽 | 방향 | 메시지 타입 | source/sink | rate | queue_size | latch | buff_size |
|------|------|-----------|-------------|------|-----------|-------|-----------|
| `/hmi/threejs/map` | pub | std_msgs/String (JSON) | `_publish_map_once` (init) | once | 1 | yes | — |
| `/hmi/threejs/tracks` | pub | std_msgs/String (JSON) | `_on_percept` (per /percept_topic msg) | ~7.3 Hz live | 2 | no | — |
| `/fusion_lidar_points` | sub | sensor_msgs/PointCloud2 | `_on_cloud` | event | 1 | — | 2^26 (64 MB) |
| `/percept_topic` | sub | perception_ros_msg/RsPerceptionMsg | `_on_percept` | event | 1 | — | 2^24 (16 MB) |

---

## 2. JSON payload shape (frontend-auditor 교차 비교용)

### `/hmi/state` (web_hmi_bridge.py:192-214) — 10 Hz, queue 2
키: `speed`, `gear`, `mode`, `aeb`, `steering`, `ego.{east,north,yaw}`, `gps.{rtk,lon_std,lat_std}`, `speed_limit`, `link_id`, `lane_label`, `on_odd`, `road_state`, `selected_mode`

라이브 페이로드 예: `{"speed":15.58,"gear":4,"mode":0,"aeb":false,"steering":0.3,"ego":{"east":935555.69,"north":1916352.369,"yaw":0.0336},"gps":{"rtk":2,"lon_std":0.02,"lat_std":0.013},"speed_limit":15,"link_id":20,"lane_label":"none","on_odd":0,"road_state":0,"selected_mode":0}`

### `/hmi/diagnostics` (web_hmi_bridge.py:217-219) — 10 Hz, queue 2
키: `status` (dict: `gps`, `adcu`, `lidar`, `radar`, `v2x`, `hmi`, `vcu`, `cam`, `ipc` — 각각 0/1/2)

라이브 페이로드 예: `{"status":{"gps":0,"adcu":1,"lidar":0,"radar":0,"v2x":0,"hmi":0,"vcu":0,"cam":0,"ipc":0}}`

### `/hmi/objects` (web_hmi_bridge.py:228-231) — ≤10 Hz, queue 2
키: `count`, `data[]`
- `data[i]`: `id`, `x`, `y`, `width`, `length`, `vx`, `vy`, `orientation`, `type` ("car" | "pedestrian")

주의: `width`/`length`는 BaseHmiStateController.`_cb_objects` (hmi_state.py:282-283)에서 `obj.size_y`/`obj.size_x` 0보다 클 때만 사용, 아니면 1.0 fallback. /track_Multi_RS의 object_msg는 `size_x`, `size_y` 필드명.

### `/hmi/popup` (web_hmi_bridge.py:146) — event, queue 4, latched
키: `text`, `severity` ("info" | "warn" | "error")

### `/hmi/traffic` (web_hmi_bridge.py:151-159) — event, queue 4, latched
키: `color` (0=none/1=green/2=amber/3=red), `time_decisec`, `look_at.{intersection_id, signal_group_id}`

### `/hmi/bag` (web_hmi_bridge.py:163) — event, queue 4, latched
키: `recording` (bool), `info` (str)

### `/hmi/topic_hz` (web_hmi_bridge.py:234-235) — 1 Hz, queue 2
키: BaseHmiStateController에서 `_emit('topic_event', 'gps'|'adcu'|'lidar'|'radar'|'v2x'|'hmi'|'vcu'|'cam'|'ipc')`만 호출되므로, 페이로드는 9개 키의 dict (값 = float Hz)

라이브 페이로드 예: `{"radar":10.0,"hmi":10.0,"lidar":10.0,"v2x":10.0,"vcu":10.0,"gps":10.0,"ipc":10.0,"cam":10.0,"adcu":10.0}`

### `/hmi/map` (web_hmi_bridge.py:256-258) — once, queue 1, latched
키: `polylines[][[east, north]]` — 좌표 EPSG:5179 (rospy ~map_shp 파라미터의 shapefile 그대로). 좌표 단위 [m], 평행이동 없음.

### `/hmi/threejs/map` (web_hmi_threejs_bridge.py:270-274) — once, queue 1, latched
키: `epsg` (=5179), `origin` ([east0, north0]), `layers` (dict per LAYERS_ALL)
- `layers[<name>] = { kind: "point"|"polyline"|"polygon", data: [...] }`
  - `kind=point`: `data[i] = [dx, dy]` (origin shifted)
  - `kind=polyline|polygon`: `data[i] = [[dx,dy], ...]`
- 레이어 이름: `A1_NODE`, `A2_LINK`, `A3_DRIVEWAYSECTION`, `A4_SUBSIDIARYSECTION`, `A5_PARKINGLOT`, `B1_SAFETYSIGN`, `B2_SURFACELINEMARK`, `B3_SURFACEMARK`, `C1_TRAFFICLIGHT`, `C3_VEHICLEPROTECTIONSAFETY`, `C4_SPEEDBUMP`, `C5_HEIGHTBARRIER`, `C6_POSTPOINT` (총 13개)
- 좌표: 입력 EPSG:32652 (UTM Zone 52N) → 출력 EPSG:5179 (origin shift된 dx/dy)

### `/hmi/threejs/tracks` (web_hmi_threejs_bridge.py:192-196) — ~7.3 Hz live, queue 2
키: `stamp` (float, sec), `tracks[]`
- `tracks[i]`: `id`, `tid` (둘 다 동일 `coreinfo.trakcer_id.data`), `type` ("car"|"pedestrian"), `x`, `y`, `vx`, `vy`, `size_x`, `size_y`, `orientation` (rad, atan2(direction.y, direction.x)), `confidence`, **선택** `points` (flat `[x0,y0,z0, x1,y1,z1, ...]` mm 단위 round, 최대 4096 points × 3 = 12288 floats)
- 라이브 페이로드 확인: 다수 트랙이 `points` 포함, NaN 좌표 다수 포함 (cloud_indices가 invalid 영역 가리킴) — 아래 발견 항목 참고.

---

## 3. 메시지 타입 / 필드 정합성 검증

| import / Subscriber 타입 | 위치 | .msg 검증 | 결과 |
|---|---|---|---|
| `from std_msgs.msg import Bool, Empty, String` | bridge.py:31 | std_msgs | OK |
| `from sensor_msgs.msg import PointCloud2` | threejs.py:23 | std_msgs/sensor_msgs | OK |
| `from perception_ros_msg.msg import RsPerceptionMsg` | threejs.py:37 | RsPerceptionMsg.msg | OK |
| `from mmc_msgs.msg import to_control_team_from_local_msg` | hmi_state.py:35 | to_control_team_from_local_msg.msg | OK |
| `from mmc_msgs.msg import chassis_msg` | hmi_state.py:35 | chassis_msg.msg | OK |
| `from katech_custom_msgs.msg import ioniq5_ad_can_msg, v_can_msg` | hmi_state.py:34 | OK | OK |
| `from v2x_msgs.msg import intersection_array_msg` | hmi_state.py:36 | intersection_array_msg.msg | OK |
| `from perception_ros_msg.msg import object_array_msg` | hmi_state.py:37 | object_array_msg.msg | OK |
| `from std_msgs.msg import UInt8` | hmi_state.py:24 | std_msgs | OK |
| 9개 katech_diagnostic_msgs | hmi_state.py:25-33 | OK | OK |

### 필드 접근 검증 (.msg 대조)

#### `_cb_local` (hmi_state.py:221-241) → to_control_team_from_local_msg
- `msg.Speed_Limit` ✓ (int8 Speed_Limit)
- `msg.LINK_ID` ✓ (int8 LINK_ID)
- `getattr(msg, 'lane_name', None) or getattr(msg, 'lane_id', None)` ✓ — 둘 다 .msg에 존재 (lane_name string, lane_id uint8). getattr fallback은 메시지 호환성 안전책으로 보이나, 둘 다 정의됨.
- `msg.On_ODD` ✓
- `msg.Road_State` ✓
- `msg.look_at_IntersectionID` ✓ (uint16, msg 명에 ID 대문자 — **case sensitive**)
- `msg.look_at_signalGroupID` ✓ (int8, msg 명에 mixed case)
- `msg.host_east`, `msg.host_north`, `msg.host_yaw` ✓ (float64)

#### `_cb_traffic` (hmi_state.py:243-272) → intersection_array_msg
- `msg.data[]` ✓ (intersection_msg[])
- `intersection.IntersectionID` ✓ (uint16)
- `intersection.Movements` ✓ (movement_msg, **단수 — single field, not array**)
- `movement.SignalGroupID` ✓ (uint8)
- `movement.TimeChangeDetails` ✓ (int32)
- `movement.MovementPhaseStatus` ✓ (uint8) — phase 6/8/3 매핑 SAE J2735

#### `_cb_objects` (hmi_state.py:274-290) → object_array_msg
- `msg.data[]` ✓ (object_msg[])
- `obj.id`, `obj.status`, `obj.x`, `obj.y`, `obj.size_x`, `obj.size_y`, `obj.vx`, `obj.vy`, `obj.orientation` ✓

#### `_cb_v_can` (hmi_state.py:201-210) → v_can_msg
- `msg.steering_angle` ✓ (float64)
- `msg.wheel_speed_fl/fr/rl/rr` ✓ (float64)
- `msg.gear_status` ✓ (uint8)

#### `_cb_chassis` (hmi_state.py:212-219) → chassis_msg
- `getattr(msg, 'AEB_flag', 0)` ✓ (chassis_msg.msg:23 uint8 AEB_flag)
- 주석 (hmi_state.py:213-215): `vehicle_speed`는 chassis_msg에 **없음** (legacy 코드의 fallback 0). 이 코드는 의도적으로 vehicle_speed를 무시하고 있어 정상.

#### `_cb_ad_can` (hmi_state.py:196-199) → ioniq5_ad_can_msg
- `msg.autonomous_mode` ✓ (uint8)

#### `_on_percept` (threejs.py:140-196) → RsPerceptionMsg
- `msg.lidarframe` ✓ (LidarFrameMsg)
- `lf.timestamp.data` ✓ (std_msgs/Float64)
- `lf.objects.objects[]` ✓ (Objects.objects = Object[])
- `obj.coreinfo` ✓ (CoreInfo)
- `obj.hassupplmentinfo.data` ✓ — **.msg 필드명 typo 그대로**: `hassupplmentinfo` (Object.msg:2). bridge에서 동일 typo 사용 → OK
- `obj.supplementinfo.cloud_indices` ✓ (SupplementInfo.msg:7 std_msgs/Int32[])
- `ci.trakcer_id.data` ✓ — **.msg 필드명 typo 그대로**: `trakcer_id` (CoreInfo.msg:16, Korean note 기준 typo). bridge에서 동일 typo 사용 → OK
- `ci.type.data`, `ci.center.{x,y}.data`, `ci.velocity.{x,y}.data`, `ci.size.{x,y}.data`, `ci.direction.{x,y}.data`, `ci.exist_confidence.data` ✓

#### `_cb_gps` (hmi_state.py:148-154) → cpt7_gps_diagnostic_msg
- `msg.GPSRTK_StatCode` ✓ (uint8) — 주의: `GPS_StatCode`(line 4)와 다른 필드. 코드는 RTK 코드를 사용 중이므로 OK.
- `msg.lon_std`, `msg.lat_std` ✓ (float64)

---

## 4. 발견 항목

| # | severity | 위치 | 발견 | 권장 조치 |
|---|----------|------|------|----------|
| B1 | **medium** | `web_hmi_threejs_bridge.py:148-186` | `/percept_topic`이 도착하면 그 시점의 캐시된 `_cloud_xyz`를 슬라이스하여 points 생성. 그러나 `/percept_topic`은 보통 `/fusion_lidar_points`보다 빠르거나 별도 클럭. `/fusion_lidar_points`가 ~1.5 Hz라고 주석(line 43)에 명시됐는데 `/percept_topic`은 ~7.3 Hz live → 동일 클라우드를 4-5번 다시 슬라이스할 수 있고, percept의 `cloud_indices`가 새 클라우드 인덱스 공간에서 생성됐다면 stale 클라우드와 매칭되어 잘못된 점이 보일 수 있음 (NaN 다수 발견과 일관). 시간 매칭 검증 안 됨. | `lf.timestamp`와 `cloud msg.header.stamp` 차이를 로깅하여 stale window 측정. 차이가 클 경우 `points`를 채우지 말거나, 가장 가까운 클라우드를 짝지어 보관(round-robin queue). |
| B2 | **medium** | `web_hmi_threejs_bridge.py:179-185` | `points` 슬라이스에서 NaN 좌표가 그대로 페이로드에 포함됨(라이브에서 다수 확인). NaN은 JSON 표준이 아니며 일부 JSON 파서·rosbridge·브라우저에서 파싱 실패 또는 `null`로 변환됨. Three.js에서는 NaN vertex 위치가 BufferGeometry 깨짐을 일으킬 수 있음. | `np.isfinite` 마스크로 NaN 제거 후 슬라이스, 또는 `if np.isnan(pt).any(): continue`. (frontend-auditor와 교차 점검: TrackPointClouds.jsx 측에서 NaN 처리 여부.) |
| B3 | **low** | `web_hmi_threejs_bridge.py:198-220` `_slice_points` | 이미 캐스팅된 `idx_arr` 사용 후 `cloud_xyz[idx_arr]` indexing 시 cloud_xyz는 view (`np.frombuffer().reshape`). bytes를 `arr = buf.reshape(n, stride)[:, 0:12].copy()` 한 뒤 다시 `np.frombuffer(arr, ...)` 하므로 메모리는 이미 분리. 위험 없음. 다만 `cloud_xyz`가 별 스레드에서 교체되는 동안 `_on_percept`가 인덱싱 중일 수 있어 race가능 — Python GIL상 atomic이라 큰 문제는 아님. | 명시적 lock보다 한 번에 로컬 변수로 캡쳐(`cloud = self._cloud_xyz`)하는 현재 패턴 유지. 코멘트로 "GIL atomic snapshot" 명시 권장. |
| B4 | **low** | `web_hmi_bridge.py:117-120` | `rospy.Timer` 핸들이 `self._publish_timer`, `self._hz_timer`에 저장되지만 `BaseHmiStateController.shutdown()`이 호출돼도 timer는 별도 shutdown 안 함. rospy.spin 종료 시 자동 정리되나 명시성 부족. | shutdown에서 두 timer도 `.shutdown()` 호출하도록 추가. (코드 단순성 우선이면 그대로 둬도 무방.) |
| B5 | **low** | `web_hmi_bridge.py:131-143` `_emit('objects_changed')` 처리 | 10 Hz throttle은 OK이나, `_publish_periodic`(line 222-225)에서 dirty flag 처리할 때 `time.monotonic()`을 다시 사용. 문제 없으나 `_objects_dirty=True` 상태에서 `_emit('objects_changed')`가 새로 와도 dirty만 갱신되어 publish 보장은 다음 100ms tick에서. tick과 emit의 race가 있어도 결과적으로 1tick 내 publish 보장. 문제 없음. | 변경 불필요. |
| B6 | **low** | `web_hmi_bridge.py:177-187` `_schedule_mode_pulse_reset` | 이전 timer를 `shutdown()` 호출로 정리. `try/except Exception` 광범위. shutdown 실패 시 silent 무시 → 메모리 leak 가능성 매우 작음. | 변경 불필요. |
| B7 | **info** | `web_hmi_threejs_bridge.py:36-39` | `RsPerceptionMsg` import 실패 시 단순히 None으로 두고 line 102-106에서 logerr만 띄우고 subscriber 미등록. 개발 환경 호환성 좋음. | 그대로 유지 권장. |
| B8 | **info** | `web_hmi_threejs_bridge.py:118-138` `_on_cloud` | `try/except (ValueError, AttributeError)`로 좁게 잡음. PointCloud2 디코딩 실패 시 silent return. logwarn 추가하면 디버깅 도움. | 옵션: 1회만 log하는 throttled warn 추가. 동작상 문제 아님. |
| B9 | **info** | `web_hmi_bridge.py:248-264` `_publish_map_once` | `try/except Exception`이 광범위 (BLE001 noqa 명시). map 로딩 한 번만 호출, 실패 시 logerr → 안전. | 그대로 유지. |
| B10 | **medium** | `web_hmi_bridge.py:117` `/hmi/state` queue_size=2 | 10 Hz publish에 queue=2는 짧음. rosbridge 전송 지연 시 drop 가능성. 다만 latest-state 의미상 drop OK. | 변경 불필요. (현재 라이브 측정 10.0 Hz 정확.) |
| B11 | **info** | `web_hmi_bridge.py:200-202` `ego` JSON shape | `host_yaw`는 to_control_team_from_local_msg.msg:26 주석상 `[rad]`. round 4자리. 라이브 yaw 0.0336 (rad) 확인 — 단위 일관. wrap 범위는 source 측 결정. | OK. |
| B12 | **info** | `web_hmi_bridge.py:203-207` `gps` shape | `lon_std`, `lat_std`는 cpt7_gps_diagnostic_msg:10-11 float64 [m로 추정]. round 4자리 (= 0.1mm). 라이브 0.02, 0.013 확인 (cm scale). 단위는 .msg에 명시 안 됨 → 발견 항목으로 기록. | .msg 정의에 단위 주석 추가 권장 (다른 에이전트 영역). |
| B13 | **low** | `web_hmi_threejs_bridge.py:149-152` cloud snapshot | `_on_percept` 진입 시 `cloud = self._cloud_xyz; n_cloud = self._cloud_n` 로 한 번 캡쳐. 그러나 race로 `n_cloud != cloud.shape[0]`이 될 수 있음 (writer가 둘을 비원자적으로 갱신, threejs.py:135-136). 언동기 race이지만 GIL 덕에 부분 갱신은 단일 명령 단위. | `_cloud_xyz`만 캡쳐하고 `n_cloud = cloud.shape[0]`으로 계산하면 race 제거. |
| B14 | **info** | `web_hmi_threejs_bridge.py:152` | `lf.objects.objects` (Objects.msg:1 `Object[] objects`) — 이중 'objects' 명명 정확. | OK. |
| B15 | **info** | `web_hmi_threejs_bridge.py:163` `percept_type_str` | `type==1 → pedestrian`, else "car". 주석에 "Refine when other type ids confirmed"로 의도 노출됨. | OK. |
| B16 | **info** | 좌표계/단위 일관성 종합 | `_cb_local` 의 host_east/north [m] (EPSG:5179), host_yaw [rad]. /hmi/state 그대로 전달. /hmi/map polylines [m] EPSG:5179. /hmi/threejs/map은 EPSG:32652→5179 변환 후 origin shift된 dx/dy [m]. /hmi/threejs/tracks의 x,y는 percept 원본 lidarframe 좌표(/base_link 기준 [m] 추정, 단위 명시 없음). yaw 변환 atan2 → [-π,π] 범위. | OK. |
| B17 | **medium** | `web_hmi_threejs_bridge.py:140-146` `_on_percept` 진입부 | `try/except AttributeError: return` — 페이로드 구조 깨질 때 silent drop. `lf.timestamp.data` 추출만 보호. 이후 line 156, 176의 except도 silent continue. 비정상 메시지를 디버그할 수 없음. | rospy.logwarn_throttle(5.0, ...) 추가. |
| B18 | **info** | `web_hmi_bridge.py:131` `_emit('topic_event', args[0])` | BaseHmiStateController에서 9개 토픽만 `topic_event` emit (gps, adcu, lidar, radar, v2x, hmi, vcu, cam, ipc). Hz tracker key set은 정확히 9개. JSX의 useTopicHz가 더 많은 키를 기대하면 fallback 0. | OK (frontend-auditor 비교 필요). |
| B19 | **low** | `web_hmi_bridge.py:79-87` `popup`, `traffic`, `bag` queue_size 4 + latch=True | event 토픽 + latch는 늦게 join하는 subscriber에 마지막 상태 전달. 적절. queue 4는 dedup이 매번 필터하므로 충분. | OK. |
| B20 | **info** | 토픽 매트릭스 일관성 | `WebHmiBridge`는 11개 pub + 16개 sub(BaseHmiStateController 포함). `WebHmiThreejsBridge`는 2개 pub + 2개 sub. **두 브리지가 모두 launch에서 활성화될 때 `/hmi/threejs/map`/`tracks`는 web_hmi_threejs_bridge만 발행** (충돌 없음). | OK. |

---

## 요약

- 메시지 타입/필드 접근은 모두 .msg와 일치 (typo `trakcer_id`, `hassupplmentinfo`도 .msg 그대로 사용 — OK).
- 토픽 매트릭스에 누락 없음. 라이브 ROS에서 모든 발행 토픽 정상 publish 확인.
- **medium 발견 4건** (B1, B2, B10 부분, B17): /percept_topic ↔ /fusion_lidar_points 시간 동기 미검증, NaN 좌표 페이로드 포함, except가 silent.
- **low/info 다수**: 대체로 안전한 패턴.
- 좌표계 단위 일관성 OK (EPSG:5179 m, rad). yaw wrap 범위는 source(to_control_team) 결정.
- frontend-auditor 비교 대상 JSON 키 인벤토리 위 §2 참고.
