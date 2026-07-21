---
name: perception-object-type
description: RoboSense perception ObjectType enum(정수 class) 실제 값 + status=type 사실 + KATECH 코드의 1=보행자 오분류 수정 이력
metadata:
  node_type: memory
  type: reference
  originSessionId: c2a74392-f1b6-49d8-91ad-e78f7a6568f9
---

RoboSense(RS) perception SW 의 **실제 ObjectType enum**(사용자 소스 제공, authoritative). `perception_ros_msg` = RS 출력. 관련 [[crosswalk_position]], [[percept_filter_policy]].

## ObjectType (coreinfo.type = /track_Multi_RS·matcher 의 object_msg.status)
```
0=UNKNOW  1=CONE  2=PED  3=BIC  4=CAR  5=TRUCK_BUS  6=ULTRA_VEHICLE  (7=MAX)
```
- **status = type**: `lidar_object_publisher_v2.py:282 msg.status = features['obj_type']`, `percept_topic_matcher.cpp:139 object_status = coreinfo.type.data`. 즉 `/track_Multi_RS` 의 `object_msg.status` 는 트래킹 상태가 아니라 **class(ObjectType)** 다. (object_msg 에 별도 type 필드 없음.)
- bag(20260508) 실측 분포: 1(CONE)15.5% · 2(PED)7.8% · **3(BIC)54%** · 4(CAR)12.9% · 6(ULTRA)9.8% (0/5 미출현). BIC 비율 높음.
- 웹 공개문서엔 정수 enum 없음(상용 RS-Fusion SDK 내부). RS-Fusion-P6 카테고리: 보행자/자전거탑승자/대형·소형차량/트레일러/콘/스톤파이어.

## KATECH 코드의 1=보행자 오분류 (수정함, 2026-07-21)
원인: 코드가 `type/status==1 → 보행자` 로 가정했으나 실제 1=CONE, PED=2. → 콘을 보행자로, 진짜 보행자(2)·자전거(3)를 차량으로 처리하던 버그.
**PED(2)+BIC(3)=보행자** 로 정정한 5곳(`==1` → `∈{2,3}`):
1. `web_hmi_threejs_tracks_node.cpp` `percept_type_str`(C++, /hmi/threejs/tracks)
2. `web_hmi_threejs_bridge.py` `percept_type_str`(Python, guarded)
3. `pyqt_hmi/utils/hmi_state.py:338` `obj_type`(/hmi/objects + pyqt 컨트롤러 공용)
4. `katech_ped_detector.py` `status in [1,2]`→`[2,3]`(CAN 횡단보도 보행자 검출)
5. `percept_topic_matcher.cpp:206` `is_pedestrian`(CAN, 사라진 보행자 2초 occlusion 캐시)
- **미수정(대기)**: `pyqt_hmi/widgets/main_window.py:666` `obj.status == 1` — pyqt main_window 는 사용자 스코프 제외라 보류. 고치려면 `obj.status in (2,3)`.
- C++ 2개(1,5) `catkin_make` 재빌드 필요. `percept_type_str` 은 이진(pedestrian/car)이라 자전거도 pedestrian 으로 합쳐짐.

## 함정
- object_msg.status 를 "트래킹 상태"로 오해 금지 — class 다. 값집합 {1,2,3,4,6}(0/5/7 드묾)이면 ObjectType.
- 새 소비자 작성 시 보행자류 = {2,3}, 차량 = {4,5,6}, 콘=1, 미상=0 로 매핑.
