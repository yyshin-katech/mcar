---
name: ped-detector-cpp
description: 횡단보도 보행자 on_crosswalk 검출 Python→C++ 포팅 + 크기·방향 게이트 하네스. 객체 vx,vy=ego-상대(절대속도 계산법), PCA 길이축, 게이트 순서
metadata:
  node_type: memory
  type: project
  originSessionId: c2a74392-f1b6-49d8-91ad-e78f7a6568f9
---

`katech_ped_detector.py`(횡단보도 보행자 `on_crosswalk` 검출)를 **C++ 노드로 포팅 + 크기 게이트 + 방향 게이트** 추가. 하네스 `ped-detector-cpp`(2026-07-21, siheung_dev). 관련 [[crosswalk_position]], [[perception_object_type]].

## 결과물 (2026-07-21)
- **신규** `src/sensing/can/src/katech_ped_detector.cpp`(target `katech_ped_detector`). Python `katech_ped_detector.py` 는 **파일 유지, launch 에서만 교체**(`katech_test.launch` line 57 `type` `.py`→C++, node name 동일 → **dual-publisher 없음**). Python 이 유일 기동 지점이었음.
- 동작 등가 보존: 토픽(sub `/track_Multi_RS`=perception_ros_msg::object_array_msg·`/localization/to_control_team` / pub `/katech_msg/crosswalk_detection`·`/katech_msg/crosswalk_occupancy`), crosswalk_data(1~9)·CW_LINKS(§134)·ray-casting·LINK 게이팅·occupancy(occupied_ids+crosswalk1/2) 전부 동일(값 오차 0). CAN writer·fusion·msg 무변경.
- CMakeLists(can): 신규 노드용 `find_package`/`catkin_package` 에 **perception_ros_msg·katech_custom_msgs 추가**(기존 CAN writer 는 공유 devel/include 로 우연히 빌드됐음). package.xml depend 3쌍.

## 게이트 (신규, 순서 타입→크기→멤버십→방향)
1. **타입**: `status ∈ {2,3}`(PED,BIC). (status=ObjectType, [[perception_object_type]])
2. **크기**: `max(size_x,size_y) ≤ 2.0 m`(BIC≈5m·CAR≈10m 배제, 실질 PED 소형만).
3. **멤버십**: ego LINK_ID∈CW_LINKS[N] 인 크로스워크만 ray-cast(절대좌표 `host+R(yaw)·(x,y)`).
4. **방향**: 이동객체(sp≥0.3)만 — 절대속도 방향과 크로스워크 **PCA major axis(길이축)** 사이각 `acos(|v̂·axis|) ≤ 40°`(횡단). 정지(sp<0.3)는 **방향 스킵, 타입+크기로 유지**(서 있는 보행자 보호). 통과 차량(수직)=배제.
- 상수 `ped_size_max=2.0/move_min_speed=0.3/cross_angle_max_deg=40`(ros::param 튜닝).

## 핵심 함정 — 객체 vx,vy = **ego-상대 속도**
- 검증: ego 10.2m/s 시 객체 중앙속도 11.4m/s(ratio 1.12) — 정지물이 ego 속도로 뒤로 흐름 → **ego-상대**. 단순 (vx,vy) 로 방향판정 시 전면 오판.
- **절대속도** `v_map = R(host_yaw)·(vx,vy) + v_ego_map`. `v_ego_map` = host_east/north **유한차분**+EMA(α=0.3, dt가드) — `to_control_team_from_local_msg` 에 host 속도/speed 필드 **없음**(host_e/n/yaw+LINK_ID+MANUAVER 뿐).
- object_msg 는 **flat 구조**(std_msgs 래핑 없음, `o.x/o.vx/o.size_x` 직접). (std_msgs 래핑은 CoreInfo 계열 얘기, object_msg 엔 무관.)

## 크로스워크 길이축
폴리곤 vertices PCA(공분산 고유벡터/SVD) **major axis** = 길이방향(보행자 횡단방향). ctor 에서 9개 사전계산. 부호 무관(`|dot|`). 보행자 횡단=축평행(각 작음), 차량 통과=축수직(각 큼).

## 의도된 Python 대비 차이 2건(regression 아님)
① detection 엔트리 객체별 독립값(Python 의 ped_msg 참조 aliasing 버그 자연 교정, 엔트리 수 동일). ② occupancy 를 **완전 게이트 통과** 객체로 산출(Python 은 타입+멤버십만). zero-entry(타입객체 0→size1) / 빈배열(전원 탈락→size0) 규칙은 CAN writer 분기 정합 위해 보존.

## ⚠️ 원복 (2026-07-21) — 크기 게이트 제거 + 타입 원복
web_hmi 가 차량(type3=BIC≈5m)을 사람으로 표시 → 게이트 변경: **타입 `status∈{2,3}`→`{1,2}`(이전버전) 원복 + 크기 게이트 제거**(사용자 "진행방향만 고려"). **현재 순서 = 타입{1,2} → 멤버십(LINK) → 방향(≤40°, 정지 스킵)**. `ped_size_max_` param 은 코드에 남지만 미사용. 분류 원복 상세 [[perception-object-type]].
