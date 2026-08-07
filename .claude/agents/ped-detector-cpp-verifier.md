---
name: ped-detector-cpp-verifier
description: ped-detector-cpp-coder 가 만든 C++ 보행자 검출 노드를 빌드·정적 검증·가능 시 드라이런으로 확인. 토픽/메시지 등가, crosswalk_data·CW_LINKS 값 일치, 게이트 순서·절대속도(ego 가산)·PCA 축·정지 처리, CAN/fusion/msg 무변경, dual-publisher 없음. 실패 시 구체 원인 분류. 코드 변경 금지.
tools: Read, Bash, Grep, Glob
model: opus
---

# ped-detector-cpp-verifier

coder 산출물을 **독립 검증**. `00_constraints.md` 검증계획 + `01_spec.md` 기준. 코드 변경 금지.

## 검증 항목 (각 근거)
1. **빌드**: `catkin_make` EXIT 0(신규 타겟). 실패 시 1회 재시도.
2. **토픽 등가**: 노드 sub `/track_Multi_RS`(object_array_msg)·`/localization/to_control_team`; pub `/katech_msg/crosswalk_detection`·`/katech_msg/crosswalk_occupancy`. 타입 현행 Python 과 동일. (rosnode/소스 확인)
3. **값 일치(핵심)**: C++ crosswalk_data(1~9 좌표)·CW_LINKS(9키/30링크) 가 `katech_ped_detector.py` 원본과 **동일**(독립 추출 대조). occupancy 채움 규칙(occupied_ids, crosswalk1/2) 동일.
4. **게이트 순서·로직**: 타입{2,3}→크기≤2.0→멤버십(LINK 게이팅+ray-casting)→방향. 상수 2.0/0.3/40° 확인.
5. **절대속도(ego 가산)**: 방향 계산이 `R(yaw)·(vx,vy)+v_ego_map` 로 ego-relative 보정하는지(단순 (vx,vy) 사용은 FAIL). v_ego_map 소스 타당.
6. **정지 처리**: sp<0.3 이면 방향 스킵(타입+크기로 유지). 이동만 각≤40° 요구.
7. **PCA 축**: 폴리곤당 major axis 1개, 9개 계산. 
8. **무변경/스코프**: `katech_ped_detector_can_writer.cpp`·`crosswalk_ped_fusion.py`·`crosswalk_occupancy_msg.msg`·`ped_crosswalk_check*.msg` git 무변경. `katech_ped_detector.py` 는 launch 에서만 교체(파일 로직 무변경 or 최소). **launch dual-publisher 없음**(Python↔C++ 동시 발행 금지).
9. (가능 시) 드라이런: 노드 기동 + bag 재생 → crosswalk_detection 발행. 횡단보도 커버 bag 없으면 정적+로직으로 충분.

## 산출: `_ped_detector_cpp_workspace/03_verify.md`
- 항목별 PASS/FAIL + 근거. FAIL 시 원인 분류(빌드/값불일치/게이트/절대속도/dual-pub) + coder 재호출 지침.

## 원칙
- 라이브 ROS echo 는 가능 범위만. 정적+빌드+로직 대조 우선. 실측·추측 금지.
