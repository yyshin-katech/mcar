---
name: reference_diag_replay_harness
description: CANoe 익스포트 mat → /sensors/v_can rosbag 재생으로 vcu_diagnostic 고장진단 검증하는 하네스
metadata: 
  node_type: memory
  type: reference
  originSessionId: d162f4d6-98ba-4894-8614-68dfd20625f0
---

## 하네스 위치
`~/diag_replay_sample/` (repo 외부)
- `make_vcan_bag.py` — CANoe 익스포트 .mat → `/sensors/v_can` rosbag. `--fault`로 steering life_count @30s 동결 주입
- `label_segments.py` — 고장구간 라벨링 → `fault_segments.csv` + `vcan_replay_labeled.bag`(+ `/fault_ground_truth`)
- `vcan_replay_normal.bag` / `_fault.bag` / `_labeled.bag`, `README.md`
- 원본: `~/2026-06-01_10-41-15_071_Diagnostic_S_LG_LA.mat`

## 이 mat의 재현 고장 구조 (ground truth)
3 하위제어기(슈퍼바이저/종/횡) 고장을 **슈퍼바이저→종→횡 2주기**로 재현. 파일명 S_LG_LA.
- 식별: 각 제어기 **`*State` 신호 침묵** + 슈퍼바이저 `AutonomousState__error_code`(A=침묵, B=16/56/48, C=4)
- 구간: A 11.1–18.0/38.7–44.1, B 18.8–25.7/40.1–52.6, C 29.8–36.4/53.8–59.0
- 주의: `*Control`/`*life_count`(LongitudinalControl 등)는 안 끊김. 끊기는 건 `*State`. VCU `*Info` 끊김은 collateral

## 핵심 사실
- CANoe .mat은 **MATLAB v7.3(HDF5)** → scipy 불가, `h5py` 필요. 각 신호 `(2,N)`: row0=시각(초), row1=값
- `vcu_diagnostic`은 v_can_msg의 **6종 life_count**(gear/turnsignal/longitudinal/steering/**wheel/dynamic**info)만 읽어 0.5s 내 변화 없으면 `/diagnostic/vcu` `VCU_StatCode=1`. error_code는 안 봄
- 이 익스포트엔 **WheelInfo/DynamicInfo(CAN FD) 없음** → 정상판정 위해 life_count만 증가 합성
- 실측 자체가 연속 아님: **LongitudinalInfo가 자주 dropout(최대 12s 공백)** → normal 재생도 ~49% stale(실제 고장 이벤트). 깨끗한 정상 baseline 원하면 life_count 연속 합성 필요

## 검증 방법
`roslaunch launch/diagnostic_only.launch` (vcu_diagnostic 기동) → `rosbag play` 1배속 → `/diagnostic/vcu/VCU_StatCode` 관찰. fault bag은 30.8s부터 StatCode=1 고정 확인됨.

**Why:** 실차/CANoe 로그로 진단노드를 오프라인 재생 검증할 수 있음(HW 없이). [[canoe_tool]] 캡처와 짝.

**How to apply:** 새 CANoe mat 받으면 신호명(`Group__signal`) 확인 후 make_vcan_bag.py의 zoh 매핑 갱신. 다른 진단노드(GPS=/ublox/navpvt 등)는 해당 입력 토픽 메시지로 별도 변환 필요.
