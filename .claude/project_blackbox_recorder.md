---
name: project_blackbox_recorder
description: "pyqt_hmi blackbox_recorder 노드 - Take_Over_Request 트리거 시 [이전10s,이후2s] 자동 저장"
metadata: 
  node_type: memory
  type: project
  originSessionId: c33bd427-3676-488b-95e5-e46292e27002
---

`src/visualization/pyqt_hmi/scripts/blackbox_recorder.py` (standalone rospy 노드, hmi.launch 에 등록).

**동작**: `/localization/to_control_team` 의 `Take_Over_Request` 0→1 상승 + 자율주행(`/sensors/ioniq5_ad_can.autonomous_mode==1` 또는 직전 auto_grace 1.0s 내 ON) → 트리거. 트리거 T 기준 `[T-pre, T+post]`(기본 10s/2s) 저장. TOR=1과 동시에 자율 해제돼도 POST 2s 저장(grace 로 동시-해제 케이스 통과). 저장 토픽 = REC OFF 모드와 동일하게 LiDAR/인지 13토픽(EXCLUDE_TOPICS) 제외한 기본 토픽.

**구조 (중요)**: Python 에서 전 토픽을 AnyMsg 로 버퍼링하면 실차 부하(v_can 600Hz, ublox/esfmeas·esfraw ~700Hz 등 ~2500+ msgs/s)에 GIL/큐가 막혀 트리거 게이트가 깨진다(초기 v1 이 그래서 실패, 하네스로 확인). 그래서 **C++ `rosbag record` 가 롤링 버퍼**(`--split --max-splits`, 항상 가동)를 담당하고, **경량 모니터(2개 저rate 토픽만 구독)** 가 트리거 판정. 트리거 시 recorder 정지(chunk flush)→`[T-pre,T+post]` offline 트림→재개(~0.5s 공백). recorder 는 모드와 무관하게 항상 가동(모드 기반 start/stop 은 TOR-동시-해제 레이스로 POST 가 잘림 → 검증으로 확인).

**검증 하네스**: `~/diag_replay_sample/make_blackbox_test_bag.py` (합성 bag: auto=1→0@12s, TOR↑@12s, 제외토픽 포함). 단일 Bash 호출로 노드+`rosbag play` 동시 실행해야 함(bg 프로세스가 호출 종료 시 죽음). 합성 검증 PASS: span 12.00s, /percept_topic 제외, POST 2s(TOR=1 21개)+자율해제후 캡처(auto=0 21개) 모두 확인. **실 bag 종단 검증은 WSL 샌드박스(1GB bag+bg 프로세스 수명+stdout 버퍼링)에서 미완 → 실차/실 PC 에서 hmi.launch 로 최종 확인 권장.** 관련: [[reference_diag_replay_harness]] [[project_v2x_spat_takeover_decoupled]]
