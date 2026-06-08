---
name: WSL rostopic hz 불안정 → echo count 우회
description: WSL2 환경에서 `rostopic hz` 가 토픽이 실제로 흐르는데도 종종 메시지 1개도 못 받고 timeout/Terminated 만 출력함. 노드 상태 판단 시 echo 카운트로 cross-check.
type: feedback
---

**Rule:** WSL2 환경에서 토픽 publish 율을 확인할 때 `rostopic hz <topic>` 결과가 0 / Terminated 면 단정하지 말고 `timeout N rostopic echo -n M <topic> | grep -c '^---'` 또는 `tcpdump` (UDP 송신인 경우) 로 cross-check.

**Why:** 2026-05-28 세션에서 `bsm_tx_node` 가 정상 publish 중인데 `rostopic hz /siheung_v2x/bsm_tx` 가 계속 Terminated 만 출력함. 같은 시점에 `rostopic echo -n 5` 는 5개를 즉시 받았고 `tcpdump -i eth0` 는 정확히 100ms 간격 UDP 송신을 보여줬다. `rostopic hz` 가 WSL TCP transport timing 에서 첫 callback 안에 충분한 sample 을 모으지 못하는 듯. 동일 명령이 ads 실차 환경에서는 정상.

**How to apply:**
- "토픽이 안 들어옴" 같은 진단 시 `rostopic hz` 만 보고 결론 내지 말기.
- 우선 `timeout 4 rostopic echo -n 5 <topic> | grep -c '^---'` → 1 이상 나오면 정상 흐름. 0 이면 진짜 멈춤.
- UDP/TCP outbound 노드는 `sudo tcpdump -i eth0 -c N '<filter>'` 가 더 신뢰성 있음 (ROS transport stack 우회).
- `source devel/setup.bash` 가 sub-shell 마다 다시 필요 — bash 매 호출이 새 shell 이라 매번 source. 그래도 hz 는 여전히 flaky 한 게 관찰됨.
