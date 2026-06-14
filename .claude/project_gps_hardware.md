---
name: GPS Hardware - USB Serial, No Ethernet
description: GPS는 USB serial 연결. 이더넷/인터넷 의존 진단(NTRIP/ping)은 무의미하니 건드릴 때 주의
type: project
originSessionId: 7bc83e9b-1c21-49ec-9db9-d586acbc5f2f
---
이 차량의 GPS(u-blox ZED-F9K)는 **USB serial(`/dev/ttyACM0`) 연결**이며, 이더넷은 사용하지 않는다. RTK 보정도 NTRIP 같은 인터넷 기반은 안 씀.

**Why:** 사용자가 "GPS는 이더넷 연결 안되어 있고 이더넷 안쓰고있어서 쓸모없어"라고 명시. cpt7_diagnostic의 `pingLoop` thread (8.8.8.8으로 인터넷 도달성 검사)는 이 환경에서 항상 실패 → `Network_Status=1` → stat_display가 GPS 항상 빨강으로 표시 → 통합경보 카운트 1 점유 → 다른 센서 1개만 죽어도 즉시 "시스템 고장 2개" 점프하는 false alarm 유발.

**How to apply:**
- cpt7_diagnostic의 `ping_thread` 시작은 `cpt7_diagnostic_pub.cpp:25`에 의도적으로 주석 처리됨. 다시 활성화하지 말 것 (재활성화하려면 ping target을 NTRIP 서버 IP로 바꿔야 의미 있음).
- GPS 진단 동작 검증할 때 네트워크 상태가 아닌 `/ublox/navpvt`의 `fixType`, `flags` (carrSoln), `hAcc/vAcc`만 봄.
- 이더넷 의존 코드 추가 제안 금지 — RTK가 필요해도 이 차량 셋업에서는 안 됨.
