---
name: Avoid root-requiring sockets — prefer system ping + worker thread
description: SOCK_RAW 같이 root 권한 필요한 코드는 system() 명령 호출 + 별도 thread로 우회하는 걸 선호
type: feedback
originSessionId: 7bc83e9b-1c21-49ec-9db9-d586acbc5f2f
---
ICMP 같은 root 권한 의존 소켓(`SOCK_RAW`) 대신 `system("ping -c 1 -W 1 IP")` 호출 + `std::thread` worker 패턴을 사용한다.

**Why:** 사용자가 lidar/v2x 진단을 "root권한 없이 ping 테스트 되도록 system 핑으로 수정 별도 스레드"라고 직접 지시. `/bin/ping`은 setuid/cap_net_raw가 이미 설정돼 있어 일반 사용자로 호출 가능. SOCK_RAW는 노드 자체에 권한 부여 필요(setcap 또는 sudo 실행)해서 launch가 깨지기 쉬움.

**How to apply:**
- 새 진단/헬스체크 코드에 ICMP가 필요하면 `system("ping -c 1 -W 1 ...")`로 작성. SOCK_RAW 쓰지 말 것.
- 결과는 `std::atomic<bool>`로 worker thread → main timer로 전달. main timer/콜백에서 직접 ping 호출 금지 (1초 timeout이 ROS spin을 블로킹함).
- 패턴 참고: `cpt7_diagnostic_pub.cpp::pingLoop`, `lidar_diagnostic_pub.cpp::pingLoop`, `v2x_diagnostic_pub.cpp::pingLoop` 모두 동일 형태 (1초 sleep + 100ms 단위 stop flag 체크).
- 명령어 인자는 신뢰 가능한 IP 문자열만 넣기 — 사용자 입력을 그대로 system()에 넘기면 command injection.
