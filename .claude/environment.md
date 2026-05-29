---
name: Environment
description: yuyeong PC(WSL)는 테스트 환경일 뿐, 실제 배포는 네이티브 Ubuntu. WSL 우회책을 실환경에 적용 금지
type: reference
originSessionId: 02987e96-3bce-4338-b524-4cc3596fca14
---
- **중요: WSL 환경(yuyeong PC)은 테스트용이고, 실제 배포 환경은 WSL 아닌 네이티브 로컬 Ubuntu(실차/현장 PC).** 아래 WSL 전용 우회책(mirrored 네트워킹, Windows 방화벽 인바운드 규칙, tcpdump/rostopic 캡처 불가, bind_ip=192.168.1.3 기본값)은 **실제 환경에 적용/고려하지 말 것**. 실환경은 일반 Linux 네트워킹으로 INADDR_ANY(0.0.0.0) UDP 수신·tcpdump·rostopic 모두 정상 동작
- 개발 PC(yuyeong): WSL2 Ubuntu on Windows 11 (빌드 26200) — **테스트 전용**
- WSL2 IP: 172.19.112.102 (NAT, 외부에서 직접 접근 불가)
- 외부 UDP 수신 방법: `.wslconfig`(`/mnt/c/Users/you0d/.wslconfig`)에 `networkingMode=mirrored` 설정 후 WSL 재시작 (PowerShell `wsl --shutdown`). 적용되면 eth0 가 호스트 LAN IP(=192.168.1.3)로 바뀜
- **mirrored 모드 외부 UDP 인바운드는 mirrored 설정만으론 부족** — Windows Defender(Hyper-V) 방화벽이 unsolicited 인바운드를 막아 0 datagram. 관리자 PowerShell 에서 `New-NetFirewallRule -DisplayName "WSL OBU UDP 9999" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 9999` 필요 (2026-05-29 OBU SPaT UDP 9999 수신 검증: 규칙 추가 즉시 `/siheung_spat` 389 msgs/5s). 진단 시 WSL `tcpdump`/`rostopic`은 mirrored 에서 캡처 0 으로 신뢰 불가 → 순수 Python UDP 리시버 + `/proc/net/snmp` InDatagrams 델타로 cross-check
- `siheung_v2x_node`(`src/v2x/siheung_v2x/src/j2735_decode.cpp`) 는 2026-05-29 부로 UDP 바인드를 INADDR_ANY → `~bind_ip`/`~bind_port` ROS param 으로 변경. **기본값 `0.0.0.0:9999`(실제 네이티브 Ubuntu 정상)**. `siheung.launch` 에 `obu_bind_ip`(기본 0.0.0.0)/`obu_bind_port` arg 추가. **실 환경은 `roslaunch launch/siheung.launch` 그대로**, **WSL 테스트만 `obu_bind_ip:=192.168.1.3` override**. (WSL 에서 INADDR_ANY 가 안 됐던 진짜 원인은 방화벽이었음 — 위 항목 참조)
- **이 세션(harness) 에서 ROS 백그라운드 실행 함정**: `roslaunch ... & disown` 은 Bash 툴 호출이 끝날 때 SIGINT 받아 "killing on exit" 으로 죽는 경우가 있음. `setsid roslaunch ... < /dev/null > log 2>&1 &` 로 새 세션 분리하거나 Bash `run_in_background:true` 로 띄워야 안정적으로 잔존
- WSL **outbound** UDP는 NAT SNAT 통해 호스트 default-route 인터페이스로 정상 송신됨 — 2026-05-28 OBU(192.168.1.5:6666) BSM 송신 검증 (호스트 Wireshark 확인은 사용자 측 필요)
- Python 3.8.10. 2026-05-28 부로 `python3-pip` (apt) + `asn1tools 0.167.0` / `bitstruct` / `pyparsing` (`~/.local`) + `ros-noetic-rosbridge-server` (apt) 설치됨. dpkt/scapy/pyproj 여전히 없음
- 좌표 변환은 `cs2cs` CLI (proj-bin 패키지, 설치 됨) subprocess 호출로 대체
- catkin workspace: `/home/yuyeong/mcar`
- 실차 환경(ads PC)과 별도 — ads PC는 `/home/ads/mcar_v13`
- git remote: https://github.com/yyshin-katech/mcar.git
- git user: yuyeong-shin
- sudo password: `1`
