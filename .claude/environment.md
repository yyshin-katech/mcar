---
name: Environment
description: yuyeong PC는 WSL2 환경, 외부 UDP 수신 시 mirrored 네트워킹 필요
type: reference
originSessionId: 02987e96-3bce-4338-b524-4cc3596fca14
---
- 개발 PC(yuyeong): WSL2 Ubuntu on Windows 11 (빌드 26200)
- WSL2 IP: 172.19.112.102 (NAT, 외부에서 직접 접근 불가)
- 외부 UDP 수신 방법: `.wslconfig`에 `networkingMode=mirrored` 설정 후 WSL 재시작
- WSL **outbound** UDP는 NAT SNAT 통해 호스트 default-route 인터페이스로 정상 송신됨 — 2026-05-28 OBU(192.168.1.5:6666) BSM 송신 검증 (호스트 Wireshark 확인은 사용자 측 필요)
- Python 3.8.10. 2026-05-28 부로 `python3-pip` (apt) + `asn1tools 0.167.0` / `bitstruct` / `pyparsing` (`~/.local`) + `ros-noetic-rosbridge-server` (apt) 설치됨. dpkt/scapy/pyproj 여전히 없음
- 좌표 변환은 `cs2cs` CLI (proj-bin 패키지, 설치 됨) subprocess 호출로 대체
- catkin workspace: `/home/yuyeong/mcar`
- 실차 환경(ads PC)과 별도 — ads PC는 `/home/ads/mcar_v13`
- git remote: https://github.com/yyshin-katech/mcar.git
- git user: yuyeong-shin
- sudo password: `1`
