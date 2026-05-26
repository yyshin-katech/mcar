---
name: mqtt-vpn-setup 하네스
description: MQTT V2N 트래픽만 kanavi VPN split-tunnel 로 보내는 구성·검증 파이프라인 (analyst → configurator → verifier). 실 환경은 SecuwaySSL (Secuwiz) 였으나 하네스 사양은 OpenVPN 기준으로 작성됨
type: project
originSessionId: 8099a2fb-c590-474a-b3ff-3eb9f9dff9be
---
`prod 브로커 192.168.255.173:10044` 트래픽만 kanavi VPN 게이트웨이 (서버측이 클라이언트에 `tun0=172.18.113.51` 동적 할당) 로 보내고, 나머지 트래픽 (인터넷, ublox NTRIP/RTK, ROS local, OBU LAN) 은 default route 유지하는 split-tunnel 을 외과적으로 구성한다. siheung_v2x 의 `mqtt_spat_rx_node` / `mqtt_bsm_tx_node` 가 `mqtt_server:=prod` 인자로 그대로 동작하게 한다.

## 에이전트 구성 (3 단계 파이프라인)

| 단계 | 에이전트 | 입력 | 산출물 |
|------|---------|------|--------|
| 분석 | `vpn-net-analyst` | 사용자 .ovpn 위치, launch/siheung.launch | `_workspace_vpn/01_analyst_spec.md` |
| 구성 | `vpn-net-configurator` | 사양서 + 자격증명 (prompt 평문) + sudo PW | `_workspace_vpn/02_configurator_report.md` + 시스템 변경 |
| 검증 | `vpn-net-verifier` | configurator 보고서 + sudo PW | `_workspace_vpn/03_verifier_report.md` + PASS/PARTIAL/FAIL |

각 에이전트는 `.claude/agents/{name}.md` 정의, `model: "opus"`, `subagent_type: "general-purpose"`.

## split-tunnel 핵심 (verifier 가 분리 검증)

1. `ip route get 192.168.255.173` → dev = `tun0` (또는 `tun-kanavi`) — 브로커는 VPN 으로
2. `ip route get 8.8.8.8` → dev ≠ `tun*` (기타 트래픽은 기존 default 유지)

두 조건이 *동시에* 참이어야 success. (8.8.8.8 이 터널 dev 로 가면 split-tunnel 실패 — OpenVPN 의 경우 `route-nopull` / `pull-filter ignore "redirect-gateway"` 누락 의심.)

## 실제 검증된 환경 (2026-05-26)

실 환경 VPN 은 **SecuwaySSL (Secuwiz, LEA-128-CBC 한국 국정원 인증)** 으로 판명. OpenVPN 이 아님. `.ovpn` 발급 없이 ID/PW + 외부 공인 IP 만으로 동작.

- VPN 클라이언트: `/home/katech/sslvpn/SecuwaySSLU_client` (sudo 필요)
- 설정 파일: `/home/katech/sslvpn/conf/client.info` — `vpn_ip: 27.101.133.111`, `vpn_port: 443`, `crypto: yes`, `log_size: 50`
- 다른 PC 셋업용 산출물 (git 추적):
  - 클라이언트 tgz: `tools/sslvpn/SSU21-2.1.0.2-20230331.tgz` (3.6 MB, 2023-05-22 빌드)
  - 설치/접속 가이드: `tools/sslvpn/README.md` (다른 PC 에서 PDF 없이도 동일 절차 재현 가능)
- 원본 가이드: `~/리눅스 클라이언트 설치 및 사용 가이드 (1).pdf` (운영자 제공, git 비포함)
- 시작: `printf '<sudo_pw>\n<id>\n<pw>\n\n' | sudo -S ./SecuwaySSLU_client` (OTP 는 운영 환경에서 disabled, 빈 엔터로 통과)
- tun0 IP 는 서버가 동적 할당 — 본 환경에서 `172.18.113.51/24` 로 부여됨 (=사용자가 "OBU IP" 라 부르던 것)
- Split-tunnel: 서버 측 push 라우트로 자동 구성. `192.168.255.172~179/32`, `10.30.0.0/16`, `172.16.0.0/12`, `172.18.113.0/24` 만 tun0 으로, 그 외 default 유지. OpenVPN 의 `route-nopull` 류 옵션 불필요
- 종료: `sudo kill <SecuwaySSLU_client PID>` → tun0 자동 제거
- 검증 100% 통과 — broker = tun0, 8.8.8.8 = wlo1, TCP 10044 OPEN, SPaT 5 Hz 수신

## 검증된 V2N 송수신 (2026-05-26)

prod 브로커 (`192.168.255.173:10044`, user `xcms-mtqq`, pw `xcms***!`) 의 `V2N/1321103202/trf_drct/spat` 토픽이 시화 QuadKey 1321103202 영역의 SPaT 를 5 Hz 로 발행 중.

- 검증된 intersection ID: **517** (시화 교차로)
- 검증된 signalGroupID: 60, 80 (양쪽 모두 동시 GO 진행 중)
- 페이로드 길이: 110~280 byte (V2N container 16 byte + J2735 SPaT UPER 94+ byte)
- V2N container head (검증): `04 00 ff 11 <seq:2> <reserved:2> 00 01 40 85 <len:4>` — 표 4-11 정합
- mqtt_spat_rx_node 동작: prod 인자로 broker_host/port/user/pass/topic 모두 정상, decode 성공 → `/siheung_v2x/mqtt_spat` 발행

**중요 — publishSpat 필터 (`j2735_decode.cpp:144-204`):**
- `cur_intersection_id == 0` (= `/localization/to_control_team.look_at_IntersectionID` 미수신) 이면 빈 메시지 발행하고 return
- 디코더가 매칭 발행하려면 ego 의 link 정보 (`look_at_IntersectionID`, `look_at_signalGroupID`, `MANUAVER`) 가 `/localization/to_control_team` 으로 흘러야 함
- 시연용 dummy publish: `rostopic pub -r 5 /localization/to_control_team mmc_msgs/to_control_team_from_local_msg "{look_at_IntersectionID: 517, look_at_signalGroupID: 0, MANUAVER: 0}"`

## 시스템 변경

- `/etc/openvpn/client/kanavi-mqtt.conf` (644, root:root) — `dev tun-kanavi`, `route-nopull`, `pull-filter ignore "redirect-gateway"`, `pull-filter ignore "dhcp-option DNS"`, `route 192.168.255.173 255.255.255.255`, `auth-user-pass /etc/openvpn/auth-kanavi.txt`
- `/etc/openvpn/auth-kanavi.txt` (600, root:root) — 2줄 (ID, PW)
- systemd unit `openvpn-client@kanavi-mqtt.service` (Ubuntu 기본 generator 자동)

## 자격증명 보안

- ID `kana***1`, PW `kana****#$` (마스킹 표기) — 실제 값은 사용자가 chat 에서 직접 제공, 메모리/git 에는 평문 금지
- 메모리·사양서·보고서·git·코드 어디에도 평문 금지
- 시스템 경로 (`/etc/openvpn/auth-kanavi.txt`) 에만 실제 값 (chmod 600)
- 오케스트레이터가 configurator Agent prompt 본문에만 1회 전달

**Why:** 2026-05-26 kanavi VPN 통해서만 prod MQTT 브로커 접속 가능. 인터넷·NTRIP 등 다른 트래픽은 default route 유지해야 하므로 (OpenVPN 의 경우 redirect-gateway 차단, SecuwaySSL 은 서버 push 라우트로 자동 처리). 자격증명은 사용자가 chat 에 직접 입력 — git 평문 금지가 강한 제약.

**How to apply:**
- 사용자가 "vpn 설정", "mqtt vpn", "split-tunnel" 등을 언급하면 `mqtt-vpn-setup` 스킬 호출
- VPN 종류 먼저 확인: `.ovpn` (OpenVPN) vs PDF 가이드/tgz (SecuwaySSL). 본 환경은 후자
- SecuwaySSL 환경에서는 `_workspace_vpn/01_analyst_spec.md` 의 OpenVPN BLOCKED 결과를 무시하고 PDF 가이드 따라 `client.info` 만 수정
- sudo 비밀번호 없으면 configurator 가 명령만 보고서에 적고 사용자 직접 실행 요청
- 변경 후 verifier 의 핵심 2조건 (브로커=tun*, 8.8.8.8=default) 확인 필수
- 마스킹 (`kana***1` / `kana****#$`) 으로만 인용

## 작업 디렉토리

- `_workspace_vpn/` (프로젝트 루트 하위, .gitignore 에 추가됨)
- 기존 `_workspace/` (senario-gps-pub), `_adapt_workspace/` (web-hmi-adapt) 와 분리
