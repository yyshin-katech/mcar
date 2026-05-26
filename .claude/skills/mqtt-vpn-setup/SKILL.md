---
name: mqtt-vpn-setup
description: MQTT V2N 인터페이스 트래픽만 시흥시 VPN 터널 (게이트웨이 172.18.113.51) 로 보내고, 나머지 인터넷·NTRIP·로컬 LAN 트래픽은 default route 를 유지하는 split-tunnel 을 구성·검증한다. siheung_v2x 의 mqtt_spat_rx_node / mqtt_bsm_tx_node 가 prod 브로커 (192.168.255.173:10044) 에 접속할 수 있도록 분석→구성→검증 파이프라인으로 적용한다. 사용자가 "vpn 설정", "mqtt vpn", "split-tunnel", "시흥시 vpn", "mqtt 인터페이스 vpn", "vpn 다시 설정", "vpn 재구성", "openvpn 분리 터널", "192.168.255.173 vpn", "OpenVPN 클라이언트 설정" 등을 요청하면 반드시 이 스킬을 사용한다. 단순 ROS/launch 질문이나 일반 네트워크 진단은 직접 응답.
---

# mqtt-vpn-setup — MQTT V2N OpenVPN Split-Tunnel (Orchestrator)

prod MQTT 브로커 (`192.168.255.173:10044`) 트래픽만 OpenVPN 터널 (`172.18.113.51`) 로 보내고, 그 외 트래픽 (인터넷, ublox NTRIP/RTK, ROS local, OBU LAN 등) 은 *원래 default route* 를 유지하는 split-tunnel 을 외과적으로 구성한다. siheung_v2x 의 `mqtt_spat_rx_node` / `mqtt_bsm_tx_node` 가 `mqtt_server:=prod` 인자로 그대로 동작하도록 한다.

## 실행 모드

**서브 에이전트 파이프라인**: analyst → configurator → verifier. 시스템 경로 (`/etc/openvpn/`) 변경과 systemd 통합이 섞여 있어 단계별 격리가 안전하고, 자격증명을 어느 에이전트가 가지고 있는지가 명확해야 한다 (analyst 는 위치만, configurator 만 실제 쓰기).

각 에이전트는 `Agent` 도구로 호출하고 `model: "opus"` 명시. 산출물은 `_workspace_vpn/` 파일로 전달.

## 보안 원칙 (모든 Phase 공통)

- 자격증명 (ID `kana***1`, PW `kana****#$` — 마스킹 표기) 은 **오케스트레이터 메모리에만** 보관. 사양서·보고서·메모리 파일·코드·git 어디에도 평문 금지. 실제 값은 사용자가 chat 에서 직접 제공.
- 마스킹 규칙: ID `kana***1`, PW `kana****#$` 로만 표기.
- 시스템 경로 사용: `/etc/openvpn/client/siheung-mqtt.conf` (644), `/etc/openvpn/auth-siheung.txt` (600, root:root).
- `.gitignore` 갱신은 *프로젝트 경로* 에 임시 .ovpn 이나 auth 파일이 들어올 때만. 시스템 경로는 git 무관.

## Phase 0: 컨텍스트 확인

작업 시작 전 `_workspace_vpn/` 상태로 실행 모드 결정.

| 상태 | 모드 |
|------|------|
| `_workspace_vpn/` 없음 | **초기 실행** — 1~3 Phase 전체 |
| `_workspace_vpn/` 존재 + 사용자가 부분 수정 요청 (예: "브로커 대역을 /24로", "포트만 변경", "재검증만") | **부분 재실행** — 해당 Phase 만 |
| `_workspace_vpn/` 존재 + 사용자가 새 입력 (다른 .ovpn, 다른 VPN 서버) | **새 실행** — 기존을 `_workspace_vpn_prev/` 로 mv 후 처음부터 |

또한 사용자에게 다음 두 가지를 확인:
- `.ovpn` 파일 존재 여부 / 위치 (없으면 BLOCKED, configurator 진행 불가)
- sudo 비밀번호 사용 가능 여부 (없으면 configurator 가 명령만 출력하고 사용자에게 직접 실행 요청)

## Phase 1: 분석 (vpn-net-analyst)

목표: 현재 네트워크 상태 + .ovpn 가용 여부 + 라우팅 충돌 가능성 진단 → split-tunnel 사양서 1개.

핵심 입력:
- `launch/siheung.launch` (mqtt_host_prod / mqtt_port_prod 추출)
- 사용자 .ovpn 파일 (위치는 사용자 확인)
- 시스템 상태 (`ip route`, `ip addr`, `openvpn --version`, `ip link show type tun`)

산출물: `_workspace_vpn/01_analyst_spec.md`

핵심 결정 사항:
- tun 인터페이스명 (기본: `tun-siheung` — 다른 VPN 충돌 회피)
- 브로커 라우팅 범위 (`/32` 단일 IP vs `/24` 서브넷)
- pull-filter 정책 (`redirect-gateway`, `dhcp-option DNS` 무시)
- VERDICT: ✅ / ⚠ (경고 있으나 진행 가능) / BLOCKED (.ovpn missing 등)

## Phase 2: 구성 (vpn-net-configurator)

목표: 사양서대로 OpenVPN 클라이언트 + systemd 적용. 사양 외 변경 금지.

핵심 입력:
- `_workspace_vpn/01_analyst_spec.md` (VERDICT 가 BLOCKED 면 중단)
- 오케스트레이터가 전달한 자격증명 (chat 에서 사용자 제공)
- sudo 비밀번호 (오케스트레이터가 환경 변수로 제공 시)

적용 대상:
- `/etc/openvpn/client/siheung-mqtt.conf` (사양서의 .conf 본문 그대로)
- `/etc/openvpn/auth-siheung.txt` (자격증명 2줄, chmod 600)
- systemd unit (`openvpn-client@siheung-mqtt.service` generator 자동)
- launch/.gitignore (사양서가 요구 시만)

산출물: `_workspace_vpn/02_configurator_report.md` + 시스템 변경.

## Phase 3: 검증 (vpn-net-verifier)

목표: 실제 동작 확인. split-tunnel 의 두 조건이 동시에 참인지 분리 검증.

핵심 입력:
- `_workspace_vpn/02_configurator_report.md` (verdict 확인)
- 시스템 상태 (`ip route`, `tun-siheung`)

검증 핵심:
1. `ip route get 192.168.255.173` → output 에 `dev tun-siheung` 포함
2. `ip route get 8.8.8.8` → output 의 dev 가 **tun-siheung 아님** (default 유지)

산출물: `_workspace_vpn/03_verifier_report.md` + verdict:
- **PASS**: 1~4 단계 + 6번 TCP 이상 통과
- **PARTIAL PASS**: 1~4 통과, 5~7 은 환경 한계 (사용자 사내망 후속 검증)
- **FAIL**: 1~4 중 실패 → configurator 재호출

## 데이터 흐름

```
사용자 자격증명 ─┐
.ovpn 파일 ──── analyst ──spec──> configurator ──시스템변경──> verifier ──verdict──> 사용자
                                       │                            │
                                       └── (FAIL 시 재작업) ─────────┘
```

전달 방식: 파일 기반 (`_workspace_vpn/`). 자격증명은 *오케스트레이터 prompt 만* 통해 configurator 에 전달, 파일에 평문 저장 금지.

## 자격증명 흐름

```
사용자 (chat) → 오케스트레이터 메모리
                 │
                 ├── analyst       : 위치만 기록 (사양서에 "사용자 제공값" 표기)
                 ├── configurator  : Agent prompt 로 평문 전달 (1회), /etc/openvpn/auth-siheung.txt 에만 기록
                 └── verifier      : 미전달 (verifier 는 자격증명 불필요, TCP 도달성만 확인)
```

오케스트레이터가 configurator Agent 호출 시 prompt 본문에 자격증명을 포함하되, 보고서/메모리/git 어디에도 평문 저장 금지. 마스킹 (`kana***1` / `kana****#$`) 만 노출.

## 에러 핸들링

| 단계 | 실패 유형 | 대응 |
|------|----------|------|
| Phase 1 | .ovpn 미제공 | 사양서 VERDICT=BLOCKED. 사용자에게 .ovpn 위치 요청. configurator 진행 불가. |
| Phase 1 | `172.18.113.51` ping 불가 | 경고로만 (사내망 진입 전이라 정상). configurator 는 진행, verifier 가 실제 검증. |
| Phase 2 | sudo 비밀번호 없음 | configurator 가 *실행 명령 그대로* 보고서에 적고 사용자에게 직접 실행 요청. verifier 는 사용자 실행 후 재호출. |
| Phase 2 | 사양서 BLOCKED | configurator 중단, 사용자에게 보고. |
| Phase 2 | 기존 siheung-mqtt.conf 충돌 | 백업 후 덮어쓰기. 사용자에게 백업 경로 보고. |
| Phase 3 | systemctl start 실패 | journalctl 캡쳐 → configurator 재호출 (사양 점검). |
| Phase 3 | `ip route get 192.168.255.173` 가 tun-siheung 가 아님 | FAIL 보고. configurator 재호출 (route 줄 점검). |
| Phase 3 | `ip route get 8.8.8.8` 가 tun-siheung | **심각** — split-tunnel 실패, default route 가 VPN 으로 전환됨. configurator 즉시 재호출 (`route-nopull` + `pull-filter` 점검). |
| Phase 3 | mosquitto_sub timeout | TCP 도달성으로 디그레이드. PARTIAL PASS. |
| Phase 3 | 2회 연속 FAIL | 사용자에게 보고 후 중단. |

## 에이전트 호출 패턴

오케스트레이터는 다음 순서로 Agent 도구를 호출한다. 모두 `model: "opus"` 명시, `subagent_type: "general-purpose"`.

```
Phase 1:
  Agent(name="vpn-net-analyst", model="opus",
        prompt="<.claude/agents/vpn-net-analyst.md 본문 + 사용자 요청 + Phase 0 결과 + .ovpn 위치 (사용자 제공 시)>")

Phase 2 (Phase 1 완료 + VERDICT 확인 후):
  Agent(name="vpn-net-configurator", model="opus",
        prompt="<.claude/agents/vpn-net-configurator.md 본문 + 사양서 경로 + 자격증명 평문 (이 prompt 만) + sudo 비밀번호 (제공 시)>")

Phase 3 (Phase 2 완료 후):
  Agent(name="vpn-net-verifier", model="opus",
        prompt="<.claude/agents/vpn-net-verifier.md 본문 + configurator 보고서 경로 + sudo 비밀번호>")
```

에이전트 정의는 `.claude/agents/<name>.md` 에서 읽어 전달. 보고서는 `_workspace_vpn/` 에 저장하므로 다음 에이전트는 파일 경로만 받으면 충분.

## 테스트 시나리오

### 정상 흐름
1. 사용자: "vpn 설정해줘, 시흥시 VPN 으로 mqtt 만 분리 터널"
2. 오케스트레이터: .ovpn 위치 확인 → `~/Downloads/siheung.ovpn`
3. analyst: 환경 진단 + 사양서 작성 → `_workspace_vpn/01_analyst_spec.md` (VERDICT=✅)
4. configurator: `/etc/openvpn/client/siheung-mqtt.conf` + `/etc/openvpn/auth-siheung.txt` 적용 → `_workspace_vpn/02_configurator_report.md`
5. verifier:
   - `systemctl start openvpn-client@siheung-mqtt` → active
   - `ip route get 192.168.255.173` → tun-siheung ✅
   - `ip route get 8.8.8.8` → 기존 default ✅
   - `timeout 3 bash -c '</dev/tcp/192.168.255.173/10044'` → OK
   - → `_workspace_vpn/03_verifier_report.md` (PASS)
6. 사용자에게 최종 보고 (파일 경로 + 실행 명령 + 보고서 위치 + auto-start 권장 여부)

### 에러 흐름 (.ovpn missing)
1. 사용자 요청 동일
2. 오케스트레이터: .ovpn 위치 질문 → 사용자가 "아직 없어"
3. analyst: VERDICT=BLOCKED, 사양서에 "사용자에게 .ovpn 제공 요청" 기재
4. 오케스트레이터: 사용자에게 "OpenVPN 서버 인증서/설정파일 (.ovpn) 이 없으면 split-tunnel 자체가 불가합니다. 발급 절차 안내 또는 파일 제공 부탁드립니다." 보고
5. 사용자가 .ovpn 제공 후 부분 재실행 (Phase 0 의 부분 모드 — analyst 부터)

### 에러 흐름 (split-tunnel 실패)
1. Phase 1~2 정상 완료
2. verifier: `ip route get 8.8.8.8` 가 tun-siheung 으로 감 → **심각 FAIL**
3. 오케스트레이터: configurator 재호출, "route-nopull / pull-filter 적용 여부 재점검" 사유 전달
4. configurator: 사양서 비교 후 누락된 줄 추가
5. verifier 재실행 → PASS

## 작업 디렉토리

루트: 프로젝트 루트 (`/home/katech/mcar_v13`).
- `_workspace_vpn/01_analyst_spec.md`
- `_workspace_vpn/02_configurator_report.md`
- `_workspace_vpn/03_verifier_report.md`
- 시스템 변경: `/etc/openvpn/client/siheung-mqtt.conf`, `/etc/openvpn/auth-siheung.txt`

## 산출물 체크리스트 (오케스트레이터가 사용자에게 보고 시)

- [ ] tun 인터페이스명 + 라우팅 범위 (`/32` 또는 `/24`)
- [ ] 검증 단계별 결과 (PASS / PARTIAL / FAIL)
- [ ] split-tunnel 핵심 조건 2개의 출력 인용 (`ip route get 192.168.255.173` / `ip route get 8.8.8.8`)
- [ ] 실행 명령: `sudo systemctl start openvpn-client@siheung-mqtt`, `sudo systemctl enable ...` (권장 여부 verifier 가 결정)
- [ ] ROS 실행 명령: `roslaunch launch/siheung.launch mqtt_server:=prod`
- [ ] `_workspace_vpn/` 보고서 3개 위치
- [ ] 자격증명은 마스킹 (`kana***1` / `kana****#$`) 으로만 인용

## 후속 작업

- VPN 재구성 (.ovpn 갱신, 포트 변경, 브로커 대역 확장) → 부분 재실행
- 재검증만 (`ip route get` 결과 재확인) → verifier 만
- 자격증명 변경 → configurator 만 재호출 (auth-siheung.txt 만 갱신)
- VPN 제거 → 별도 요청 시 configurator 가 systemd disable + conf 백업 + 자격증명 파일 삭제
