# SecuwaySSL VPN 클라이언트 — kanavi 게이트웨이 접속

prod MQTT V2N 브로커 (`192.168.255.173:10044`) 에 외부 인터넷에서 접속할 때 사용하는 SecuwaySSL (Secuwiz, 한국 국정원 인증 LEA-128-CBC) VPN 클라이언트.

운영자(카네비) 가 발급한 ID/PW 만 있으면 동작하며 `.ovpn` 인증서 / OpenVPN 설치 불필요. 서버가 split-tunnel 라우트를 자동 push 하므로 클라이언트 측 분리 터널 설정 작업도 필요 없다.

원본 가이드 PDF: `~/리눅스 클라이언트 설치 및 사용 가이드 (1).pdf` (운영자 제공, git 비포함)

## 적용 환경

- Ubuntu 20.04 / 22.04 (sudo 가능 유저)
- 외부 인터넷 회선 (방화벽이 TCP 443 outbound 차단하지 않을 것)
- `iproute2`, `net-tools` (대부분 기본 설치됨)
- 다른 OpenVPN/VPN 클라이언트가 default route 를 점유하지 않을 것

## 설치 (다른 PC 에서 처음 셋업할 때)

```bash
# 1. tgz 를 사용자 홈으로 복사 후 해제 (sslvpn 디렉토리 생성됨)
cp /path/to/mcar_v13/tools/sslvpn/SSU21-2.1.0.2-20230331.tgz ~/
cd ~/ && tar xzf SSU21-2.1.0.2-20230331.tgz
ls ~/sslvpn/    # SecuwaySSLU_client, conf/, log/, sbin/, ssu.tgz

# 2. 서버 주소 설정 (운영자가 발급한 값으로 갱신)
cat > ~/sslvpn/conf/client.info <<'EOF'
vpn_ip: 27.101.133.111
vpn_port: 443
crypto: yes
log_size: 50
EOF
```

`client.info` 의 4개 키:

| 키 | 값 | 비고 |
|----|----|----|
| `vpn_ip` | 서버 공인 IP (운영자 제공) | 사설 IP (172.x, 10.x) 는 외부에서 접속 불가 |
| `vpn_port` | `443` | TLS 포트 |
| `crypto` | `yes` | LEA-128-CBC 활성화 |
| `log_size` | `50` | `~/sslvpn/log/` 회전 한계 (MB) |

## 접속

```bash
cd ~/sslvpn
sudo ./SecuwaySSLU_client
# 프롬프트 순서:
#   User ID  : <발급 ID>
#   Password : <발급 PW>
#   OTP      : <빈 엔터>   ← 운영 환경에서 OTP disable 이라 빈 입력으로 통과
```

자격증명 (`ID` / `PW`) 은 카네비 운영자가 발급하며 **git/메모리/문서 어디에도 평문 금지**. 마스킹 표기 (`kana***1` / `kana****#$`) 만 인용 가능.

`SecuwaySSL client conncted!!` 메시지가 나오면 접속 완료. 백그라운드로 데몬화하려면:

```bash
printf '<sudo_pw>\n<id>\n<pw>\n\n' | sudo -S ./SecuwaySSLU_client > /tmp/sslvpn.log 2>&1 &
```

## 접속 검증 (split-tunnel 핵심 2조건)

```bash
ip addr show tun0 | head -3
# inet 172.18.113.51/24   ← 서버가 동적 할당, PC 마다 다를 수 있음

ip route get 192.168.255.173
# 192.168.255.173 via 172.18.113.1 dev tun0 ...   ← 브로커 = tun0 ✅

ip route get 8.8.8.8
# 8.8.8.8 via <기본 gw> dev <기존 NIC> ...        ← 인터넷 = default 유지 ✅

timeout 3 bash -c '</dev/tcp/192.168.255.173/10044' && echo "TCP 10044 OPEN"
```

두 라우팅 조건이 동시에 충족되면 split-tunnel 정상.

## 서버가 push 하는 라우트 (참고)

이 클라이언트는 서버 측에서 다음 대역만 tun0 로 가도록 자동 설정한다. 클라이언트 추가 설정 불필요.

- `192.168.255.172/32` ~ `192.168.255.179/32` (prod MQTT/제어 서버 8대)
- `10.30.0.0/16`
- `172.16.0.0/12`
- `172.18.113.0/24` (VPN 내부 가상 LAN)

그 외 모든 트래픽 (인터넷, NTRIP, ROS local, OBU LAN) 은 기존 default route 유지.

## 종료

```bash
sudo kill "$(pgrep -f SecuwaySSLU_client)"
# tun0 자동 제거 + 라우팅 원복 확인
ip addr show tun0   # → Device "tun0" does not exist.
```

## prod MQTT + ROS 시연

VPN 접속 상태에서 다음 절차로 SPaT 디코딩 확인 가능 (`launch/siheung.launch` 가 prod 인자로 broker_host/port/user/pass 를 채워 줌).

```bash
cd ~/mcar_v13
source /opt/ros/noetic/setup.bash && source devel/setup.bash
roscore &
rosrun siheung_v2x mqtt_spat_rx_node \
  _broker_host:=192.168.255.173 _broker_port:=10044 \
  _username:=<user> _password:='<pw>' \
  _topic:=V2N/1321103202/trf_drct/spat \
  _spat_topic:=/siheung_v2x/mqtt_spat \
  _verbose_first_msg:=true &
# 다른 터미널
rostopic echo -n 5 /siheung_v2x/mqtt_spat
```

`publishSpat` 가 `/localization/to_control_team.look_at_IntersectionID` 매칭 교차로만 발행하므로, 실차 localization 없이 단독 시연 시 dummy publish 필요:

```bash
rostopic pub -r 5 /localization/to_control_team mmc_msgs/to_control_team_from_local_msg \
  "{look_at_IntersectionID: 517, look_at_signalGroupID: 0, MANUAVER: 0}"
```

검증된 IID 517 (시화 교차로) 의 signalGroup 60/80 이 GO(green-prot) phase 로 5 Hz 발행된다.

## 트러블슈팅

| 증상 | 원인 / 조치 |
|------|----|
| `vpn_ip` 가 사설 IP (172.x, 10.x) | 외부에서 라우팅 불가. 운영자에게 공인 IP 확인 |
| `Connection refused` / TCP 443 NG | 회선 방화벽 outbound 차단. 일반 모바일 핫스팟으로 시험 |
| `Authentication failed` | ID/PW 오타 또는 운영자측에서 계정 비활성. OTP 칸은 빈 엔터 |
| `tun0` 만들어졌으나 `ip route get 192.168.255.173` 가 default 로 빠짐 | 다른 VPN/route 가 우선. `ip route` 로 conflict 확인 |
| `8.8.8.8` 도 `tun0` 로 빠짐 | 서버 설정이 redirect-gateway 인 경우. 운영자에 split push 확인 요청 |
| `mosquitto_sub` 가 TCP OPEN 인데 메시지 없음 | 토픽 발행 빈도 낮은 경우 가능. `V2N/+/+/+` 와일드카드 + 10초 대기 |
| client.log 에 `crypto init failed` | `client.info` 의 `crypto: yes` 누락 또는 LEA-128 라이브러리 손상. tgz 재해제 |

자세한 검증 결과와 실제 명령 출력은 `.claude/mqtt_vpn_setup_harness.md` (Claude 메모리, git 추적) 참조.
