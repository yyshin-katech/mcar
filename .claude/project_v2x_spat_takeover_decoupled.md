---
name: project_v2x_spat_takeover_decoupled
description: V2X SPaT 끊김→TOR 경로. V2X_StatCode 경로는 단절(주석)이나 AliveCount(=SPaT 카운터) 경로로는 SPaT 갭이 TOR 유발. 버스트성 SPaT가 staleness 임계 넘겨 false takeover
metadata: 
  node_type: memory
  type: project
  originSessionId: c33bd427-3676-488b-95e5-e46292e27002
---

V2X SPaT 끊김으로는 `Take_Over_Request`(TOR)가 절대 안 뜬다. 체인이 중간에 끊겨 있음:

**TOR 발화 조건** (`to_control_team_demo.py:503`):
`Take_Over_Request=1` ⟸ `takeoverreq==1 OR Road_State==2 OR On_ODD==1 OR LINK_ID==0`.
`takeoverreq`(diag_cb, L205)는 `/diagnostic/system`(katech_diagnostic_msg)의 9개 status 중 하나라도 ≠0이면 1.

**V2X가 TOR로 가는 유일 경로 = `v2x_status`**. 그런데 stat_display는 `v2x_status`를
**V2X 노드 staleness(AliveCount, V2X_AliveCnt_Check → 0 또는 2)만** 반영하고,
**SPaT 끊김(`V2X_StatCode==1`, "SPaT 30틱 끊김")→`v2x_status` 연결은 주석 처리됨**:
`stat_display.cpp:637-639` `else if(v2x_msg.V2X_StatCode==1){ // v2x_status = 1; ...}` — 글자색만 주황, downstream 전파 0.

따라서 SPaT가 끊겨도 `/diagnostic/system` v2x_status는 안 변하고 TOR도 안 뜬다.
V2X로 TOR을 띄우려면 V2X **노드 자체가 stale**(AliveCount 6틱 정지 → v2x_status=2)해야만 함.

**검증 근거**: bag 2026-06-09-10-46-17...0.bag (25.6s). TOR=0 전구간(0.00s init transient 1 제외).
`/diagnostic/system` v2x=0 전구간, `/diagnostic/v2x` V2X_StatCode=0 전구간(SPaT 안 끊김).
24.1s "V2X 센서 고장" 팝업+v2x_warning.mp3 는 v2x_status=2(노드 ~1s 정지) transient라 /diagnostic/system(~1Hz)엔 미반영.

**고치려면**: `stat_display.cpp:639` `// v2x_status = 1;` 주석 해제(노드 stale=2 보다 낮은 심각도 유지).
그래야 SPaT 끊김 → v2x_status → /diagnostic/system → takeoverreq → TOR 체인 동작.
관련: [[project_v2x_spat_topic]] [[project_control_fault_not_displayed]]

**2026-06-09 정정/보강 (중요)**: 위 "SPaT 끊김은 TOR 전파 안 됨"은 *V2X_StatCode 경로* 한정. 실제로는
**AliveCount 경로로 SPaT 끊김이 TOR을 유발한다**. `V2X_AliveCount`는 v2x_diagnostic timer_callback에서
**SPaT(`/katri_v2x_node/katri_spat`) 한 건당 +1** 되는 *SPaT 수신 카운터*(v2x_diagnostic_pub.cpp L46-51).
SPaT가 멈추면 AliveCount 동결 → stat_display `V2X_AliveCnt_Check`가 unchanged>5틱(≈0.5s)이면 `v2x_status=2`
→ /diagnostic/system → takeoverreq → **TOR**. (V2X_StatCode는 ping 실패시에만 1, SPaT와 무관.)

오늘 bag(10:47:18/11:12:33/11:13:11)에서 TOR이 ~1s씩 뜬 원인 = **SPaT가 버스트+매 ~1초 주기로 0.6~0.8s 갭**
으로 들어와 staleness 임계(0.5s) 경계를 간헐적으로 넘김 = false takeover(실제 두절 아님). V2X_StatCode=0 유지.
해결: stat_display staleness 임계 상향(unchanged>5 → >10~15) 또는 SPaT 송신 버스트 평탄화.

**SPaT 소스 구조 (katri_obu_interface = `/katri_v2x_node`, katri_v2x.cpp)**: ROS 밖 UDP 프로그램이
*필요한 교차로/시그널그룹만 필터*해서 UDP(LOCAL_PORT 50000)로 보냄. 노드는 `recvfrom` **블로킹 대기**
→ UDP 패킷 1개당 `intersection_array_msg` 1번 `pub.publish` (내부 필터 없음, 5개 엔트리 고정 pack).
주의: SO_RCVTIMEO를 `int optVal=10000`으로 설정하나 struct timeval 아니라 타입/크기 불일치 → 무시됨 → recv 무한 블로킹.
⇒ **외부 프로그램이 보낼 게 없어 패킷을 안 보내면 발행 정지 → AliveCount 동결 → v2x_status=2 → TOR**.
진단이 "관련 교차로 없음(정상)" vs "V2X 실제 두절(고장)"을 구분 못 함(둘 다 '메시지 부재'로 동일).
트리거는 *내용이 빈 것*이 아니라 *패킷 자체가 안 온 것*. 외부가 빈 패킷이라도 계속 보내면 AliveCount 증가→고장 안 잡힘.

**2026-06-09 구현 (게이팅)**: stat_display.cpp V2X_Text_Gen 을 재작성. `V2X_AliveCnt_Check`(무조건 staleness=고장) 제거,
대신 `last_spat_time_`(traffic_light_callback에서 SPaT 수신시각 기록) 기반 `spat_stale=(now-last_spat)>0.5s` +
`tl_needed=(local_msg.look_at_signalGroupID!=0)`. **`v2x_status=2`(→TOR)는 tl_needed && spat_stale 일 때만**.
색상: spat_stale(또는 ping V2X_StatCode==1)이면 주황, 아니면 초록. `// v2x_status = 1;` 주석 유지(미사용).
즉 신호등 불필요 구간 SPaT 끊김은 고장 아님(주황 정보만), 신호등 필요 구간 SPaT 0.5s 미수신만 고장→TOR.
검증(원본 bag 10Hz 오프라인 sim, 코드와 동일 로직): look_at==0 구간 고장 0틱(false TOR 완전 제거).
10:47:18 false TOR 79%↓, 11:12:33 50%↓, 11:13:11 0%↓(전 구간 신호등 필요라 모두 정당).
**잔존 이슈**: SPaT가 버스트+~0.7s 갭이라 신호등 필요(교차로 접근) 구간엔 0.5s 임계를 정상 cadence가 넘겨 TOR이 간헐 발생(11:13:11 12%). off-intersection false TOR만 제거됨. 추가로 줄이려면 SPaT 송신 평탄화 또는 임계>0.7s.
