---
name: spat-merge-obu-mqtt
description: OBU+MQTT SPaT 를 교차로 단위 OBU 우선으로 병합하는 spat_merge_node (/spat_merged) — MQTT-only 교차로(302)를 HMI/CAN 에 전달
metadata: 
  node_type: memory
  type: project
  originSessionId: 52c4ff5f-49f1-423d-91ad-86c42d4f3a4e
---

# spat_merge_node — OBU + MQTT SPaT 병합 (siheung_dev, 2026-06-22)

**문제:** OBU(`/siheung_spat`)에 없는 교차로가 MQTT(`/siheung_v2x/mqtt_spat`)에는 들어옴 (특히 IID **302**, 링크 417의 타깃). 그런데:
- HMI 3곳(web_hmi `hmi_state.py`, pyqt_hmi `main_window.py`, stat_display)은 `/siheung_spat`(OBU)만 구독 → MQTT-only 교차로 **무조건 미표시**.
- `spat_CAN_writer`는 OBU+MQTT 둘 다 구독했지만 `CALLBACK_MQTT_SPAT`의 OBU 우선 게이트가 **전역(all-or-nothing)** — OBU가 2초 내 *아무 교차로나* 1건 실으면 MQTT 전체 skip. OBU가 302를 안 실어도 OBU가 살아있으면 302 CAN 차단.

**해결 (병합 노드 방식):** 신규 `src/v2x/siheung_v2x/src/spat_merge_node.cpp`.
- OBU + MQTT 구독 → 교차로(IID) 단위 OBU 우선 병합 → 단일 `/spat_merged` 10Hz 재발행.
- movement 단위 캐시 키 `(IntersectionID, SignalGroupID, MovementStateName)` — `v2x_msgs/intersection_msg.Movements`는 **단일** movement_msg(배열 아님)라 같은 IID가 movement마다 data[] 원소로 분리돼 들어옴.
- OBU 콜백: `obu_iid_last_seen[IID]=now` + 캐시 갱신. MQTT 콜백: 해당 IID가 OBU에 fresh(`obu_priority_timeout` 기본 2s)면 skip, 아니면 캐시.
- 발행 타이머: IID별 승자 소스(OBU fresh면 OBU, 아니면 MQTT) 결정 → **소스 혼합 없이** 승자 엔트리만 출력. `entry_ttl`(기본 3s) 지난 엔트리 제거(죽은 신호 사라짐).
- params: `obu_topic`/`mqtt_topic`/`merged_topic`/`obu_priority_timeout`/`entry_ttl`/`publish_rate`.

**소비자 4곳 `/siheung_spat` → `/spat_merged` 재지정:**
- `src/sensing/can/src/spat_CAN_writer.cpp`: sub1 토픽 변경, **sub3(`/siheung_v2x/mqtt_spat`) 제거** (병합은 노드가 처리). `CALLBACK_SPAT`이 병합 스트림을 받아 그대로 CAN write. `CALLBACK_MQTT_SPAT`/`OBU_IS_FRESH`는 미호출 잔존(무해).
- `src/visualization/stat_display/lib/stat_display.cpp:44`
- `src/visualization/pyqt_hmi/scripts/widgets/main_window.py:602`
- `src/visualization/pyqt_hmi/scripts/utils/hmi_state.py:144` (web_hmi `BaseHmiStateController` 공용)
- `v2x_diagnostic`은 OBU 건전성 확인용이라 raw `/siheung_spat` **그대로 둠**.

**CMake/launch:** siheung_v2x `CMakeLists.txt`에 `spat_merge_node` 타겟 추가. `launch/siheung.launch`에 노드 기동 추가(mqtt_spat_rx_node 앞).

**소비자 매칭 공통:** CAN writer도 HMI(`hmi_state.py:269`)도 결국 ego 타깃 IID 하나만 필터(IID==look_at + SG==look_at_signalGroupID). 링크→IID 매핑은 `link_*.mat`의 `look_at_IntersectionID/look_at_signalGroupID/MANUAVER`. 링크 **417 → IID 302, SG 70, MANUAVER -1(LEFT), is_stop_line 1**.

**검증:** prod VPN 라이브에서 `/spat_merged`에 34개 IID + 302(LEFT/SG70 phase3, STR/SG70 phase6) 확인. 재기동 후 `/spat_merged` 구독자 = spat_CAN_writer/stat_display/web_hmi_bridge 확인. CAN 실제 출력은 자차가 417 진입해 `look_at_IntersectionID=302`일 때(현재 0이면 V2X_SPaT_1=0, 정상).

관련: [[siheung_map_senario3]] [[web_hmi_adapt_harness]]
