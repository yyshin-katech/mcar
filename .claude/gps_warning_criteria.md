---
name: gps-warning-criteria
description: rviz(stat_display) GPS 경고 색 판정 기준 — Novatel bestpos 기반이며 빨강은 GPS 가 아니라 ping 8.8.8.8 실패
metadata: 
  node_type: memory
  type: reference
  originSessionId: 7f53efd4-6fc4-4e42-ad25-0d572b5ea702
  modified: 2026-07-27T05:44:17.919Z
---

rviz `workspace_config/ioniq_statdisplay.rviz` 의 GPS 경고는 **`/rviz/jsk/gps_stat` (GPSRTK 오버레이) 하나**가 낸다. 발행자 = `stat_display_node` `GPS_Text_Gen()` (`src/visualization/stat_display/lib/stat_display.cpp`), `diag_timer_` 10 Hz. 입력은 `/diagnostic/cpt7_gps` **단 하나** (유일 발행자 = `cpt7_gps_diagnostic`, 원천은 **Novatel `/sensors/gps/bestpos`·`/inspva`** — ublox 아님).

## 색 판정 (우선순위 순)

| 조건 | 색 | gps_status |
|---|---|---|
| `GPSRTK_StatCode != 0x38` \|\| `lon_std > 0.05` \|\| `lat_std > 0.05` | 🟠 주황 | 0 |
| 위 조건 아님 | 🟢 초록 (0,0.8,0) | 0 |
| `Network_Status == 1` (override) | 🔴 빨강 | 2 |

- `GPSRTK_StatCode`: `bestpos.position_type == "INS_RTKFIXED"` 일 때만 `0x38`, **그 외 전부 0** → RTK Fixed 아니면 무조건 주황.
- `lon_std`/`lat_std` = `bestpos.lon_sigma`/`lat_sigma` (m) → 5 cm 초과 주황.
- `Network_Status` = `ping -c 1 -W 1 8.8.8.8` 실패 시 1 (약 1 s 주기).
- `gps_status != 0` 이면 1 Hz `system_status_check()` 가 `/rviz/jsk/popup` 에 "⚠️ GPS 센서 고장" + 사운드 → **팝업·소리는 ping 실패 때만**, 주황(RTK 미획득)은 색만 바뀐다.

## 함정

1. **빨강 = GPS 상태가 아니라 인터넷 연결 상태.** RTK Fixed 여도 외부망 ping 실패면 빨강 + "GPS 센서 고장" 팝업.
2. **AliveCnt 통신두절 검사는 무력화돼 있다.** `GPS_AliveCnt_Check()` 가 `gps_status=2` 로 올려도 호출 직후 줄의 `gps_status = 0;` 이 덮어씀. 게다가 `cpt7_gps_diagnostic` 은 bestpos/inspva 콜백 카운트가 변할 때만 publish → **GPS 가 끊기면 `cpt7_msg` 가 마지막 값으로 남아 색이 얼어붙는다.**
3. **RTK 판정식이 rviz ↔ HMI 다름.** rviz = `== 0x38` 정확 비교 / pyqt·web_hmi(`utils/hmi_state.py`) = `gps_rtk_code < 2` 경고. 실제 코드값은 0 또는 0x38(56)뿐이라 HMI 쪽은 0x38 이 항상 통과. 5 cm 임계(`GPS_STD_WARN_M`)만 동일.
4. **`GPS_Over` 는 죽은 필드.** `to_control_team_demo.py` 어디서도 세팅하지 않아 항상 msg 기본값 0. 2026-07-27 stat_display 의 GPS_Over 주황 트리거를 제거(commit 2d81350). 단 **msg 필드(`to_control_team_from_local_msg.msg`)와 CAN(`local_CAN_writer` `On_ODD_Stat.GPS_Over`, DBC 3개)은 그대로 유지** — CAN 프레임 불변 정책. 관련 [[feedback_can_frozen_additive]].

## 경고 아닌 것

`/rviz/jsk/gps_std_text` (GpsStdText, `pyqt_hmi/scripts/gps_std_relay.py`) 는 `lat_sigma`/`lon_sigma` 를 cm 로 표시만 하고 **색 고정(청록)** — 임계값·경고 없음.
