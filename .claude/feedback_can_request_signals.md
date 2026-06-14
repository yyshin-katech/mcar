---
name: CAN Request Signals - Pulse Pattern
description: 모드 변경류 CAN request 신호는 1초 펄스로 송출, request 송출 후 재요청 막는 UX
type: feedback
originSessionId: a6eb69e2-ca85-40c8-af59-d781c0bd2015
---
CAN의 mode change request 류 신호 (예: `On_ODD_Stat::MD_AD_Req`)는 held state가 아니라 **펄스(요청 트리거)**로 다뤄야 한다.

**Why:** DBC value table에 명시되어 있음 — `MD_AD_Req=1`은 "MD -> AD Mode Change Request", `0`은 "Mode Change Not Req". 즉 1은 요청 신호이지 모드 자체가 아님. 차량이 모드 전환 처리한 뒤엔 0으로 돌아가야 정상. 계속 1을 유지하면 차량 측에서 재요청으로 오인하거나 스펙 위반.

**How to apply:**
- HMI 버튼 → CAN 송출 체인 만들 때 1초 펄스 + 자동 0 복귀 패턴 사용 (`QTimer.singleShot`).
- 차량이 이미 그 모드면 버튼 비활성화 (`setEnabled(False)`)해서 재요청 차단.
- 비슷한 request 시그널 (예: 차후 추가될 모드/기능 요청 신호) 작업 시 동일 패턴 적용.
- 새 CAN 신호 작업 전엔 DBC value table description (`VAL_` 항목) 먼저 확인해서 request인지 state인지 판단.

**구현 위치 (참고):**
- `pyqt_hmi/scripts/widgets/main_window.py` — `on_auto_button_clicked` + `_reset_mode_request`, `periodic_update`의 `auto_button.setEnabled` 토글
- `local_CAN_writer.cpp` — `/vehicle/mode_command` 구독 → `md_ad_req` 멤버 → `On_ODD_Stat` 송출 (단, 이쪽엔 timeout 없음. PyQt 죽으면 마지막 값 그대로 유지되니 향후 timeout 추가 검토 필요)
