---
name: feedback-wslg-wheel-input
description: WSLg/XWayland는 마우스 wheel/클릭을 키보드 포커스가 아닌 커서 위치 기준으로 전달 — PyQt 위젯 wheelEvent에 isActiveWindow 가드 필요
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a685a357-4413-467a-b1a9-271828299584
---

WSLg/XWayland 환경에서 PyQt 창은 마우스 **wheel(및 클릭) 이벤트를 키보드 포커스가 아니라 커서 위치 기준**으로 전달받는다. 그래서 Ctrl+Tab으로 다른 창을 선택해도 커서가 PyQt 창 위에 있으면 wheel이 그 위젯으로 가서 동작이 먹는다(예: vehicle_view 줌인/아웃).

**Why:** pyqt_hmi `main_display.py` 실행 시 "다른 창 선택 후 스크롤해도 HMI가 줌됨" 증상. 코드상 grabMouse/installEventFilter/always-on-top 없음 → 환경(WSLg) 입력 라우팅 특성.

**How to apply:**
- 줌/스크롤 핸들러는 `if not self.isActiveWindow(): event.ignore(); return` 가드 필수. (`src/visualization/pyqt_hmi/scripts/widgets/vehicle_view.py` wheelEvent, 2026-06-04 적용)
- `underMouse()` 가드는 WSLg에서 불안정할 수 있어 정상 줌까지 막을 위험 → isActiveWindow 만 사용.
- "클릭이 다른 창에서 잘 안됨"도 같은 WSLg 입력 라우팅 추정. *다른 창의* 클릭은 PyQt 코드로 정상화 불가(WM/WSLg 레벨). 네이티브 리눅스에서 재현 안 되면 환경 이슈로 확정.

관련: [[feedback-wsl-rostopic-hz]], [[environment]]
