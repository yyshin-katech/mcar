---
name: "Disable undo" means restore correct logic, not literal revert
description: 무력화 해제 요청 시 단순히 강제 override 라인만 떼지 말고, 그게 가려주던 버그까지 정리
type: feedback
originSessionId: 7bc83e9b-1c21-49ec-9db9-d586acbc5f2f
---
코드에 박혀있는 무력화 라인(`status = 0` 같은 강제 override)을 "해제"하라는 요청은, 그 라인만 단순 삭제가 아니라 **이전부터 잠재되어 있던 로직 버그까지 정상화**하는 의미로 해석한다.

**Why:** V2X 진단 무력화 해제 작업에서 사용자는 `v2x_status = 0; v2x_msg.V2X_StatCode = 0;` 두 줄을 떼라고 했지만, 그것만 떼면 남은 `||` 분기 구조가 (status=2, StatCode=0) 조합에서 잘못 녹색으로 빠지는 게 명백했음. 이걸 짚고 GPS와 동일한 우선순위 분기로 재구성한 변경을 사용자가 그대로 수용. "literal undo가 아니라 동작이 의도대로 되도록"이 사용자 의도였음.

**How to apply:**
- 무력화 라인 발견 시 그 라인이 **무엇을 가리고 있었는지** 먼저 조사. 가린 사이에 잠재 버그가 들어왔을 가능성 높음.
- 단순 삭제 후 로직이 깨끗하지 않으면, 사용자에게 "단순 undo로는 (X 조건)에서 잘못 동작하니 분기 구조도 바꾸겠다"고 한 줄 알리고 적용. 보통 OK 받음.
- 다른 비슷한 노드(예: 진단 모듈 8개)의 분기 패턴을 reference로 삼아 일관성 있게 정리.
