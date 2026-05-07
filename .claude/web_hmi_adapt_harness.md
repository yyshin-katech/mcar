---
name: web-hmi-adapt harness
description: 브랜치 간 web_hmi 데이터 구조 어댑트용 파이프라인 하네스 (.claude/agents + .claude/skills)
type: reference
originSessionId: 3ea36247-9ca8-4af1-aa8b-d69a44426fbf
---
`web_hmi`는 ioniq5_hmi_dev 기준으로 작성됨. 다른 브랜치(siheung_dev 등)로 가져갈 때 토픽/메시지/launch/맵 구조 차이를 어댑트하는 파이프라인 하네스가 `.claude/`에 있음.

**위치**:
- `.claude/agents/match-detective.md` — 분석 (변경 없음)
- `.claude/agents/bridge-adapter.md` — 외과적 패치 적용
- `.claude/agents/adapt-verifier.md` — 빌드/import/launch 파싱/맵 로드 검증
- `.claude/skills/web-hmi-adapt/SKILL.md` — 오케스트레이터

**트리거 키워드** (CLAUDE.md 변경이력 참조): "web_hmi 데이터 매칭", "web_hmi 어댑트", "브랜치 매칭", "다시 어댑트".

**산출물**: `_adapt_workspace/{01_match,02_adapter,03_verify,00_consolidated}.md` (작업 산출물, 미커밋).
