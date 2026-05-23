---
name: V2N FID/PSID xlsx (경찰청 정의안)
description: 경찰청 V2N 정보연계 메시지별 fid/psid/방향/토픽 정의 xlsx 파일 위치. siheung_v2x 노드의 16-byte 헤더 fid/psid 필드 단일 출처.
type: reference
originSessionId: a5726bf9-12cf-4b64-86e5-fc80d40279c4
---
**파일**: `/home/sim/protocol/경4 V2N FID PSID 정의안 (1).xlsx`

**시트 구성**:
- `PSID` 시트: 메시지 타입 ↔ PSID(decimal/hex) 매핑 (BSM=82050, SPaT=82055, TLSM=82176, 통행지시=82177)
- `FID` 시트: 메시지 타입 ↔ FID(16-bit hex) ↔ 방향(차량↔센터) ↔ MQTT 토픽 suffix

**파싱**: openpyxl 로 직접 읽기 (tesseract 등 OCR 불필요). 헤더 행이 있으므로 `iter_rows(min_row=2)` 형태로 순회.

**관련**: `<표 4-11>` 표준 V2N container 의 offset 2-3 (fid) / offset 8-11 (psid) 값 결정에 사용. 메모리 `project_mqtt_v2n_spat.md` 의 표가 이 xlsx 에서 유도됨.

**주의**: 경찰청 외부 표 (`cits-architecture.itskorea.kr/hp/arch/{fid,psid}.do`) 는 전국 V2X 공통이지만 본 xlsx 는 시흥 시범사업 운영자 부여 값이 포함될 수 있음 — V2N 메시지 추가/변경 시 본 xlsx 를 우선 확인.
