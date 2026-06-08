---
name: V2X Package
description: siheung_v2x 패키지 구조 - SPaT(KSR1600) + SDSM(2020) 디코딩, OBU 헤더 포맷
type: project
---

## siheung_v2x 패키지 (src/v2x/siheung_v2x)

기존 j3224_decode(j2735_decode)에서 이름 변경됨.

### 구조
- `j2735_decode.cpp` — 메인 노드, SPaT 디코딩 (KSR1600 헤더, WSMP 자동 스킵)
- `sdsm_decode.cpp` — SDSM 디코딩 (2020 헤더, 별도 번역 단위로 심볼 충돌 방지)
- `ffasn1-j2735-2026-KSR1600.c/.h` — KSR1600 ASN.1 라이브러리 (소스 컴파일, .so는 ARM64라 x86에서 사용 불가)
- `ffasn1-j2735-2026.h` → KSR1600.h 심볼릭 링크 (.c가 이 이름으로 include)
- ASN.1 전처리 참고: `~/decode_md/project_asn1_preprocessing.md` (asn1tools용 KSR1600 전처리 방법)

### OBU UDP 패킷 포맷 (port 9999)
```
[0] Frame Type   : 0=OBU→PC, 1=PC→OBU
[1] Seq No       : 0~255
[2] msg source   : 0=from RSU, 1=from Uu
[3] isMsgFrame   : 0=MessageFrame, 1=not (경2 SDSM은 1)
[4] Reserved
[5~] (옵션) WSMP 서브헤더 [03 80 + BER길이] + J2735 body (UPER)
```
실제 OBU는 WSMP 서브헤더(`03 80` + BER 길이)가 포함됨. `skipWsmpHeader()`로 자동 감지/스킵.

### 시흥 교차로 정보 (pcapng 검증 완료, 2026-04-08)
- 네트워크: OBU=192.168.1.5 → PC=192.168.1.3, UDP 9999
- 교차로: region=1200, id=301(port 46757), id=24931(port 53629)
- 301 교차로: 11개 movement (SG 50/60/70/80, STR/LEFT/PED), 큰 패킷(214B)
- 24931 교차로: 3개 movement (SG 50/60/80), 축약 패킷(81B)
- movementName: "STRAIGHT", "LEFT", "PEDESTRIAN" (Windows 테스트시 STR/LEFT/PED 약어와 다름)
- **minEndTime**: 표준 TimeMark가 아닌 카운트다운 값 (남은 시간, 1/10초 단위, 연속 패킷에서 감소 확인됨)

### 디코딩 흐름
- WSMP 서브헤더 자동 감지/스킵 (03 80 패턴)
- isMsgFrame=0 → UPER MessageFrame → messageId=19이면 SPaT(KSR1600) → `/siheung_spat`
- isMsgFrame=1 → SDSM UPER 직접 디코딩(2020) → `/obu/sdsm`

### SPaT 매칭
- `/localization/to_control_team` 구독 → intersection_id, signal_group_id, MANUAVER 수신
- MANUAVER: -1→LEFT, 0→STR, 1→RIGHT 으로 movementName 매칭
- signalGroup: 50/60/70/80 등

### 테스트
- `test/spat_sample_sender.py` — 샘플 SPaT 데이터 UDP 전송기 (`--wsmp` 옵션으로 WSMP 헤더 포함 가능)
- `test/pcap_spat_decoder.cpp` — pcapng에서 SPaT 추출/디코딩 → JSON 출력 (ffasn1 라이브러리 사용)
- Windows 테스트 작업 기록: `~/decode_md/` (메모리, spat_udp_decode.py, sample.pcapng)

**Why:** 시흥 자율주행 시나리오에서 V2X 신호 정보 수신 필요
**How to apply:** V2X 관련 작업 시 이 패키지 구조와 OBU 헤더 포맷 참조
