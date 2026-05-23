---
name: V2N MQTT 16-byte 헤더 = 표준 V2N container 0x04 (메시지별 fid/psid 다름)
description: 시흥 V2N MQTT payload 앞 16 byte 는 경찰청 V2N 규격 `<표 4-11>` 단일 메시지 전송 V2N container (sem_length=0). fid/psid 는 메시지 타입(BSM/SPaT/TLSM/통행지시)별로 다르며 ~/protocol xlsx 가 단일 출처.
type: project
originSessionId: a5726bf9-12cf-4b64-86e5-fc80d40279c4
---
`siheung_v2x` 의 `mqtt_spat_rx_node` / `mqtt_bsm_tx_node` 의 16-byte 헤더는 **경찰청 V2N 정보연계 규격 `<표 4-11>` 단일 메시지 전송 V2N container (container_type=0x04)** 의 fixed portion (sem_length=0 일 때 16 byte) 와 정확히 일치.

```
offset 0  : container_type (8 bit)  = 0x04 (단일 메시지 전송, <표 4-3>)
offset 1  : version       (8 bit)  = 0x00
offset 2-3: fid           (16 bit) = 메시지 타입별 (BE, 아래 표 참조)
offset 4  : standard_type (8 bit)  = 0x01 (KS 표준, <표 4-5>)
offset 5  : sem_length    (8 bit)  = 0x00 (SEM 없음)
offset 6  : flags         (8 bit)  = 0x00
offset 7  : message_id    (8 bit)  = sequence counter (container-level msg id)
offset 8-11: psid         (32 bit) = 메시지 타입별 (BE, 아래 표 참조)
offset 12-15: message_length (32 bit) = BE-u32 (UPER 페이로드 길이)
offset 16..   : J2735 UPER MessageFrame
```

**메시지 타입별 fid/psid (출처: `~/protocol/경4 V2N FID PSID 정의안 (1).xlsx`)**

| 메시지 | 방향 | fid | psid (hex) | psid (dec) | 토픽 suffix (xlsx) |
|--------|------|------|-----------|-----------|-------------------|
| BSM | 차량→센터 | `0xFF10` | `0x00014082` | 82050 | `/bsm` |
| SPaT | 센터→차량 | `0xFF11` | `0x00014085` | 82055 | `/spat` |
| TLSM | 센터→차량 | `0xFF12` | `0x00014100` | 82176 | `/tlsm` |
| 통행지시 돌발 (제공) | 센터→차량 | `0xFF00` | `0x00014101` | 82177 | `/trf_drct/req` |
| 통행지시 결과 (수집) | 차량→센터 | `0xFF09` | `0x00014101` | 82177 | `/trf_drct/res` |

**Why:** 초기 2026-05-23 에는 BSM TX 코드가 SPaT 의 fid/psid (`0xFF11`/`0x00014085`) 를 그대로 쓰는 버그가 있었음 (mqtt_spat_rx_node 코드를 복사하면서 fid/psid 도 함께 복사). 사용자 제공 xlsx 시트 (PSID/FID 두 시트) 로 정정 후 commit 692393e 에 반영 (BSM: 0xFF10/0x00014082).

**Why standard_type=1 (KS):** 한국 KS 표준 메시지 (J2735 한국화). **container_type=0x04** 는 경찰청 우선 적용 컨테이너 (`section18.txt L93`).

**How to apply:**
- 새 V2N 메시지 송수신 노드 추가 시 fid/psid 를 xlsx 에서 먼저 확인. 다른 노드 코드 복사 금지 (fid/psid 가 메시지별로 다름).
- `SpatDecoder::skipMqttV2nHeader()` (mqtt_spat_rx_node.cpp L138-152) 가 `container_type==0x04` 검사 후 16 byte 반환.
- 코드 주석은 `<표 4-11>` 필드명 + xlsx 출처를 함께 인용 (commit 34253ce, 692393e 패턴).

**현재 운영 토픽 (launch/siheung.launch):**
- SPaT RX: `V2N/1321103202/trf_drct/spat` (xlsx 문서상 `/spat` 와 다름 — 운영자가 실제 발행하는 토픽 기준)
- BSM TX: `V2N/1321103202/bsm` (xlsx 와 일치)

검증 (2026-05-23, 테스트 브로커 121.137.106.141:23312, 토픽 `V2N/1321103202/trf_drct/spat`):
- decoded SPaT: 1 intersection, iid=80, 12 movements
- MANUAVER=-1 → SigGrp=70 LEFT Phase=3 (STOP/red), minEnd 1초씩 카운트다운
- 토픽 sibling: `V2N/1321103202/trf_drct/tlsm` (TLSM, 별도 포맷)

규격 정합성 검증 (2026-05-23, protocol-spec-check 하네스, `[ITSK-00150-2] 경찰청 V2N 정보연계 규격 hwpx`):
- 16-byte 헤더는 표준 `<표 4-11>` V2N container 0x04 (sem_length=0 일 때 fixed=16 byte) 와 완전 일치.
- QuadKey `1321103202` = 부속서 H 산출 zoom=10, lat=37.38113, lon=126.72394 (시흥). 재현 확인.
- 토픽 suffix `/trf_drct/spat`, `/bsm` 중 `/bsm` 은 xlsx 와 일치, `/trf_drct/spat` 은 운영자 확장 (section24 "사업자별 확장 가능" 으로 허용 해석).
- BSM Part I (coreData 15개 필드) 모두 J2735 매핑 정확. accuracy, accelSet.vert, brakes.{traction/abs/scs/brakeBoost/auxBrakes} 는 INSPVA/VCAN 미수집 항목으로 unavailable 고정.
