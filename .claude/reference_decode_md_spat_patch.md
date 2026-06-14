---
name: decode_md spat_udp_decode 호환 패치
description: ~/decode_md/spat_udp_decode.py (J2735 ASN UPER 디코더) 를 WSL Python + 최신 asn1tools 에서 컴파일/실행 가능하게 하는 preprocess_asn() 보강 패치
type: reference
---

`/home/yuyeong/decode_md/spat_udp_decode.py` 는 OBU(192.168.1.5) → PC dst port 9999 SPaT UDP 패킷을 tkinter GUI 로 표출하는 개인 도구. 원래 Windows Python 3.12 환경 가정으로 작성됐고 WSL Python 3.8 + asn1tools 0.167.0 에서 그대로는 안 돈다.

**필요한 시스템 셋업 (1회):**
- `sudo apt-get install -y python3-pip`
- `pip3 install --user asn1tools` (→ asn1tools 0.167.0, bitstruct, pyparsing)

**preprocess_asn() 보강 (필수, 적용됨):**
1. **두 번째 extension marker `...` 라인 제거** — `_J2735_20250110_KSR1600_4세부.asn` 의 `PersonalDeviceUserType` 등 ENUMERATED 가 `..., 추가항목..., ...` 패턴을 쓰는데 asn1tools 가 두 번째 단독 `...` 를 거부. 같은 brace depth 에서 두 번째 이후의 단독 `...` 라인을 카운트 기반으로 제거. 주석 동반 (`..., -- 호환성...`) 도 매치.
2. **닫는 `}` 직전 trailing comma 정리** — `...` 라인 제거 후 마지막 항목 뒤 컴마가 남으면 컴파일 실패. 주석 라인이 컴마와 `}` 사이에 끼어있어도 매치되는 regex 사용.

**LOCAL_IP 변경:**
- `192.168.1.3` → `0.0.0.0` — WSL eth0 IP 가 172.19.x 라 그 IP 로 바인딩 불가.

**실행 위치:**
- WSL 에서 실행해도 UDP listen 은 되지만 192.168.1.5 OBU 패킷은 mirrored 네트워킹 아니면 도달 안 함 → **Windows Python (`/mnt/c/Users/you0d/AppData/Local/Programs/Python/Python312/python.exe`) 에서 실행**해야 실수신 가능.
- 기동: `cd /home/yuyeong/decode_md && python.exe spat_udp_decode.py` (WSL 안에서 Windows python 호출. cwd가 \\wsl.localhost 경로로 매핑되어 ASN/스크립트 모두 읽힘)
- 검증: 192.168.1.5:34923 → 0.0.0.0:9999 에서 1초당 17+ SPaT 패킷 디코드 성공 (2026-05-28 확인)

**Why:** SPaT 디코더가 mcar 트리 밖 `~/decode_md/` 에 있어 git 추적 안 됨. 다음에 같은 PC 에서 작업 시 또 컴파일 실패하면 패치가 사라졌는지 의심해서 spat_udp_decode.py 의 `preprocess_asn` 마지막에 `solo_ellipsis_re` / `ellipsis_at_depth` / trailing comma regex 가 살아있는지 확인.

**How to apply:** 사용자가 "obu spat 뷰어 실행" 요청 시 이 파일 참고. asn1tools.ParseError "Expected '}', found ','" 가 나오면 본 메모의 두 가지 보강이 빠진 것.
