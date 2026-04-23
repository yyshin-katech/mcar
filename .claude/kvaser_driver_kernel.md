---
name: Kvaser canlib 드라이버 ↔ 커널 매칭
description: Kvaser canlib이 채널을 0개로 인식할 때의 원인/진단/복구. 5.15 DKMS만 빌드됨, 6.2 DKMS 시도 실패. 2026-04-23 Wi-Fi 문제 해결되어 5.15가 일상 운영 커널로 확정 (GRUB/apt hold 고정)
type: project
originSessionId: b7e279de-24ed-4887-b483-808add15ade6
---
이 머신의 Kvaser canlib은 특정 커널 버전에서만 동작한다. `/usr/doc/canlib/examples/listChannels` → "Found 0 channel(s)" 이면 거의 항상 DKMS-커널 미스매치.

- DKMS 빌드된 커널: `5.15.0-139-generic`, `5.15.0-67-generic` (`dkms status` 확인)
- 미지원 커널(예: `6.2.0-060200-generic`)로 부팅되면 `mhydra`/커스텀 모듈이 올라오지 않아 canlib은 채널 0개로 인식
- `lsusb`에 Kvaser 장치(`0bfd:0105 Kvaser USBcan Pro 5xHS` 등)가 보여도, userspace canlib API와 무관한 징후이므로 가려내지 못함
- in-tree `kvaser_usb.ko`(커널 트리 드라이버)는 **SocketCAN용**이라 이 프로젝트의 Kvaser canlib API와 호환되지 않음 → 대체로 쓸 수 없음

**Why:** 커널 업그레이드 후 DKMS 자동 재빌드가 실패(또는 미수행)하면 Kvaser 유저스페이스 스택이 그대로 먹통이 된다. `katech_test.launch`의 모든 CAN writer/reader는 문제없이 뜨지만 실제 버스로는 한 프레임도 나가지 않는다.

**How to apply:** "CAN 데이터가 안 나간다" / "CAN writer는 도는데 차량이 반응이 없다" 류 제보를 받으면, ROS 쪽을 보기 전에 먼저 `uname -r`와 `listChannels`부터 확인한다. 채널 0개면 DKMS↔커널 미스매치로 진단.

**✅ 2026-04-23 해결: 5.15.0-139에서 Wi-Fi 복구됨 → 5.15가 일상 운영 커널로 확정.**
초기 진단에선 "5.15 부팅 시 wifi 동작 안 함"이라 6.2를 일상 운영 커널로 쓰자고 결론 냈으나, 실제 원인은 기본 iwlwifi가 AX211(51f1) 미지원이었을 뿐. `linux-modules-iwlwifi-5.15.0-139-generic` 패키지로 해결 (상세: wifi_ax211.md). Kvaser + Wi-Fi 둘 다 5.15에서 동작 → 위의 "다음에 다시 시도할 때 선택지 #2"가 실제로 성공한 경로.

**부팅 커널 고정 (2026-04-23):**
- `/etc/default/grub`: `GRUB_DEFAULT="gnulinux-advanced-edc45fda-d455-4d76-828d-988ba02c3faa>gnulinux-5.15.0-139-generic-advanced-edc45fda-d455-4d76-828d-988ba02c3faa"` (백업: `/etc/default/grub.bak.20260423`)
- `apt-mark hold`: `linux-{generic,image-generic,headers-generic,modules-iwlwifi-generic}-hwe-20.04` 메타 4종 + `linux-{image,modules,modules-extra,modules-iwlwifi,headers}-5.15.0-139-generic` 5종
- 원래 GRUB은 TIMEOUT=0 hidden에 기본값이 6.2.0-060200으로 설정돼 있었음 (위험). 지금은 5.15.0-139로 고정.
- 6.2.0-060200 커널 패키지(`linux-image-unsigned-6.2.0-060200-generic`, `linux-modules-6.2.0-060200-generic`, `linux-headers-6.2.0-060200*`)는 제거하지 않고 남김 — broken deps 원인이지만 사용자 요청 시에만 제거.

## 2026-04-23 시도: Kvaser 5.51.461 for kernel 6.2.0-060200-generic (실패, 진행중)

### 한 것
1. Ubuntu mainline PPA에서 `linux-headers-6.2.0-060200-generic` 설치 (libc6/libssl3 의존은 `--force-depends`로 무시)
2. `ppa:ubuntu-toolchain-r/test` 추가 후 `gcc-12` 설치 (5.15 커널은 gcc-9로 빌드됐지만 6.2는 gcc-12 고정)
3. 헤더 패키지에 포함된 `tools/objtool/objtool`, `scripts/basic/fixdep`, `scripts/mod/modpost`, `scripts/mod/mk_elfconfig`는 GLIBC_2.33/2.34 요구 바이너리라 Ubuntu 20.04(focal, glibc 2.31)에선 실행 불가 → `linux-6.2.tar.xz` 소스 받아 각각 gcc-12로 리빌드
4. Kvaser 5.38.841은 6.2에서 `do_exit` 심볼이 더 이상 export 되지 않아 빌드 실패 → Kvaser에서 `linuxcan_5_51_461.tar.gz` 받아 `/usr/src/linuxcan-5.51.461/`에 배치, dkms.conf에 `PACKAGE_VERSION="5.51.461"` 추가
5. `config.mak`의 `KBUILD_EXTRA_SYMBOLS`가 `kvcommon` 자기 자신을 참조하는 버그 회피 패치:
   `KBUILD_EXTRA_SYMBOLS = $(if $(filter-out kvcommon,$(KV_MODULE_NAME)),$(KBUILD_EXTMOD)/../common/Module.symvers)`
6. `dkms install linuxcan/5.51.461 -k 6.2.0-060200-generic` → 모듈 8개 전부 빌드·설치 완료 (`/lib/modules/6.2.0-060200-generic/updates/dkms/`)

### 막힌 지점
`modprobe kvcommon` → `Exec format error`. dmesg:
```
module: x86/modules: Skipping invalid relocation target, existing value is nonzero for type 1, loc ..., val ffffffffc112eab5
```
`dkms status`도 `installed (WARNING! Diff between built and installed module!)` 경고 표시.

### 원인 추정
커널 6.2 mainline PPA는 **Ubuntu 22.04 이상 기준**으로 빌드된 바이너리 툴(objtool/modpost 등)을 포함하고, 이는 glibc 2.33/2.34를 요구한다. 우리는 glibc 2.31(focal)이라 툴을 소스에서 리빌드했는데, 그렇게 만든 툴이 생성하는 `.ko`의 섹션/심볼/릴로케이션 포맷이 Ubuntu가 정식 빌드한 결과물과 미묘하게 어긋난다(특히 `CONFIG_RETHUNK`/`CONFIG_X86_KERNEL_IBT` 등 6.2에서 추가된 보안 피처 관련). 결과물 `.ko`는 만들어지지만, 런타임 릴로케이션 단계에서 타입 1(R_X86_64_64) 항목의 기존 값이 0이 아니라 커널 로더가 거부.

근본적으로 **Ubuntu 20.04 focal + mainline 6.2 커널 + 외부 커널 모듈** 조합은 업스트림이 지원하지 않는 구성. 툴 버전 맞추기를 아무리 해도 완전히는 어긋남을 잡기 어려움.

### 6.2 재시도 자산 (2026-04-23 사용자 결정: 보존)
2026-04-23 Wi-Fi가 5.15에서 해결되어 "5.15 정규 운영" 방침으로 확정됐지만, 6.2 DKMS 재시도 가능성을 열어두기 위해 아래 자산들은 **의도적으로 남겨둠**. `linux-{image-unsigned,modules,headers}-6.2.0-060200*` 패키지도 제거하지 않음.

- `/usr/src/linuxcan-5.51.461/` — Kvaser 5.51.461 소스 + dkms.conf (PACKAGE_NAME="linuxcan", 기존 kvaser-drivers와는 별개 패키지). 실제로 5.15.0-139에도 빌드되어 동작 중(`dkms status`에 `linuxcan, 5.51.461, 5.15.0-139-generic: installed` 확인).
- `/usr/src/linux-headers-6.2.0-060200-generic/` — mainline 헤더. tools/objtool, scripts/mod/{modpost,mk_elfconfig}, scripts/basic/fixdep는 우리가 리빌드한 바이너리로 교체됨(`objtool.glibc234.bak` 원본 백업). **삭제되면 6.2 재시도 시 이 리빌드 작업을 처음부터 반복해야 함**이 보존의 핵심 이유.
- `/usr/src/kvaser-drivers-5.38.841/config.mak` — KBUILD_EXTRA_SYMBOLS 패치본 (5.15용에는 영향 없음, `.bak` 있음)
- `/lib/modules/6.2.0-060200-generic/updates/dkms/` — 8개 `.ko` 설치됨 (릴로케이션 에러로 로드는 실패하지만 두는 데 해는 없음). `dkms status`: `linuxcan, 5.51.461, 6.2.0-060200-generic: installed (WARNING! Diff between built and installed module!)`
- `/tmp/linux-6.2/` — 이미 삭제됨(재부팅으로 날아감). 재시도 시 다시 받아야 함.

정말 정리하고 싶을 때: `dkms remove linuxcan/5.51.461 -k 6.2.0-060200-generic` → `apt-get purge linux-image-unsigned-6.2.0-060200-generic linux-modules-6.2.0-060200-generic linux-headers-6.2.0-060200-generic linux-headers-6.2.0-060200` → `update-grub`. 단 gcc-12/libstdc++-12-dev broken deps(`ppa:ubuntu-toolchain-r/test` + focal glibc 2.31 충돌)는 별건이라 그대로 남음.

### 다음에 다시 시도할 때 선택지
1. **Ubuntu 22.04 jammy로 OS 업그레이드** — 이게 "정공법". glibc 2.35라 PPA 헤더 바이너리가 그대로 실행됨. do-release-upgrade는 큰 변경이라 별도 세션/점검 필요.
2. **Kvaser 더 최신 릴리스 확인 후 커널 5.15-generic wifi 드라이버 백포트/교체** — CAN이 되는 5.15를 계속 쓰되 wifi만 어떻게든 올리는 방향. 5.15 wifi가 왜 안 되는지 조사 선행.
3. **SocketCAN 전환** — 코드 전체가 canlib에 묶여 있어 실질적으로 불가(21개 실행파일).
4. **릴로케이션 이슈 직접 디버깅** — `readelf -r` 로 문제 위치가 어떤 심볼·어떤 C 코드에서 나온 건지 추적, `CONFIG_RETHUNK` 등을 Kvaser Makefile에서 끌 수 있는지 실험. 시간 많이 듦.

**원칙(2026-04-23 갱신): 5.15.0-139가 이 머신의 "정규 운영 커널".** GRUB/apt-hold로 고정됨. CAN과 Wi-Fi 둘 다 여기서만 동시에 동작. 위 선택지 1~4는 더 이상 필요 없음(선택지 #2가 성공한 결과).

관련 패키지: `kvaser-canlib-dev` 5.38.841, `kvaser-drivers-dkms` 5.38.841, `kvaser-linlib-dev` 5.38.841. (canlib 유저스페이스는 5.38.841 그대로, 커널 모듈만 5.51.461로 올리려 시도)

**재발 이력:** 2026-04-22, 2026-04-23 연속 동일 증상. 2026-04-23에 6.2용 DKMS 빌드 시도했으나 릴로케이션 이슈로 로드 실패 → 같은 날 5.15에서 Wi-Fi 복구 성공하여 "5.15 고정"으로 방향 확정. 현시점 이 머신에서 "CAN 데이터가 안 나간다" 제보는 "uname -r 확인 → 5.15.0-139가 아니면 GRUB/hold가 풀린 것"으로 즉답 가능.
