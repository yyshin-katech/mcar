---
name: wifi_ax211
description: Intel Wi-Fi 6E AX211 (PCI 8086:51f1) on kernel 5.15.0-139 — iwlwifi 51f1 미지원, linux-modules-iwlwifi-*-generic 패키지로 해결
type: project
originSessionId: cdbed72c-1fa0-4e03-bfc0-1b99c730b014
---
IPC Wi-Fi 카드 = Intel Wi-Fi 6E AX211 (PCI ID `8086:51f1`), 인터페이스명 `wlo1`, 저장된 AP `IPR-5000W_5G_63B220`.

현재 커널 `5.15.0-139-generic`의 `/lib/modules/.../kernel/drivers/net/wireless/intel/iwlwifi/iwlwifi.ko`는 AX211(51f1)을 지원하지 않는다 (`modinfo -F alias iwlwifi | grep 51f1` → 빈 결과). 그래서 드라이버가 로드돼도 장치에 바인딩되지 않고 wlan 인터페이스가 생성되지 않는다.

**Why:** Ubuntu 20.04 focal의 5.15 HWE 커널 기본 패키지는 Alder Lake 이후 AX211을 포함하지 않음. Ubuntu는 별도 `linux-modules-iwlwifi-*-generic` 패키지로 업데이트된 iwlwifi 모듈(`iwlwifi-stack-public:master:11510`)을 제공하며 이게 AX211을 포함한다.

**How to apply:** 현재 커널 유지한 채 Wi-Fi 복구하려면:
1. `apt-get download linux-modules-iwlwifi-$(uname -r) linux-modules-iwlwifi-generic-hwe-20.04`
2. `sudo dpkg -i linux-modules-iwlwifi-*.deb` (apt install은 기존 broken deps — gcc-12, linux-headers-6.2.0 — 때문에 막힘, dpkg 직접 사용)
3. `sudo modprobe -r iwlmvm iwlwifi; sudo modprobe iwlwifi`
4. 모듈 경로는 `/lib/modules/$(uname -r)/ubuntu/iwlwifi/iwlwifi.ko`가 되고 `wlo1`이 뜸. NetworkManager가 저장된 AP에 자동 연결.

**절대 하지 말 것:**
- `backport-iwlwifi-dkms` (focal 버전 8324): BUILD_EXCLUSIVE_KERNEL가 `^((5\.[0-3]|4\.))`로 제한되어 5.15에서 빌드 안 됨. 설치해도 무용지물.
- 커널 업그레이드: Kvaser DKMS가 5.15.0-139에만 빌드되어 있어 커널 교체 시 CAN이 끊김 (kvaser_driver_kernel.md 참조).

**향후 커널 변경 시:** 새 커널 버전에 해당하는 `linux-modules-iwlwifi-<신버전>-generic` 설치 필요. 메타패키지 `linux-modules-iwlwifi-generic-hwe-20.04`는 현재 HWE 커널 버전을 따라가므로 HWE 업데이트되면 자동으로 올바른 모듈 끌어옴.

**커널 고정 (2026-04-23):** 이 머신은 Kvaser 때문에 5.15.0-139-generic에 묶여 있음. `GRUB_DEFAULT`를 `gnulinux-advanced-...>gnulinux-5.15.0-139-generic-advanced-...`로 지정하고 관련 커널/메타패키지 9개를 `apt-mark hold`함. 상세와 배경은 kvaser_driver_kernel.md 참조.
