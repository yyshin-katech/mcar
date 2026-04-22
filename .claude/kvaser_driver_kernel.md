---
name: Kvaser canlib 드라이버 ↔ 커널 매칭
description: Kvaser canlib이 채널을 0개로 인식할 때의 원인/진단/복구 (kvaser-drivers-dkms와 실행 커널이 어긋나는 문제)
type: project
originSessionId: b7e279de-24ed-4887-b483-808add15ade6
---
이 머신의 Kvaser canlib은 특정 커널 버전에서만 동작한다. `/usr/doc/canlib/examples/listChannels` → "Found 0 channel(s)" 이면 거의 항상 DKMS-커널 미스매치.

- DKMS 빌드된 커널: `5.15.0-139-generic`, `5.15.0-67-generic` (`dkms status` 확인)
- 미지원 커널(예: `6.2.0-060200-generic`)로 부팅되면 `mhydra`/커스텀 모듈이 올라오지 않아 canlib은 채널 0개로 인식
- `lsusb`에 Kvaser 장치(`0bfd:0105 Kvaser USBcan Pro 5xHS` 등)가 보여도, userspace canlib API와 무관한 징후이므로 가려내지 못함
- in-tree `kvaser_usb.ko`(커널 트리 드라이버)는 **SocketCAN용**이라 이 프로젝트의 Kvaser canlib API와 호환되지 않음 → 대체로 쓸 수 없음

**Why:** 커널 업그레이드 후 DKMS 자동 재빌드가 실패(또는 미수행)하면 Kvaser 유저스페이스 스택이 그대로 먹통이 된다. `katech_test.launch`의 모든 CAN writer/reader는 문제없이 뜨지만 실제 버스로는 한 프레임도 나가지 않는다.

**How to apply:** "CAN 데이터가 안 나간다" / "CAN writer는 도는데 차량이 반응이 없다" 류 제보를 받으면, ROS 쪽을 보기 전에 먼저 `uname -r`와 `listChannels`부터 확인한다. 채널 0개면:
1. 1회용으로 5.15.0-139로 부팅: `sudo grub-reboot "Advanced options for Ubuntu>Ubuntu, with Linux 5.15.0-139-generic" && sudo reboot` (다음 부팅부터는 GRUB_DEFAULT=0이라 자동으로 6.2로 복귀)
2. 부팅 후 검증: `uname -r` → 5.15.0-139, `lsmod | grep mhydra` → 모듈 로드, `listChannels` → 5개 채널(USBcan Pro 5xHS)
3. 영구 해결이 필요하면 현재 커널 헤더로 DKMS 재빌드(`sudo dkms install kvaser-drivers/5.38.841 -k $(uname -r)`) 또는 GRUB_DEFAULT를 5.15로 고정. 단 Kvaser 5.38.841은 2023년 초 릴리스라 6.2 이상에서는 빌드 실패할 수 있음.

관련 패키지: `kvaser-canlib-dev` 5.38.841, `kvaser-drivers-dkms` 5.38.841, `kvaser-linlib-dev` 5.38.841.
