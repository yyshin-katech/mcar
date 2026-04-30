---
name: NUC13ANH-B 오디오 환경 및 PulseAudio 복구 절차
description: 시스템 사운드(유튜브, ROS sound_play 경고음 등) 무음 시 PulseAudio 카드 프로파일 강제 활성화 절차
type: reference
originSessionId: d3d85e57-260b-4d7c-b424-d66fb3d7987b
---
## 하드웨어 / 드라이버 상태
- 사운드 카드 1개만 인식: Realtek ALC256 (PCH HDA, `/proc/asound/card0`)
- 재생 디바이스: ALC256 Analog (3.5mm 잭) 1개뿐
- **HDMI/DP 오디오 출력 없음** (모니터 스피커 사용 불가)
- 결론: **유일한 출력 경로는 후면 3.5mm 잭(헤드폰/스피커 연결 필요)**

## HDMI/DP 오디오가 안 나오는 근본 원인 (2026-04-28 진단)
- **iGPU(00:02.0, Raptor Lake-P GT2 `8086:a7a0`)에 i915 드라이버가 PCI bind되지 않음**
  - `/sys/bus/pci/devices/0000:00:02.0/driver` 파일 없음
  - `/dev/dri/` 디렉토리 자체 없음
  - i915 모듈은 lsmod에 보이지만 use count = 0
- 시스템은 **EFI Framebuffer(`efifb`)** 로 fallback 출력 중 (`/proc/fb` = `0 EFI VGA`)
- Xorg는 `modesetting`/`intel`이 아니라 **`fbdev` 드라이버** 사용 (efifb 위에 출력)
- 결과: `snd_hda_intel`이 audio component를 못 찾아 `couldn't bind with audio component` 발생 → HDMI/DP 사운드 카드 자체가 만들어지지 않음
- 모니터 2대(HDMI + USB-C/DP)가 efifb로 화면은 나와도 **모니터 스피커는 시스템 sink 목록에 없어 사용 불가**
- NVIDIA 패키지/모듈 없음, modprobe.d에 i915 blacklist 없음 → 차단된 상태가 아니라 **부팅 시 i915 probe 실패**
- 추정 원인: efifb의 BAR 점유, 5.15.0-139 커널의 Raptor Lake-P 지원, 또는 BIOS multi-display 설정. 정확 진단·복구는 GUI 끊길 위험이 있어 재부팅 가능 시점에 별도 진행 필요.

## 헤드폰 잭 연결 여부 빠른 확인
```bash
amixer -c 0 cget numid=7   # 'Headphone Jack' state: off=미연결, on=연결됨
```

## 증상: 유튜브/시스템 사운드 무음
- `pactl list short sinks` 결과에 `auto_null` (module-null-sink, "Dummy Output")만 있음
- 카드 활성 프로파일이 비어있거나 `off`로 빠진 상태
- 모든 출력 프로파일이 "available: 아니요"로 표시되는 경우가 있음 (잭 감지 이슈)

## 복구 절차
```bash
# 카드 프로파일 강제 활성화
pactl set-card-profile 0 output:analog-stereo

# default sink 재설정
pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo

# 음소거/볼륨 확인
pactl set-sink-mute alsa_output.pci-0000_00_1f.3.analog-stereo 0
pactl set-sink-volume alsa_output.pci-0000_00_1f.3.analog-stereo 70%

# 검증 (LANG=C 권장 — 한국어 로케일에서 grep 매칭 실패 회피)
LANG=C pactl info | grep -i "default sink"
LANG=C pactl list cards | grep "Active Profile:"
LANG=C pactl list sinks | grep -E "Name:|Mute:|Volume: front|State:|Active Port:"
```

## 주의 사항
- 프로파일이 다시 `off`로 빠지면 PulseAudio 재시작: `pulseaudio -k && pulseaudio --start`
- 이미 재생 중이던 브라우저/앱은 더미 sink에 묶여있을 수 있어 **앱 재시작 또는 페이지 새로고침** 필요
- 한국어 로케일에서 `pactl info`/`list` 결과의 `Default Sink:`/`Active Profile:` 라벨이 한국어로 나와 grep 패턴 안 잡히는 경우가 있어 **LANG=C** 권장

## 관련 ROS 의존
- `stat_display` 패키지의 경고음 (sound_play_node `/robotsound`)도 이 시스템 sink로 출력됨
- `katech_test.launch`에는 `sound_play` 노드가 없음 — `diagnostic_only.launch`에서만 띄움 ([diagnostic_only.launch:28](launch/diagnostic_only.launch#L28))
