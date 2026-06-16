#!/usr/bin/env bash
# audio_watchdog.sh — 시스템 사운드 잠금 워치독
#
# diagnostic_only.launch 에서 실행. 소프트웨어 동작 중 음소거/음량 변경/프로파일 off 를
# 주기적으로 원복해 안전 경고음(/robotsound: 테이크오버·어린이보호구역·ODD·AEB 등)의
# 가청성을 보장한다.
#
# 동작 (INTERVAL 주기, 4개 모두 idempotent — 이미 정상이면 PulseAudio 가 no-op 처리):
#   1) 카드 프로파일 복구  : off/auto_null 로 빠지면 analog-stereo 로 복귀 (무음 방지)
#   2) default sink 재지정
#   3) 음소거 해제
#   4) 음량 40% 고정
#
# 한계: PulseAudio/ALSA 에는 세션 소유자가 못 되돌리는 "볼륨 잠금" 플래그가 없다.
#       즉 변경을 "차단"하는 게 아니라 "원복"한다(최대 INTERVAL 만큼 지연).
#       sudo 로 이 워치독을 종료하거나 다른 sink 를 만들면 우회 가능.
#
# 값 근거: NUC13ANH-B = ALC256 analog 잭 1개뿐(HDMI 오디오 없음).
#   card 0 / profile output:analog-stereo / sink alsa_output.pci-0000_00_1f.3.analog-stereo

CARD=0
PROFILE="output:analog-stereo"
SINK="alsa_output.pci-0000_00_1f.3.analog-stereo"
VOLUME="40%"
INTERVAL=1

echo "[audio_watchdog] start: sink=$SINK vol=$VOLUME profile=$PROFILE interval=${INTERVAL}s"

while true; do
    pactl set-card-profile "$CARD" "$PROFILE" 2>/dev/null
    pactl set-default-sink "$SINK"            2>/dev/null
    pactl set-sink-mute    "$SINK" 0          2>/dev/null
    pactl set-sink-volume  "$SINK" "$VOLUME"  2>/dev/null
    sleep "$INTERVAL"
done
