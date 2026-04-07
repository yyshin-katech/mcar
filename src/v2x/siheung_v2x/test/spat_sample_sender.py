#!/usr/bin/env python3
"""
SPaT decode sample 기반 UDP 테스트 전송기

SPAT decode sample.txt의 원본 데이터를 OBU 5바이트 헤더 + J2735 UPER 형태로 변환하여
UDP 9999 포트로 전송합니다.

사용법:
  python3 spat_sample_sender.py              # 1회 전송
  python3 spat_sample_sender.py --loop 10    # 10Hz 반복 전송
  python3 spat_sample_sender.py --loop 10 --port 9999
"""

import socket
import time
import argparse

# ============================================================
# 샘플 데이터 (SPAT decode sample.txt 기반)
# ============================================================

# 원본 265바이트: WSM 헤더(16) + J2735 MessageFrame UPER(249)
ORIGINAL_WSM_PACKET = bytes([
    # WSM 헤더 (16 bytes) - OBU 포맷에서는 사용하지 않음
    0x04, 0x00, 0xFF, 0x11, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x40, 0x85, 0x00, 0x00, 0x00, 0xF9,
    # J2735 MessageFrame UPER (249 bytes)
    # messageId=0x13(19)=SPaT, intersectionId=2, region=1
    # 12 movements: signalGroup 50/60/70/80, movementName STR/LEFT/PED
    0x00, 0x13, 0x80, 0xF5, 0x60, 0xF0, 0x36, 0x1A,
    0x2E, 0xEC, 0xEC, 0x18, 0x30, 0x64, 0x07, 0x03,
    0x45, 0xDD, 0x9D, 0x83, 0x06, 0x0C, 0xA0, 0x00,
    0x20, 0x00, 0x43, 0x00, 0x00, 0x00, 0x00, 0x01,
    0xB4, 0xEC, 0x2D, 0x02, 0xA7, 0x52, 0x92, 0x30,
    0x21, 0xFC, 0x00, 0x00, 0x10, 0xB8, 0x13, 0x38,
    0x00, 0x00, 0x00, 0x00, 0x10, 0x39, 0x91, 0x63,
    0x54, 0x46, 0x04, 0x3F, 0x80, 0x00, 0x02, 0x17,
    0x02, 0x67, 0x00, 0x00, 0x00, 0x00, 0x02, 0x05,
    0x42, 0x2C, 0x44, 0x60, 0x43, 0xF8, 0x00, 0x00,
    0x0D, 0x70, 0x23, 0x50, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x54, 0xEA, 0x52, 0x50, 0x04, 0x6F, 0x80,
    0x00, 0x00, 0x5A, 0x00, 0xAA, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x07, 0x32, 0x2C, 0x6A, 0x8A, 0x00,
    0x87, 0xF0, 0x00, 0x00, 0x0E, 0x60, 0x52, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x40, 0xA8, 0x45, 0x88,
    0xA0, 0x08, 0x7F, 0x00, 0x00, 0x01, 0xAE, 0x04,
    0x6A, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0A, 0x9D,
    0x4A, 0x46, 0x40, 0x87, 0xF0, 0x00, 0x00, 0x32,
    0x00, 0x4C, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x40,
    0xE6, 0x45, 0x8D, 0x50, 0xC8, 0x10, 0xFE, 0x00,
    0x00, 0x06, 0x40, 0x09, 0x9C, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x15, 0x08, 0xB1, 0x0C, 0x81, 0x0F,
    0xE0, 0x00, 0x00, 0x35, 0xC0, 0x8D, 0x40, 0x00,
    0x00, 0x00, 0x00, 0x81, 0x53, 0xA9, 0x48, 0xF0,
    0x11, 0xBE, 0x00, 0x00, 0x01, 0x68, 0x02, 0xA8,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x1C, 0xC8, 0xB1,
    0xAA, 0x1E, 0x02, 0x1F, 0xC0, 0x00, 0x00, 0x39,
    0x81, 0x4A, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
    0xA1, 0x16, 0x21, 0xE0, 0x21, 0xFC, 0x00, 0x00,
    0x06, 0xB8, 0x11, 0xA8, 0x00, 0x00, 0x00, 0x00,
    0x00,
])

# J2735 MessageFrame UPER 부분만 (WSM 헤더 제외)
J2735_MESSAGEFRAME = ORIGINAL_WSM_PACKET[16:]

# OBU 5바이트 헤더 + J2735 MessageFrame
def build_obu_spat_packet(seq_no=0):
    """isMsgFrame=0 (MessageFrame) SPaT 패킷"""
    obu_header = bytes([
        0x00,       # Frame Type: OBU→PC
        seq_no & 0xFF,  # Seq No
        0x00,       # msg source: from RSU
        0x00,       # isMsgFrame: 0 = MessageFrame
        0x00,       # Reserved
    ])
    return obu_header + J2735_MESSAGEFRAME


# ============================================================
# 기대되는 디코딩 결과 (검증용)
# ============================================================
EXPECTED = """
=== SPaT 디코딩 기대값 ===
MessageFrame messageId: 19 (0x13) = SPaT
SPAT timeStamp: 61494
SPAT name: Eng0002

IntersectionState:
  name: Eng0002
  id: region=1, id=2
  revision: 12
  status: 0x0000
  moy: 0
  timeStamp(DSecond): 27963

MovementStates (12개):
  SigGrp=70  STR   stop-And-Remain     minEnd=1070  maxEnd=1230
  SigGrp=70  LEFT  stop-And-Remain     minEnd=1070  maxEnd=1230
  SigGrp=70  PED   stop-And-Remain     minEnd=430   maxEnd=1130
  SigGrp=80  STR   protected-Movement  minEnd=180   maxEnd=340
  SigGrp=80  LEFT  stop-And-Remain     minEnd=230   maxEnd=1320
  SigGrp=80  PED   stop-And-Remain     minEnd=430   maxEnd=1130
  SigGrp=50  STR   stop-And-Remain     minEnd=800   maxEnd=1230
  SigGrp=50  LEFT  stop-And-Remain     minEnd=800   maxEnd=1230
  SigGrp=50  PED   stop-And-Remain     minEnd=430   maxEnd=1130
  SigGrp=60  STR   protected-Movement  minEnd=180   maxEnd=340
  SigGrp=60  LEFT  stop-And-Remain     minEnd=230   maxEnd=1320
  SigGrp=60  PED   stop-And-Remain     minEnd=430   maxEnd=1130
"""


def main():
    parser = argparse.ArgumentParser(description='SPaT sample UDP sender')
    parser.add_argument('--port', type=int, default=9999, help='UDP port (default: 9999)')
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Target host')
    parser.add_argument('--loop', type=float, default=0, help='반복 전송 Hz (0=1회)')
    parser.add_argument('--info', action='store_true', help='기대 디코딩 결과 출력')
    args = parser.parse_args()

    if args.info:
        print(EXPECTED)
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    seq = 0
    packet = build_obu_spat_packet(seq)

    print(f"Target: {args.host}:{args.port}")
    print(f"Packet size: {len(packet)} bytes (OBU 5 + J2735 {len(J2735_MESSAGEFRAME)})")
    print(f"Original WSM: {len(ORIGINAL_WSM_PACKET)} bytes")
    print()

    if args.loop > 0:
        interval = 1.0 / args.loop
        print(f"반복 전송: {args.loop} Hz (Ctrl+C로 중지)")
        try:
            while True:
                packet = build_obu_spat_packet(seq)
                sock.sendto(packet, (args.host, args.port))
                print(f"\r  seq={seq:3d}, sent {len(packet)} bytes", end='', flush=True)
                seq = (seq + 1) % 256
                time.sleep(interval)
        except KeyboardInterrupt:
            print(f"\n중지. 총 {seq}개 전송")
    else:
        sock.sendto(packet, (args.host, args.port))
        print(f"전송 완료: {len(packet)} bytes")

    sock.close()


if __name__ == '__main__':
    main()
