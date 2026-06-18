#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
"""
GPS(ublox) 전원 재인가 자동복구 워치독.

배경: GPS 전원이 끊기면 /dev/ttyACM0 가 사라지고, ublox 드라이버는 read 에러를
무한 재시도하며 좀비 상태로 스핀한다(async_worker.h readEnd). 프로세스가 죽지
않으니 launch 의 respawn="true" 도 발동하지 않아, 전원을 다시 넣어도 ttyACM0 를
재오픈하지 못한다 → 지금은 katech_test.launch 재실행이 필요.

이 노드는 다음만 한다:
  - 장치 파일이 존재(전원 복귀)하고
  - /ublox/navpvt 가 STALE(노드가 데이터 미발행)이면
  → `rosnode kill /ublox` 로 ublox 노드를 종료시켜 respawn 이 새 프로세스로
    재기동하게 만든다. 새 프로세스가 ttyACM0 를 다시 열어 복구된다.

장치 파일이 없을 때(전원 차단 중)는 아무것도 하지 않는다 — 복구 불가 상태이며
GPS 고장 표시는 그대로 유지(정상).
"""
import os
import subprocess

import rospy
from ublox_msgs.msg import NavPVT

DEVICE        = '/dev/ttyACM0'   # ublox config(zed_f9k.yaml)의 device 와 일치해야 함
NAVPVT_TOPIC  = '/ublox/navpvt'
UBLOX_NODE    = '/ublox'         # ublox_device.launch 의 node_name 기본값
STALE_SEC     = 3.0              # navpvt 미수신이 이 시간 넘으면 STALE (정상 ~20Hz)
COOLDOWN_SEC  = 15.0            # 재기동 후 재연결 대기 (respawn_delay 3s + 설정시간 여유)
CHECK_PERIOD  = 0.5             # 점검 주기 [s]


class UbloxWatchdog(object):
    def __init__(self):
        now = rospy.Time.now()
        self.last_navpvt = now      # 마지막 navpvt 수신 시각
        self.last_restart = now     # 마지막 재기동 시각 (초기 COOLDOWN 유예)
        rospy.Subscriber(NAVPVT_TOPIC, NavPVT, self._navpvt_cb, queue_size=1)
        rospy.Timer(rospy.Duration(CHECK_PERIOD), self._check_cb)
        rospy.loginfo("ublox watchdog 시작: device=%s topic=%s stale=%.1fs cooldown=%.1fs",
                      DEVICE, NAVPVT_TOPIC, STALE_SEC, COOLDOWN_SEC)

    def _navpvt_cb(self, _msg):
        self.last_navpvt = rospy.Time.now()

    def _check_cb(self, _evt):
        # 전원 차단 중(장치 없음)이면 복구 불가 — 대기
        if not os.path.exists(DEVICE):
            return

        now = rospy.Time.now()
        stale = (now - self.last_navpvt).to_sec() > STALE_SEC
        if not stale:
            return

        # 직전 재기동 후 재연결 유예 시간 동안은 다시 건드리지 않음
        if (now - self.last_restart).to_sec() < COOLDOWN_SEC:
            return

        rospy.logwarn("GPS 장치(%s) 복귀했으나 navpvt %.1fs STALE → %s 재기동(respawn 유도)",
                      DEVICE, (now - self.last_navpvt).to_sec(), UBLOX_NODE)
        self._restart_ublox()
        self.last_restart = now

    def _restart_ublox(self):
        try:
            subprocess.call(['rosnode', 'kill', UBLOX_NODE],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:  # noqa: BLE001 - 재기동 시도는 실패해도 다음 주기에 재시도
            rospy.logerr("rosnode kill %s 실패: %s", UBLOX_NODE, e)


if __name__ == '__main__':
    rospy.init_node('gps_ublox_watchdog')
    UbloxWatchdog()
    rospy.spin()
