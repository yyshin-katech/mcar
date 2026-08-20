#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
/ublox/navpvt (ublox_msgs/NavPVT) → /ublox/inspva (novatel_gps_msgs/Inspva) 브리지.

siheung_v2x 의 BSM 송신 계열(bsm_tx_node/mqtt_bsm_tx_node/bsm_uploader_node)은
novatel Inspva 를 구독하도록 작성되어 있다. 이 브랜치의 GPS 는 ublox 뿐이라
소스를 고치는 대신 이 브리지로 메시지 타입만 변환한다 — 소비자 3곳은
`~inspva_topic` rosparam 오버라이드로 이 브리지의 발행 토픽을 구독한다.

⚠ 발행 토픽은 반드시 `/ublox/inspva` 다. `/sensors/gps/inspva` 로 발행하면
활성 노드 track_CAN_writer_no_grid 의 잠자는 콜백이 깨어나 제어로 가는
PCAN2 오브젝트 CAN 송신 내용이 바뀐다 (CAN 동결 원칙 위반).

필드 매핑 (BSM 계열이 실제 쓰는 6필드만):
  latitude/longitude : NavPVT lat/lon [1e-7 deg] → [deg]
  height             : NavPVT height [mm, 타원체고] → [m] (hMSL 아님)
  north/east_velocity: NavPVT velN/velE [mm/s] → [m/s]
  azimuth            : NavPVT heading(headMot) [1e-5 deg] → [deg]
                        단, gSpeed < 0.5 m/s 이면 ublox heading 이 발산하므로
                        마지막 유효 heading 을 유지한다.
"""
import rospy
from ublox_msgs.msg import NavPVT
from novatel_gps_msgs.msg import Inspva

INSPVA_TOPIC = '/ublox/inspva'
NAVPVT_TOPIC = '/ublox/navpvt'
HEADING_FREEZE_SPEED_MPS = 0.5  # gSpeed 가 이 미만이면 heading 갱신 보류


class NavpvtToInspva(object):
    def __init__(self):
        self._last_azimuth = 0.0
        self._pub = rospy.Publisher(INSPVA_TOPIC, Inspva, queue_size=10)
        rospy.Subscriber(NAVPVT_TOPIC, NavPVT, self._cb, queue_size=1)

    def _cb(self, msg):
        out = Inspva()
        out.header.stamp = rospy.Time.now()
        out.latitude = msg.lat * 1e-7
        out.longitude = msg.lon * 1e-7
        out.height = msg.height * 1e-3

        gspeed_mps = msg.gSpeed * 1e-3
        if gspeed_mps >= HEADING_FREEZE_SPEED_MPS:
            self._last_azimuth = msg.heading * 1e-5
        out.azimuth = self._last_azimuth

        out.north_velocity = msg.velN * 1e-3
        out.east_velocity = msg.velE * 1e-3

        self._pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('navpvt_to_inspva')
    NavpvtToInspva()
    rospy.spin()
