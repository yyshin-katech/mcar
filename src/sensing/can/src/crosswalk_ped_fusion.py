#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

"""횡단보도 보행자 퓨전 노드 (CAN 무손상, additive).

자체(라이다) 횡단보도 점유(/katech_msg/crosswalk_occupancy)와 OBU(V2X) 보행자
(/obu/v2x_pedes_assistance)를, 현재 주행 링크(/localization/to_control_team 의
LINK_ID)로 선택된 활성 횡단보도(#1~#9)에 대해 퓨전하여
/katech_msg/crosswalk_ped_fusion 으로 10 Hz 발행한다.

횡단보도 <-> 링크/필드 매핑 (원천 crosswalk_position.md §134, 아래 CW_LINKS):
  active_crosswalk_id : ego LINK_ID 가 속한 CW_LINKS[N] 의 N (없으면 0). 링크셋 disjoint.
  own  : occupied_ids(검출 노드가 링크-active 크로스워크 중 보행자 있는 id) 에 active 포함 여부.
  obu  : #1 -> south_pedes, #2 -> east_pedes, #3~#9 -> 없음(단일 RSU 4방향 한계, own 만).

패턴: 콜백 -> 멤버 저장, 10 Hz rospy.Timer 에서 판정 후 발행 (CLAUDE.md 준용).
"""

import rospy

from katech_custom_msgs.msg import crosswalk_occupancy_msg, crosswalk_ped_fusion_msg
from v2x_msgs.msg import v2x_pedes_assist_msg
from mmc_msgs.msg import to_control_team_from_local_msg

# 크로스워크 -> 접근 LINK_ID 맵 (SSOT).
# 원천: claude_work_list/crosswalk_position.md §134 와 동기화할 것 (katech_ped_detector.py 와 동일 dict).
CW_LINKS = {
    1: {1239, 1238, 1242, 1241},
    2: {1205},
    3: {465, 467, 463},
    4: {417},
    5: {1029, 1025, 1027},
    6: {1092, 1094, 1090, 1091, 1093, 1089},
    7: {1370, 1368, 1372, 1369, 1367, 1371},
    8: {1326, 1325, 1330, 1331},
    9: {1257, 1258},
}


class CrosswalkPedFusion:
    def __init__(self):
        rospy.init_node('crosswalk_ped_fusion')

        # 최신 입력값 (콜백 -> 멤버 저장)
        self.occ1 = False        # crosswalk1_occupied (자체, #1)
        self.occ2 = False        # crosswalk2_occupied (자체, #2)
        self.occupied_ids = []   # 링크-active 이고 보행자 있는 크로스워크 id 목록(1~9)
        self.obu_south = False   # south_pedes (OBU, #1)
        self.obu_east = False    # east_pedes  (OBU, #2)
        self.link_id = 0         # 현재 주행 LINK_ID

        self.fusion_pub = rospy.Publisher('/katech_msg/crosswalk_ped_fusion',
                                          crosswalk_ped_fusion_msg, queue_size=10)

        rospy.Subscriber('/katech_msg/crosswalk_occupancy', crosswalk_occupancy_msg,
                         self._occupancy_cb, queue_size=10)
        rospy.Subscriber('/obu/v2x_pedes_assistance', v2x_pedes_assist_msg,
                         self._obu_cb, queue_size=10)
        rospy.Subscriber('/localization/to_control_team', to_control_team_from_local_msg,
                         self._link_cb, queue_size=10)

        # 10 Hz 타이머에서 퓨전 판정 후 발행 (입력율과 디커플)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._publish_fusion)

    # --- 콜백: 최신값만 저장 ---
    def _occupancy_cb(self, msg):
        self.occ1 = bool(msg.crosswalk1_occupied)
        self.occ2 = bool(msg.crosswalk2_occupied)
        self.occupied_ids = list(msg.occupied_ids)

    def _obu_cb(self, msg):
        self.obu_south = bool(msg.south_pedes)
        self.obu_east = bool(msg.east_pedes)

    def _link_cb(self, msg):
        self.link_id = int(msg.LINK_ID)

    # --- 타이머: 퓨전 판정 + 발행 ---
    def _publish_fusion(self, _event):
        # active 크로스워크: 현재 LINK_ID 를 포함하는 CW_LINKS 키(없으면 0). 링크셋 disjoint → 0/1개.
        active = next((N for N, ls in CW_LINKS.items() if self.link_id in ls), 0)

        # 자체(라이다): 링크-active 이고 해당 크로스워크가 점유 목록에 있으면 present.
        own_present = active != 0 and (active in self.occupied_ids)
        # OBU(V2X): 단일 RSU 4방향뿐 → #1=south_pedes, #2=east_pedes, 3~9 는 OBU 소스 없음.
        if active == 1:
            obu_present = self.obu_south
        elif active == 2:
            obu_present = self.obu_east
        else:
            obu_present = False

        pedestrian_present = own_present or obu_present
        if own_present and obu_present:
            source = 3
        elif obu_present:
            source = 2
        elif own_present:
            source = 1
        else:
            source = 0

        out = crosswalk_ped_fusion_msg()
        out.time = rospy.Time.now()
        out.active_crosswalk_id = active
        out.pedestrian_present = pedestrian_present
        out.source = source
        self.fusion_pub.publish(out)


def main():
    try:
        CrosswalkPedFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
