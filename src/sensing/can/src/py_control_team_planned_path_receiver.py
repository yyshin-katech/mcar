#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import rospy
import can
import cantools
import math
import copy
from datetime import datetime
from std_msgs.msg import Float32, Header, String
from mmc_msgs.msg import control_team_veh_array_msg, control_team_veh_sd_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import rospkg

NUMBER_OF_CAN_MESSAGES_TO_PARSE = 7

class CAN_receiver(can.Listener):
    def __init__(self):

        rospy.init_node('planned_path_can_receiver')
        # dbpath = rospy.get_param('~dbpath')

        # dbpath = '/home/gihoon/work/mcar_ws/src/control_response/Lane_info.dbc'
        self.db = cantools.db.load_file('/home/eq900/mcar_ws/src/control_response/sensor_fusion_result_v2.dbc')

        self.planned_path_pub = rospy.Publisher('/from_control_team/planned_path', Marker, queue_size=1)

        self.can_msg = [[] for i in range(NUMBER_OF_CAN_MESSAGES_TO_PARSE)]

    def publish_can_msg(self):
        ''' can msg를 파싱해서 rostopic으로 publish '''
        current_time = rospy.Time.now()
        decoded = []
        decoded_idx_list = []

        ''' can 메세지 파싱 '''
        n_can_msg = len(self.can_msg)
        for i in range(n_can_msg):
            data = self.can_msg[i][1].data
            idx = self.can_msg[i][0]
            decoded.append(
                self.db_list[idx].decode(data=data)
            )
            decoded_idx_list.append(idx)


        ''' 파싱된 메세지를 rostopic으로 변환 '''
        planned_x = dict()
        planned_y = dict()

        # x01 = [1,2,3,4,5,6]
        # x02 = [7,8,9,10,11,12]
        # x03 = [13,14,15,16,17,18]
        # x04 = [19,20]
        # y01 = [1,2,3,4,5,6,7]
        # y02 = [8,9,10,11,12,13,14]
        # y03 = [15,16,17,18,19,20]
        #
        # xs = [x01, x02, x03, x04]
        # ys = [y01, y02, y03]

        for i in range(n_can_msg):
            ''' 점으로 표시할거라서 일단 순서 상관없이 파싱 '''
            idx = decoded_idx_list[i]
            data = decoded[i]
            for key, val in data.items():
                if idx < 4:
                    ''' x '''
                    planned_x[key] = val
                else:
                    ''' y '''
                    planned_y[key] = val

        planned_x_list = []
        planned_y_list = []
        for i in range(1, 21):
            xkey = 'EGO_TRAJ_X_%d' % (i)
            ykey = 'EGO_TRAJ_Y_%d' % (i)
            planned_x_list.append(planned_x[xkey])
            planned_y_list.append(planned_y[ykey])

        # planned_x_list = np.arange(0,20)

        m = Marker()
        m.header.stamp = current_time
        m.header.frame_id = 'base_link'
        m.id = 1
        m.type = m.POINTS
        m.action = m.ADD
        m.scale.x = 0.6
        m.scale.y = 0.6
        m.scale.z = 0.6
        m.color.r = 123/255.0
        m.color.g = 239/255.0
        m.color.b = 178/255.0
        m.color.a = 0.9
        m.pose.orientation.w = 1.0
        m.points = []
        for i, (x,y) in enumerate(zip(planned_x_list, planned_y_list)):
            p = Point()
            p.x = x
            p.y = y
            m.points.append(p)
        self.planned_path_pub.publish(m)

    def on_message_received(self, msg):
        if msg.arbitration_id in self.can_msg_id_list:
            ''' 관심있는 can msg가 들어오면 그 메세지를 저장 '''
            i = self.can_msg_id_list.index(msg.arbitration_id)
            self.can_msg[i] = [i, msg]
            if not [] in self.can_msg:
                self.publish_can_msg()

    def set_db(self):
        ''' CAN DB에서 어떤 메세지를 파싱할 것인지 '''
        self.db_list = [
            self.db.get_message_by_name('EGO_TRAJ_PRED_X01'),
            self.db.get_message_by_name('EGO_TRAJ_PRED_X02'),
            self.db.get_message_by_name('EGO_TRAJ_PRED_X03'),
            self.db.get_message_by_name('EGO_TRAJ_PRED_X04'),
            self.db.get_message_by_name('EGO_TRAJ_PRED_Y01'),
            self.db.get_message_by_name('EGO_TRAJ_PRED_Y02'),
            self.db.get_message_by_name('EGO_TRAJ_PRED_Y03')
        ]

        ''' 파싱할 메세지의 can id 리스트 '''
        self.can_msg_id_list = []
        for i in range(len(self.db_list)):
            self.can_msg_id_list.append(self.db_list[i].frame_id)

if __name__ == '__main__':

    try:
        listener = CAN_receiver()
        bus = can.interface.Bus(channel='can4', bustype='socketcan')
        listener.set_db()

        notifier = can.Notifier(bus,[listener])
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
