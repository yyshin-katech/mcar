#!/usr/bin/env python

import numpy as np
import rospy
import can
import cantools
import math
import copy
from datetime import datetime
from std_msgs.msg import Float32, Float64, Header, String
from mmc_msgs.msg import to_control_team_from_local_msg, control_team_veh_sd_msg, control_team_veh_xy_msg


class CAN_sender():
    def __init__(self):

        init_time = rospy.Time.now().to_sec()

        self.check_rate = 20

        self.lane_info_time = copy.deepcopy(init_time)

        self.can_out = True

        self.alive_count=0

        self.time = -1

        dbpath = rospy.get_param('~dbpath')

        self.db = cantools.db.load_file(dbpath)

        if self.can_out is True:

            #need to be changed
            self.bus_send = can.interface.Bus(channel='can4', bustype='socketcan')
            print('\n')
            print('[*] LANE_CAN CONNECTED ')

        self.db_lane = []
        self.db_lane.append(self.db.get_message_by_name('LANE_INFO_A'))
        self.db_lane.append(self.db.get_message_by_name('LANE_INFO_B'))

        #db revise
        self.db_lane.append

        self.db_pos = []
        self.db_pos.append(self.db.get_message_by_name('CAR_EGO_A'))
        self.db_pos.append(self.db.get_message_by_name('CAR_EGO_B'))
        self.db_pos.append(self.db.get_message_by_name('CAR_EGO_SD'))

    def callback_lane_info(self, msg):

        self.alive_count +=1
        if self.alive_count > 255:
            self.alive_count = 0

        if self.time == -1:
            self.time = msg.time.to_nsec()/1000000


        new_time = msg.time.to_nsec()/1000000-self.time

        # print('self time is: ',self.time)
        # print('time is : ',msg.time.to_nsec()/1000000)
        # print('new time is: ',new_time)

        ## lane id / alive count sender
        can_data = self.db_lane[0].encode({'LANE_ID' : msg.lane_id, 'ALLIVE_CNT': self.alive_count, 'TIME':new_time})
        send_msg = can.Message(arbitration_id = self.db_lane[0].frame_id, data = can_data, dlc=8, extended_id = False)

        try:
            if self.can_out is True:
                self.bus_send.send(send_msg)

        except:
            pass

        ## distance to entry / distance to exit sender
        can_data = self.db_lane[1].encode({'M_ENTER': msg.distance_to_entry_end, 'M_EXIT': msg.distance_to_exit_start})
        send_msg = can.Message(arbitration_id = self.db_lane[1].frame_id, data = can_data, dlc=8, extended_id = False)


        try:
            if self.can_out is True:
                self.bus_send.send(send_msg)
        except:
            pass

        ## Global X / Y sender
        can_data = self.db_pos[0].encode({'X' : msg.host_east, 'Y' : msg.host_north})

        send_msg = can.Message(arbitration_id = self.db_pos[0].frame_id, data=can_data, dlc=8, extended_id=False)

        try:
            if self.can_out is True:
                self.bus_send.send(send_msg)
        except:
            pass

        ## Global yaw sender

        # print('way point is : ',msg.waypoint_index)
        can_data = self.db_pos[1].encode({'YAW' : msg.host_yaw, 'WAYPOINT' : msg.waypoint_index})
        send_msg = can.Message(arbitration_id = self.db_pos[1].frame_id, data=can_data, dlc=8, extended_id=False)

        try:
            if self.can_out is True:
                self.bus_send.send(send_msg)
        except:
            pass

        ## station / lateral_offset sender
        can_data = self.db_pos[2].encode({'EGO_S' : msg.station, 'EGO_D' : msg.lateral_offset})
        send_msg = can.Message(arbitration_id = self.db_pos[2].frame_id, data=can_data, dlc=8, extended_id=False)

        try:
            if self.can_out is True:
                self.bus_send.send(send_msg)
        except:
            pass


    def loop(self):
        rate = rospy.Rate(self.check_rate)

        while not rospy.is_shutdown():



            rate.sleep()

        pass


if __name__ == '__main__':
    try:
        rospy.init_node('can_sender')
        cp = CAN_sender()
        sub1 = rospy.Subscriber("/localization/to_control_team", to_control_team_from_local_msg, cp.callback_lane_info)

        print('[*] Lane Info Node is Initiallized...')

        cp.loop()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
