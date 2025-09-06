#!/usr/bin/env python

import rospy
import numpy as np
import can
import cantools
import math
import rospkg

from std_msgs.msg import Float32, Float64
from mmc_msgs.msg import object_msg, object_array_msg

class CanListner(can.Listener):
    def __init__(self):
        rospy.init_node('front_radar_can_publisher')

        rospack = rospkg.RosPack()
        rospack.list()
        temp_path = rospack.get_path('can')

        dbpath = temp_path + '/dbc/180518_Radar_P_CAN_R01.dbc'
        print(dbpath)
        self.db = cantools.db.load_file(dbpath)

        # publisher
        self.sensor_pub = rospy.Publisher('/sensors/front_radar', object_array_msg, queue_size=1)

        self.sensor_msg = []
        self.sensor_polar = []

        self.time_old = rospy.Time.now().to_sec()


    def time_click(self,disp = ''):
        self.time = rospy.Time.now().to_sec()
        # print(disp+' time gap = '+str(self.time-self.time_old))
        self.time_old = self.time

    def sensor_publish(self):
        temp_time = rospy.Time.now()

        for i in range(len(self.sensor_msg)):
            temp_data = self.sensor_msg[i][1].data
            temp_idx = self.sensor_msg[i][0]
            temp_decoded = self.ESR[temp_idx].decode(data=temp_data)

            temp_status = temp_decoded['CAN_TX_TRACK_STATUS']
            temp_range = temp_decoded['CAN_TX_TRACK_RANGE']
            temp_range_rate = temp_decoded['CAN_TX_TRACK_RANGE_RATE']
            temp_angle = temp_decoded['CAN_TX_TRACK_ANGLE']

            self.sensor_polar.append([temp_idx, temp_status, temp_range, temp_range_rate, temp_angle])

        temp_publish = object_array_msg()
        for i in range(len(self.sensor_polar)):
            temp_polar = self.sensor_polar[i]
            temp_id = temp_polar[0]
            temp_range = temp_polar[2]
            temp_range_rate = temp_polar[3]
            temp_angle = temp_polar[4] * math.pi / 180.0

            temp_msg = object_msg()
            temp_msg.id = temp_id

            temp_msg.x = temp_range * math.cos(-temp_angle)
            temp_msg.y = temp_range * math.sin(-temp_angle)

            temp_msg.vx = temp_range_rate * math.cos(temp_angle)
            temp_msg.vy = 0

            temp_publish.data.append(temp_msg)

        temp_publish.time = temp_time

        self.sensor_pub.publish(temp_publish)

        # temp_decoded = self.ESR_power.decode(data=self.sensor_msg_power[0].data)
        # print(self.sensor_msg_power[-1])
        self.time_click(str(len(self.sensor_polar))+' ')
        # print(self.sensor_polar[-1])

        self.sensor_msg = []
        self.sensor_polar = []

    def on_message_received(self, msg):
        if msg.arbitration_id in self.ESR_id:
            i = self.ESR_id.index(msg.arbitration_id)
            self.sensor_msg.append([i, msg])
            if i == 31:
                self.sensor_publish()




    def set_db(self):
        self.ESR = []
        self.ESR_id = []
        for i in range(32):
            self.ESR.append(self.db.get_message_by_name('Track%02d'%(i+1)))
            self.ESR_id.append(self.ESR[i].frame_id)




if __name__ == '__main__':
    try:
        listener = CanListner()
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        listener.set_db()
        notifier = can.Notifier(bus, [listener])

        print("Initialized ...")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Chassis_Can_Pubisher() node.')
