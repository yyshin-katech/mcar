#!/usr/bin/env python

import rospy
import numpy as np
import can
import cantools
import math
from math import *

from std_msgs.msg import Float32, Float64
from mmc_msgs.msg import object_msg, object_array_msg

import rospkg

class CanListner(can.Listener):
    def __init__(self):
        print ' [*] DEFINE CAN LISTENER ...'

        rospy.init_node('corner_radar_left_can_publisher')

        print " [*] SET CAN DATABASE ..."

        rospack = rospkg.RosPack()
        rospack.list()
        temp_path = rospack.get_path('can')

        dbpath = temp_path + '/dbc/SRR2_PCAN_v19.dbc'

        self.db = cantools.db.load_file(dbpath)

        print " [*] DEFINE PUBLISHER  ... "
        # publisher
        self.sensor_pub = rospy.Publisher('/sensors/corner_radar_left', object_array_msg, queue_size=1)

        self.sensor_msg = []
        self.sensor_polar = []

        self.time_old = rospy.Time.now().to_sec()


    def time_click(self,disp = ''):
        self.time = rospy.Time.now().to_sec()
        print(disp+' time gap = '+str(self.time-self.time_old))
        self.time_old = self.time

    def sensor_publish(self):
        temp_time = rospy.Time.now()

        for i in range(len(self.sensor_msg)):
            temp_data = self.sensor_msg[i][1].data
            temp_idx = self.sensor_msg[i][0]
            temp_decoded = self.SODL[temp_idx].decode(data=temp_data)
            temp_status = temp_decoded['CAN_TX_DETECT_STATUS']
            temp_range = temp_decoded['CAN_TX_DETECT_RANGE']
            temp_range_rate = temp_decoded['CAN_TX_DETECT_RANGE_RATE']
            temp_angle = temp_decoded['CAN_TX_DETECT_ANGLE']
            temp_valid_level = temp_decoded['CAN_TX_DETECT_VALID_LEVEL']
            temp_amplitude = temp_decoded['CAN_TX_DETECT_AMPLITUDE']

            self.sensor_polar.append([temp_idx, temp_status, temp_range, temp_range_rate, temp_angle, temp_valid_level, temp_amplitude])


        temp_publish = object_array_msg()
        for i in range(len(self.sensor_polar)):
            temp_polar = self.sensor_polar[i]
            temp_id = temp_polar[0]
            temp_range = temp_polar[2]
            temp_range_rate = temp_polar[3]
            temp_angle = temp_polar[4]
            temp_valid_level = temp_polar[5]
            temp_amplitude = temp_polar[6]

            temp_msg = object_msg()
            temp_msg.id = temp_id

            left_angle = -90
            temp_msg.x = temp_range * math.cos(-1*radians(temp_angle + left_angle))
            temp_msg.y = temp_range * math.sin(-1*radians(temp_angle + left_angle))

            temp_msg.vx = temp_range_rate * math.cos(-1*radians(temp_angle + left_angle))
            temp_msg.vy = temp_range_rate * math.sin(-1*radians(temp_angle + left_angle))

            temp_msg.valid_level = temp_valid_level
            temp_msg.confidence = temp_amplitude

            temp_publish.data.append(temp_msg)

        temp_publish.time = temp_time

        self.sensor_pub.publish(temp_publish)

        self.time_click(str(len(self.sensor_polar))+' ')

        self.sensor_msg = []
        self.sensor_polar = []

    def on_message_received(self, msg):
        if msg.arbitration_id in self.SODL_id:
            i = self.SODL_id.index(msg.arbitration_id)
            self.sensor_msg.append([i, msg])

            if 63 == self.sensor_msg[-1][0]:
                self.sensor_publish()
                print('======================================================================')

    def set_db(self):
        self.SODL = []
        self.SODL_id = []
        for i in range(64):
            self.SODL.append(self.db.get_message_by_name('SODL_Track%02d'%(i+1)))
            self.SODL_id.append(self.SODL[i].frame_id)

if __name__ == '__main__':
    try:
        listener = CanListner()
        print " [*] DEFINE CAN LISTENER AND NOTIFIER ..."
        bus = can.interface.Bus(channel='can2', bustype='socketcan')
        listener.set_db()
        notifier = can.Notifier(bus, [listener])
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Chassis_Can_Pubisher() node.')
