#!/usr/bin/env python

import rospy
import numpy as np
import can
import cantools
import math

from std_msgs.msg import Float32, Float64
from mmc_msgs.msg import object_msg, object_array_msg
import rospkg

class CanListner(can.Listener):
    def __init__(self):

        rospy.init_node('front_lidar_can_publisher')

        rospack = rospkg.RosPack()
        rospack.list()
        temp_path = rospack.get_path('can')

        dbpath = temp_path + '/dbc/Ibeo_Object_and_Vehicle_data_x400_v2.dbc'

        self.db = cantools.db.load_file(dbpath)

        # publisher
        self.sensor_pub = rospy.Publisher('/sensors/front_lidar', object_array_msg, queue_size=1)

        self.memory_tracking1 = []
        self.memory_boxdata = []
        self.memory_data_class = []

        self.time_old = rospy.Time.now().to_sec()


    def time_click(self,disp = ''):
        self.time = rospy.Time.now().to_sec()
        print(disp+' time gap = '+str(self.time-self.time_old))
        self.time_old = self.time

    def sensor_publish(self):
        temp_time = rospy.Time.now()
        temp_publish = object_array_msg()
        for i, temp_msg_tracking1 in enumerate(self.memory_tracking1):
            temp_msg = object_msg()
            temp_decoded_tracking1 = self.LUX[2].decode(data=temp_msg_tracking1.data)
            temp_msg.id = temp_decoded_tracking1['Object_ID']
            temp_msg.x = temp_decoded_tracking1['Position_X'] / 100.0 - 4.0
            temp_msg.y = temp_decoded_tracking1['Position_Y'] / 100.0
            temp_msg.vx = temp_decoded_tracking1['Velocity_X']
            temp_msg.vy = temp_decoded_tracking1['Velocity_Y']

            try:
                temp_decoded_boxdata = self.LUX[3].decode(data= self.memory_boxdata[i].data)
                if temp_decoded_tracking1['Object_ID'] == temp_decoded_boxdata['ObjectID']:
                    temp_msg.size_x = temp_decoded_boxdata['BoxSize_X_length'] / 100.0
                    temp_msg.size_y = temp_decoded_boxdata['BoxSize_Y_width'] / 100.0
                    temp_msg.orientation = temp_decoded_boxdata['BoxOrientation']
            except:
                print('CAN mismatch')

            try:
                temp_decoded_class = self.LUX[5].decode(data= self.memory_data_class[i].data)
                if temp_decoded_tracking1['Object_ID'] == temp_decoded_class['ObjectID']:
                    temp_msg.valid_level = temp_decoded_class['ObjectClass']
            except:
                print('CAN mismatch')
                    

            temp_publish.data.append(temp_msg)

        temp_publish.time = temp_time

        self.sensor_pub.publish(temp_publish)
        # if len(self.memory_tracking1) != len(self.memory_boxdata):
        #     print('haha')

        self.time_click(str(len(self.memory_tracking1))+str(len(self.memory_boxdata)))

        self.memory_tracking1 = []
        self.memory_boxdata = []
        self.memory_data_class = []


    def on_message_received(self, msg):
        if msg.arbitration_id in self.LUX_id:
            i = self.LUX_id.index(msg.arbitration_id)
            if i == 4:
                self.sensor_publish()
                # self.time_click('LUX_ObjectDataListTailer')
            elif i == 2:
                # self.time_click('LUXObjectDataTracking1')
                self.memory_tracking1.append(msg)

            elif i == 3:
                # self.time_click('LUXObjectDataBox')
                self.memory_boxdata.append(msg)

            elif i == 5:
                # self.time_click('LUXObjectDataBox')
                self.memory_data_class.append(msg)




    def set_db(self):
        self.LUX = []
        self.LUX_id = []

        self.LUX.append(self.db.get_message_by_name('LUX_ObjectDataListHeader'))
        self.LUX.append(self.db.get_message_by_name('LUXTime'))
        self.LUX.append(self.db.get_message_by_name('LUXObjectDataTracking1'))
        # self.LUX.append(self.db.get_message_by_name('LUXObjectDataTracking2'))
        
        self.LUX.append(self.db.get_message_by_name('LUXObjectDataBox'))
        # self.LUX.append(self.db.get_message_by_name('LUXObjectDataContourHeader'))
        # self.LUX.append(self.db.get_message_by_name('LUXObjectDataContourPoints'))
        self.LUX.append(self.db.get_message_by_name('LUX_ObjectDataListTailer'))
        self.LUX.append(self.db.get_message_by_name('LUXObjectDataClassANDBox'))

        for i in range(len(self.LUX)):
            self.LUX_id.append(self.LUX[i].frame_id)





if __name__ == '__main__':
    try:
        listener = CanListner()
        bus = can.interface.Bus(channel='can1', bustype='socketcan')
        listener.set_db()
        notifier = can.Notifier(bus, [listener])

        print("Initialized ...")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Chassis_Can_Pubisher() node.')
