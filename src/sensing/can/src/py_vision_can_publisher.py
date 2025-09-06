#!/usr/bin/env python

import rospy
import can
import cantools
import numpy as np
from std_msgs.msg import Float32, Float64
from mmc_msgs.msg import object_msg, object_array_msg, lane_msg, lane_array_msg
import rospkg

LD_name = ['LD_Left_Lane_A','LD_Right_Lane_A','LD_Left_Lane_B','LD_Right_Lane_B']
LD_msg_name = ['Lh','Rh','Left_Lane','Right_Lane']

marker_converter = {"Dashed": 0, "Solid": 1, "Undecided": 2, "RoadEdge": 3, "DoubleLaneMark": 4, "Botts'Dots": 5, "Invalid": 6}

class CanListner(can.Listener):
    def __init__(self):
        rospy.init_node('vision_can_publisher')

        rospack = rospkg.RosPack()
        rospack.list()
        temp_path = rospack.get_path('can')

        dbpath = temp_path + '/dbc/180518_Camera_P_CAN_R01.dbc'
        # dbpath = "/home/jaeho/sensor_fusion_ws/DB/180518_Camera_P_CAN_R01.dbc"
        self.db = cantools.db.load_file(dbpath)
        # publisher
        self.sensor_pub = rospy.Publisher('/sensors/vision_pos', object_array_msg, queue_size=1)
        self.sensor_pub_lane = rospy.Publisher('/sensors/vision_lane', lane_array_msg, queue_size=1)
        self.sensor_msg_pos = []
        self.sensor_msg_lane = [[],[],[],[]]
        self.sensor_polar_pos = [[],[],[],[],[],[],[],[]]
        self.sensor_polar_lane = [[],[]]
        self.time_old = rospy.Time.now().to_sec()

        self.is_publishing = False
        self.lane_publishing_rate = 50


    def time_click(self,disp = ''):
        self.time = rospy.Time.now().to_sec()
        # print(float("{0:.4f}".format(self.time - self.time_old))*1000)
        self.time_old = self.time




    def sensor_publish_pos(self):
        temp_time = rospy.Time.now()
        # print(rospy.Time.now().to_sec())
        temp_decoded =[]
        # print('*** POSITION CAN PUBLISHED BELOW ***')

        for i in range(len(self.sensor_msg_pos)):
            temp_data = self.sensor_msg_pos[i][1].data
            temp_decoded.append(self.OBS[self.sensor_msg_pos[i][0]].decode(data=temp_data))

        for i in range(8):
            temp_id       = temp_decoded[i]['ObjectIdentifier_A_%01d'%(i+1)]
            temp_posX     = temp_decoded[i+8]['Range_B_%01d'%(i+1)]
            temp_posY     = temp_decoded[i+8]['ObjectPostionY_B_%01d'%(i+1)]
            temp_vx       = temp_decoded[i+8]['RangeRate_B_%01d'%(i+1)]
            temp_validity = temp_decoded[i+8]['ObjectValidity_B_%01d'%(i+1)]

            # [id, range, posX, posY]
            self.sensor_polar_pos[i] = [temp_id, temp_posX, temp_posY, temp_vx, temp_validity]
            # print(self.sensor_polar_pos[i])
        # print('')

        temp_publish = object_array_msg()

        for i in range(len(self.sensor_polar_pos)):
            temp_msg = object_msg()
            temp_msg.id          = self.sensor_polar_pos[i][0]
            temp_msg.x           = self.sensor_polar_pos[i][1]
            temp_msg.y           = self.sensor_polar_pos[i][2]
            temp_msg.vx          = self.sensor_polar_pos[i][3]
            temp_msg.valid_level = self.sensor_polar_pos[i][4]
            temp_publish.data.append(temp_msg)

        temp_publish.time = temp_time
        self.sensor_pub.publish(temp_publish)
        self.time_click(str(5)+' '+str(2))
        self.sensor_msg_pos = []
        self.sensor_polar_pos = [[],[],[],[],[],[],[],[]]



    def sensor_publish_lane(self):
        temp_time = rospy.Time.now()
        temp_decoded = []
        # print('************************ LANE CAN PUBLISHED BELOW ************************')
        self.sensor_polar_lane = [[],[]]
        for i in range(len(self.sensor_msg_lane)):
            temp_data = self.sensor_msg_lane[i][1].data
            temp_decoded.append(self.LD[self.sensor_msg_lane[i][0]].decode(data=temp_data))

        for i in range(2):
            temp_type       = temp_decoded[i]['%s_LaneMarkType'%LD_msg_name[i]]
            temp_quality    = temp_decoded[i]['%s_LaneMarkQuality'%LD_msg_name[i]]
            temp_position   = temp_decoded[i]['%s_LaneMarkPosition'%LD_msg_name[i]]
            temp_curv       = temp_decoded[i]['%s_LaneMarkModelA'%LD_msg_name[i]]
            temp_curv_deriv = temp_decoded[i+2]['%s_LaneMarkModelDerivA'%LD_msg_name[i]]
            temp_heading    = temp_decoded[i+2]['%s_LaneMarkHeadAngle'%LD_msg_name[i]]
            temp_view_range = temp_decoded[i+2]['%s_LaneMarkViewRange'%LD_msg_name[i]]

            # [name, a, b, c, d, view_range, quality]
            self.sensor_polar_lane[i] = [LD_msg_name[i+2], temp_curv_deriv, temp_curv, temp_heading, temp_position, temp_view_range, temp_quality, temp_type]
            # print(self.sensor_polar_lane[i])
        # print('')
        temp_decoded = []
        temp_publish_lane = lane_array_msg()

        for i, data in enumerate(self.sensor_polar_lane):
            temp_msg = lane_msg()
            temp_msg.Lane_Name  = data[0]
            temp_msg.a          = data[1] #curv_deriv
            temp_msg.b          = data[2] #curv
            temp_msg.c          = data[3] #heading
            temp_msg.d          = data[4] #position
            temp_msg.View_Range = data[5]
            temp_msg.Quality    = data[6]
            try:
                temp_msg.marker_type = marker_converter[data[7]]
            except:
                # print('Marker type error ',data[7])
                temp_msg.marker_type = 6
                
            temp_publish_lane.data.append(temp_msg)

        temp_publish_lane.time = temp_time
        self.sensor_pub_lane.publish(temp_publish_lane)
        # self.time_click(str(len(self.sensor_polar_lane))+' '+str(4))
        self.sensor_msg_lane = [[],[],[],[]]
        self.sensor_polar_lane = [[],[]]






    def on_message_received(self, msg):

        # Position
        if msg.arbitration_id in self.OBS_id:
            i = self.OBS_id.index(msg.arbitration_id)
            self.sensor_msg_pos.append([i, msg])
            if i==15:
                self.sensor_msg_pos = self.sensor_msg_pos[-16:]
                self.sensor_publish_pos()




        # Lane
        if msg.arbitration_id in self.LD_id:
            i = self.LD_id.index(msg.arbitration_id)
            self.sensor_msg_lane[i] = [i, msg]
            if not [] in self.sensor_msg_lane:
                self.sensor_publish_lane()




    def set_db(self):
        self.OBS = []
        self.OBS_id = []
        self.LD = []
        self.LD_id =[]

        # Position
        for i in range(8):
            self.OBS.append(self.db.get_message_by_name('Obstacle_Data_A_%1d'%(i+1)))
            self.OBS_id.append(self.OBS[i].frame_id)
        for i in range(8):
            self.OBS.append(self.db.get_message_by_name('Obstacle_Data_B_%1d'%(i+1)))
            self.OBS_id.append(self.OBS[i+8].frame_id)

        # Lane
        for i in range(len(LD_name)):
            self.LD.append(self.db.get_message_by_name('%s' %LD_name[i]))
            self.LD_id.append(self.LD[i].frame_id)





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
