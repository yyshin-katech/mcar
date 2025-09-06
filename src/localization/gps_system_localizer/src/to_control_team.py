#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
"""
to_control_team.py의 12월 데모를 위한 버전
"""

import rospy
import numpy as np
import scipy.io as sio

from scipy.spatial import cKDTree as KDTree
from scipy.interpolate import CubicSpline
import matplotlib.animation as animation
import time
import pyproj
# from scipy.spatial import distance

from mmc_msgs.msg import localization2D_msg, to_control_team_from_local_msg
from sensor_msgs.msg import NavSatFix
from utils import distance2curve

from utils_cython import find_closest, compute_current_lane, xy2frenet_with_closest_waypoint_loop, xy2frenet_with_closest_waypoint

MAPFILE_PATH = rospy.get_param('MAPFILE_PATH')
# GPS_EPSG = pyproj.Proj(init='epsg:4326') # WSG84
MAP_EPSG_NUMBER = 5186
# MAP_EPSG_NUMBER = int(rospy.get_param('EPSG'))
# MAP_EPSG = pyproj.Proj(init='epsg:%d' % MAP_EPSG_NUMBER) # map shapefile epsg

# lane_coeff_a_set = []
# lane_coeff_b_set = []
# lane_coeff_c_set = []
# lane_coeff_d_set = []

occupied_count = 0

class DistanceCalculator(object):

    def __init__(self):
        rospy.init_node('To_Control_Team_Node')
        self.init_variable()
        self.load_centerline_map()
        self.set_subscriber()
        self.set_publisher()

        rospy.spin()

    def init_variable(self):
        # 맵 centerlines
        self.road_1 = None
        self.road_2 = None
        self.road_3 = None
        self.road_4 = None
        self.road_5 = None
        self.road_6 = None

        self.map_loaded = False

        self.east = 0.0
        self.north = 0.0
        self.yaw = 0.0

        self.previous_index = np.zeros((4,), dtype=np.int32)

    def set_subscriber(self):
        # localization 정보 들어오면 바로 control team에 필요한 메세지 publish
        # callback 안에 publish 명령어까지 같이 들어있음
        rospy.Subscriber('/localization/pose_2d_gps', localization2D_msg, self.pose_2d_cb, queue_size=1)

    def set_publisher(self):
        self.to_control_team_pub = rospy.Publisher('/localization/to_control_team', to_control_team_from_local_msg, queue_size=1)

    def load_centerline_map(self):

        self.road_1 = sio.loadmat(MAPFILE_PATH + '/PG_link_1.mat')
        self.road_2 = sio.loadmat(MAPFILE_PATH + '/PG_link_2.mat')
        self.road_3 = sio.loadmat(MAPFILE_PATH + '/PG_link_3.mat')
        self.road_4 = sio.loadmat(MAPFILE_PATH + '/PG_link_4.mat')
        self.road_5 = sio.loadmat(MAPFILE_PATH + '/PG_link_5.mat')
        self.road_6 = sio.loadmat(MAPFILE_PATH + '/PG_link_6.mat')
        self.target_roads = [self.road_1, self.road_2,self.road_3,self.road_4,self.road_5,self.road_6]
        self.map_loaded = True

    def compute_my_lane_cy(self, e, n):
        ''' cython 버전 '''

        lane_names = ['road_1', 
                      'road_2',
                      'road_3', 
                      'road_4',
                      'road_5', 
                      'road_6' ,
                      'none']

        current_lane_id = -1
        current_lane_name = 'none'
        distance_to_entry_end = -1
        distance_to_exit_start = -1
        current_closest_waypoint_index = -1
        current_closest_waypoint_in_MATLAB = 0
        current_s = 0
        current_d = 0

        if self.map_loaded: # mat 파일 로드 

            # start = time.time()
            distances, indexs = compute_current_lane(self.target_roads, e, n)
            # print(distances)
            min_abs_d = 100.0
            for i, (dist, closest_waypoint) in enumerate(zip(distances, indexs)):
                if dist > 4.0:
                    pass
                else:
                    mapx = self.target_roads[i]['east'][0]
                    mapy = self.target_roads[i]['north'][0]
                    maps = self.target_roads[i]['station'][0]
                    # if i is 3:
                    
                    s, d = xy2frenet_with_closest_waypoint(e, n, closest_waypoint, mapx, mapy, maps)
                    
                    if abs(d) < min_abs_d:
                        current_lane_id = i
                        current_s = s
                        current_d = d
                        min_abs_d = abs(d)
                        current_closest_waypoint_index = closest_waypoint

            ''' 가장 최근에 지난 waypoint index 던져주기'''
            if current_closest_waypoint_index > 0:
                # matlab은 index가 1부터 시작하는 것에 조심하기
                maps = self.target_roads[current_lane_id]['station'][0]
                n_waypoints_in_map = len(maps)
                if maps[current_closest_waypoint_index] > current_s:
                    # 최근에 지난 waypoint index로 설정
                    current_closest_waypoint_index -= 1

                current_closest_waypoint_index = np.clip(current_closest_waypoint_index, 0, n_waypoints_in_map-1)
                current_closest_waypoint_in_MATLAB = current_closest_waypoint_index + 1

                current_lane_name = lane_names[current_lane_id]
                # print("current_lane: {}".format(current_lane_name))

        return current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB
    
    def lane_occupied_check(self,offset):

        if offset >= 0.95:

            value = 1
        
        else:
            
            value = 0
       
         
        return value
    

    def pose_2d_cb(self, msg):
        """
        localization 메세지를 받아서 control team에 필요한 메세지 publish
        """

        global occupied_count
        threshold_cnt = 20 

        ODD_id_list = [6]
        t0 = time.time()

        e = msg.east
        n = msg.north
        yaw = msg.yaw

        current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB = self.compute_my_lane_cy(e, n)


        ## KIAPI 에서 곡선부 돌때 lane id가 튀는 현상을 잡기 위한 임시방편 코드
        # if 1380 < current_s < 1490:
        #     current_lane_id = 0

        p = to_control_team_from_local_msg()

        p.time = msg.time
        p.look_at_signalGroupID = self.target_roads[current_lane_id]['look_at_signalGroupID'][0][0]
        p.look_at_IntersectionID = self.target_roads[current_lane_id]['look_at_IntersectionID'][0][0]
        p.NEXT_LINK_ID = self.target_roads[current_lane_id]['NEXT_LINK_ID'][0][0]
        p.LINK_ID = self.target_roads[current_lane_id]['LINK_ID'][0][0]
        p.have_to_LangeChange_left = self.target_roads[current_lane_id]['have_to_LangeChange_left'][0][0]
        p.have_to_LangeChange_right = self.target_roads[current_lane_id]['have_to_LangeChange_right'][0][0]
        p.left_LaneChange_avail = self.target_roads[current_lane_id]['left_LaneChange_avail'][0][0]
        p.right_LaneChange_avail = self.target_roads[current_lane_id]['right_LaneChange_avail'][0][0]
        p.Speed_Limit = self.target_roads[current_lane_id]['Speed_Limit'][0][0]
        # p.lane_coeffa = self.target_roads[current_lane_id]['a'][0][0]
        # p.lane_coeffb = self.target_roads[current_lane_id]['b'][0][0]
        # p.lane_coeffc = self.target_roads[current_lane_id]['c'][0][0]
        # p.lane_coeffd = self.target_roads[current_lane_id]['d'][0][0]
        p.distance_to_lane_end = self.target_roads[current_lane_id]['station'][0][-1] - current_s

        mapx_set = self.target_roads[current_lane_id]['east'][0]
        mapy_set = self.target_roads[current_lane_id]['north'][0]
        
        ## 자차량 ODD 상태 initilaize ##

        p.On_ODD = 0
        p.Road_State = 0
        p.distance_out_of_ODD = 200  # 현재 상태 모든 도로가 계속 ODD 영역으로 설정

        p.lane_id = current_lane_id + 1

        ## 지금 주행중인 링크랑 연결된 다음 링크가 ODD 이탈 영역 혹은 도로가 끊긴 경우 ##

        if p.NEXT_LINK_ID in ODD_id_list or p.NEXT_LINK_ID == 0:

            p.Road_State = 1  # 이탈 경고
            p.distance_out_of_ODD = p.distance_to_lane_end  # 이탈 영역까지 남은 거리


        ## 현재 영역에 주행할 경로가 없거나, ODD 이탈 영역에 들어올 때 계속 수동모드 플래그 송출 ##
        if p.lane_id == 0 or p.lane_id in ODD_id_list:
            p.left_LaneChange_avail = 0
            p.right_LaneChange_avail = 0
            p.look_at_signalGroupID = 0
            p.look_at_IntersectionID = 0
            p.have_to_LangeChange_left = 0
            p.have_to_LangeChange_right = 0
            p.Speed_Limit = 0
            p.LINK_ID = 0
            p.NEXT_LINK_ID = 0
            p.distance_to_lane_end = 0
            # p.lane_coeffa = 0
            # p.lane_coeffb = 0
            # p.lane_coeffc = 0
            # p.lane_coeffd = 0
            p.On_ODD = 1
            p.Road_State = 2
            p.distance_out_of_ODD = 0
            p.yaw_error_size = 100

        ## 현재 영역이 이탈 구역이 아닐 때 즉, 경로가 잡혔다면 ---> 제대로 그 경로안에 있고, 방향을 잘 보고 있는지 확인 ##

        else:

        ## 차선 걸쳐있을 때 마다 cnt 스코어 상승 ##

            if abs(current_d) >= 0.95:

                lane_occupied_cnt = self.lane_occupied_check(abs(current_d))

                occupied_count += lane_occupied_cnt

                if occupied_count  <= threshold_cnt and occupied_count > 0:
                  
                    p.On_ODD = 0
                    p.Road_State = 1

        ## 스코어 범위 넘으면 수동모드 전환 ##

                elif occupied_count > threshold_cnt:

                    p.On_ODD = 1
                    p.Road_State = 2
                    p.distance_out_of_ODD = 0

        ## 차선안에 들어오면 스코어 초기화  ##

            else:

                occupied_count = 0


            cs = CubicSpline(mapx_set, mapy_set, bc_type='natural')
            cs_derivative = cs.derivative()
            current_closest_waypoint_in_MATLAB = min(current_closest_waypoint_in_MATLAB, len(mapx_set) - 1)
            path_yaw = cs_derivative(mapx_set[current_closest_waypoint_in_MATLAB])
            yaw_error_size = abs(path_yaw - yaw)

            p.yaw_error_size = yaw_error_size

            ## 현재 주행할 경로쪽으로 방향이 제대로 맞으면 오토모드 송출 아니면, 수동모드 송출 ##

            if yaw_error_size >= np.deg2rad(45) :

                p.On_ODD = 1
                p.Road_State = 2
                p.distance_out_of_ODD = 0

        p.lane_name = current_lane_name
        p.host_east = e
        p.host_north = n
        p.host_yaw = yaw  # radian
        p.waypoint_index = current_closest_waypoint_in_MATLAB
        p.station = current_s
        p.lateral_offset = current_d

        
        # print("[control_pub] computation_time = %.4f (ms)" % ((time.time() - t0)*1000) )

        self.to_control_team_pub.publish(p)


if __name__ == "__main__":
    DistanceCalculator()











##############################################################################################################################


        # p.look_at_signalGroupID = self.target_roads[current_lane_id]['look_at_signalGroupID'][0][0]
        # p.look_at_IntersectionID = self.target_roads[current_lane_id]['look_at_IntersectionID'][0][0]
        # p.NEXT_LINK_ID = self.target_roads[current_lane_id]['NEXT_LINK_ID'][0][0]
        # p.LINK_ID = self.target_roads[current_lane_id]['LINK_ID'][0][0]
        # p.have_to_LangeChange_left = self.target_roads[current_lane_id]['have_to_LangeChange_left'][0][0]
        # p.have_to_LangeChange_right = self.target_roads[current_lane_id]['have_to_LangeChange_right'][0][0]
        # # p.left_LaneChange_avail = self.target_roads[current_lane_id]['left_LaneChange_avail'][0][0]
        # # p.right_LaneChange_avail = self.target_roads[current_lane_id]['right_LaneChange_avail'][0][0]
        # p.Speed_Limit = self.target_roads[current_lane_id]['Speed_Limit'][0][0]

        # lane_coeff_a_set = []
        # lane_coeff_b_set = []
        # lane_coeff_c_set = []
        # lane_coeff_d_set = []

        # mapx_set = self.target_roads[current_lane_id]['east'][0]
        # mapy_set = self.target_roads[current_lane_id]['north'][0]
        # maps_set = self.target_roads[current_lane_id]['station'][0]

        # cs = CubicSpline(mapx_set, mapy_set, bc_type='natural')
        # cs_derivative = cs.derivative()
        
        # for i in range(len(mapx_set) - 1):
            
        #     lane_coeff_a_set.append(cs.c[0, i])
        #     lane_coeff_b_set.append(cs.c[1, i])
        #     lane_coeff_c_set.append(cs.c[2, i])
        #     lane_coeff_d_set.append(cs.c[3, i])

        # x_fine = np.linspace(mapx_set[0], mapx_set[-1], len(maps_set))
        # y_fine = cs(x_fine)

        # p.lane_coeffa = lane_coeff_a_set[p.waypoint_index]
        # p.lane_coeffb = lane_coeff_b_set[p.waypoint_index]
        # p.lane_coeffc = lane_coeff_c_set[p.waypoint_index]
        # p.lane_coeffd = lane_coeff_d_set[p.waypoint_index]

#########################################################################################################################3

































