#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

# TB_senario_map.shp 기반 to_control_team_demo
# shp 파일 원본 좌표계: EPSG:32652 (WGS84 UTM Zone 52N) -> EPSG:5179로 변환하여 사용
# 속성: ID, MaxSpeed, LaneNo, R_LinkID, L_LinkID,
#        FromNodeID, ToNodeID, LinkType, Length, N_LinkID, mc_SIG_GR 등

import rospy
import numpy as np
import os
import fiona
import pyproj
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree as KDTree
import time

from katech_diagnostic_msgs.msg import katech_diagnostic_msg
from mmc_msgs.msg import localization2D_msg, to_control_team_from_local_msg, chassis_msg
from sensor_msgs.msg import NavSatFix

from utils_cython import find_closest, compute_current_lane, xy2frenet_with_closest_waypoint_loop, xy2frenet_with_closest_waypoint

# shp 파일 경로
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SHP_FILE_PATH = os.path.join(SCRIPT_DIR, 'shp_map', 'TB_senario_map.shp')

ODD_CNT_THRESHOLD = 200
ODD_OCCUPIED_OFFSET_THRESHOLD = 0.95
ODD_YAW_ERR_THRESHOLD = np.deg2rad(5)

# SHP(EPSG:32652) -> EPSG:5179 좌표 변환기
_shp_to_5179 = pyproj.Transformer.from_crs('EPSG:32652', 'EPSG:5179', always_xy=True)


def shp_feature_to_road_dict(feature):
    """
    shp의 LineString feature를 .mat 호환 dict로 변환
    좌표를 EPSG:32652 -> EPSG:5179로 변환 후 저장
    """
    coords = np.array(feature['geometry']['coordinates'])
    east_32652 = coords[:, 0].astype(np.float64)
    north_32652 = coords[:, 1].astype(np.float64)

    # EPSG:32652 -> EPSG:5179 변환
    east, north = _shp_to_5179.transform(east_32652, north_32652)

    # station: 누적 거리 계산
    dx = np.diff(east)
    dy = np.diff(north)
    ds = np.sqrt(dx**2 + dy**2)
    station = np.zeros(len(east), dtype=np.float64)
    station[1:] = np.cumsum(ds)

    props = feature['properties']

    # compute_current_lane 등 cython 함수가 road['east'][0] 형태로 접근하므로
    # [0] 인덱싱이 가능하도록 2D 배열로 감싸기
    road = {
        'east': np.array([east]),
        'north': np.array([north]),
        'station': np.array([station]),
        # shp 속성 매핑
        'ID': props['ID'],                         # 링크 고유 ID (문자열)
        'MaxSpeed': props['MaxSpeed'] or 50,        # 제한속도
        'LaneNo': props['LaneNo'] or 1,             # 차선 수
        'R_LinkID': props['R_LinkID'],               # 우측 인접 링크 ID
        'L_LinkID': props['L_LinkID'],               # 좌측 인접 링크 ID
        'FromNodeID': props['FromNodeID'],           # 시작 노드 ID
        'ToNodeID': props['ToNodeID'],               # 종료 노드 ID
        'LinkType': props['LinkType'],               # 링크 타입
        'Length': props['Length'] or 0.0,            # 링크 길이
        'mc_SIG_GR': int(props['mc_SIG_GR']) if props.get('mc_SIG_GR') else 0,   # 신호등 그룹 ID
        'mc_INT_ID': int(props['mc_INT_ID']) if props.get('mc_INT_ID') else 0,    # 교차로 ID
    }
    return road


class DistanceCalculator(object):
    def __init__(self):
        rospy.init_node('To_Control_Team_Node')
        self.init_variable()
        self.load_centerline_map()
        self.set_subscriber()
        self.set_publisher()
        self.old_lane_id = 0
        self.old_waypoint_index = 0
        self.takeoverreq = 0
        self.LC_flag = 0

        rospy.spin()

    def init_variable(self):
        self.target_roads = []
        self.id_to_index = {}   # ID(문자열) -> list index 매핑
        self.num_lanes = 0
        self.map_loaded = False

        self.east = 0.0
        self.north = 0.0
        self.yaw = 0.0

        self.occupied_count = 0
        self.previous_index = np.zeros((4,), dtype=np.int32)

    def set_subscriber(self):
        rospy.Subscriber('/localization/pose_2d_gps', localization2D_msg, self.pose_2d_cb, queue_size=1)
        rospy.Subscriber('/diagnostic/system', katech_diagnostic_msg, self.diag_cb, queue_size=1)
        rospy.Subscriber('/sensors/chassis', chassis_msg, self.chasis_cb, queue_size=1)

    def set_publisher(self):
        self.to_control_team_pub = rospy.Publisher('/localization/to_control_team', to_control_team_from_local_msg, queue_size=1)

    def load_centerline_map(self):
        try:
            os.environ['SHAPE_RESTORE_SHX'] = 'YES'
            with fiona.open(SHP_FILE_PATH) as shp:
                for idx, feat in enumerate(shp):
                    road = shp_feature_to_road_dict(feat)
                    self.target_roads.append(road)
                    # ID -> index 매핑 (ToNodeID 기반 다음 링크 탐색용)
                    if road['ID']:
                        self.id_to_index[road['ID']] = idx

            # FromNodeID -> index 매핑 (다음 링크 찾기용)
            self.from_node_to_indices = {}
            for idx, road in enumerate(self.target_roads):
                fnode = road['FromNodeID']
                if fnode:
                    if fnode not in self.from_node_to_indices:
                        self.from_node_to_indices[fnode] = []
                    self.from_node_to_indices[fnode].append(idx)

            self.num_lanes = len(self.target_roads)
            self.map_loaded = True
            rospy.loginfo(f"SHP map loaded: {self.num_lanes} links from {SHP_FILE_PATH}")

        except Exception as e:
            rospy.logerr(f"Error loading SHP map: {e}")

    def find_next_link_index(self, current_idx):
        """현재 링크의 ToNodeID와 같은 FromNodeID를 가진 다음 링크 index를 반환"""
        to_node = self.target_roads[current_idx]['ToNodeID']
        if to_node and to_node in self.from_node_to_indices:
            candidates = self.from_node_to_indices[to_node]
            if len(candidates) > 0:
                return candidates[0]
        return -1

    def compute_my_lane_cy(self, e, n):
        """cython 버전 - shp 데이터 사용"""
        current_lane_id = -1
        current_lane_name = 'none'
        distance_to_entry_end = -1
        distance_to_exit_start = -1
        current_closest_waypoint_index = -1
        current_closest_waypoint_in_MATLAB = 0
        current_s = 0
        current_d = 0

        if self.map_loaded:
            distances, indexs = compute_current_lane(self.target_roads, e, n)

            min_abs_d = 100.0

            for i, (dist, closest_waypoint) in enumerate(zip(distances, indexs)):
                if dist > 3.0:
                    continue
                else:
                    mapx = self.target_roads[i]['east'][0]
                    mapy = self.target_roads[i]['north'][0]
                    maps = self.target_roads[i]['station'][0]

                    s, d = xy2frenet_with_closest_waypoint(e, n, closest_waypoint, mapx, mapy, maps)

                    if s >= maps[-1]:
                        continue
                    else:
                        if abs(d) < min_abs_d:
                            current_lane_id = i
                            current_s = s
                            current_d = d
                            min_abs_d = abs(d)
                            current_closest_waypoint_index = closest_waypoint

            if current_closest_waypoint_index > 0:
                maps = self.target_roads[current_lane_id]['station'][0]
                n_waypoints_in_map = len(maps)
                if maps[current_closest_waypoint_index] > current_s:
                    current_closest_waypoint_index -= 1

                current_closest_waypoint_index = np.clip(current_closest_waypoint_index, 0, n_waypoints_in_map - 1)
                current_closest_waypoint_in_MATLAB = current_closest_waypoint_index + 1

                current_lane_name = self.target_roads[current_lane_id]['ID'] or 'none'

        return current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB

    def lane_occupied_check(self, offset):
        if offset >= ODD_OCCUPIED_OFFSET_THRESHOLD:
            value = 1
        else:
            value = 0
        return value

    def chasis_cb(self, msg):
        self.LC_flag = msg.LC_flag

    def diag_cb(self, msg):
        statuses = [
            msg.gps_status,
            msg.adcu_status,
            msg.lidar_status,
            msg.radar_status,
            msg.v2x_status,
            msg.hmi_status,
            msg.vcu_status,
            msg.cam_status,
            msg.ipc_status,
        ]
        if any(s != 0 for s in statuses):
            self.takeoverreq = 1
        else:
            self.takeoverreq = 0

    def pose_2d_cb(self, msg):
        """
        localization 메세지를 받아서 control team에 필요한 메세지 publish
        """
        ODD_id_list = list(range(0, self.num_lanes))  # 유효한 lane index 범위
        t0 = time.time()

        e = msg.east
        n = msg.north
        yaw = msg.yaw

        current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB = self.compute_my_lane_cy(e, n)

        p = to_control_team_from_local_msg()

        p.time = msg.time

        # shp 속성에서 정보 추출
        if current_lane_id >= 0:
            road = self.target_roads[current_lane_id]
            p.LINK_ID = current_lane_id + 1
            p.Speed_Limit = road['MaxSpeed']
            p.distance_to_lane_end = road['station'][0][-1] - current_s

            # 다음 링크 탐색 (ToNodeID -> FromNodeID 매칭)
            next_idx = self.find_next_link_index(current_lane_id)
            p.NEXT_LINK_ID = next_idx + 1 if next_idx >= 0 else 0

            # 좌우 차선변경 가능 여부 (L_LinkID, R_LinkID 존재 여부로 판단)
            p.left_LaneChange_avail = 1 if road['L_LinkID'] else 0
            p.right_LaneChange_avail = 1 if road['R_LinkID'] else 0
            p.have_to_LangeChange_left = 0
            p.have_to_LangeChange_right = 0

            # 신호등 코드
            p.look_at_signalGroupID = road['mc_SIG_GR']
            p.look_at_IntersectionID = road['mc_INT_ID']
            p.is_stop_line = 0

            mapx_set = road['east'][0]
            mapy_set = road['north'][0]
        else:
            p.LINK_ID = 0
            p.NEXT_LINK_ID = 0
            p.Speed_Limit = 0
            p.distance_to_lane_end = 0
            p.left_LaneChange_avail = 0
            p.right_LaneChange_avail = 0
            p.have_to_LangeChange_left = 0
            p.have_to_LangeChange_right = 0
            p.look_at_signalGroupID = 0
            p.look_at_IntersectionID = 0
            p.is_stop_line = 0
            mapx_set = np.array([])
            mapy_set = np.array([])

        ## 자차량 ODD 상태 initialize ##
        p.On_ODD = 0
        p.Road_State = 0
        p.distance_out_of_ODD = 200

        p.lane_id = current_lane_id + 1

        ## 다음 링크가 없는 경우 이탈 경고 ##
        if p.NEXT_LINK_ID == 0 and current_lane_id >= 0:
            p.Road_State = 1
            p.distance_out_of_ODD = p.distance_to_lane_end

        ## 현재 영역에 주행할 경로가 없는 경우 ##
        if current_lane_id < 0:
            p.Speed_Limit = 0
            p.LINK_ID = 0
            p.NEXT_LINK_ID = 0
            p.distance_to_lane_end = 0
            p.On_ODD = 1
            p.Road_State = 2
            p.distance_out_of_ODD = 0
            p.yaw_error_size = 100

        ## 경로가 잡혔다면 -> 경로안에 있고 방향을 잘 보고 있는지 확인 ##
        else:
            ## while lane change,
            if self.LC_flag:
                self.occupied_count = 0
            ## 차선 걸쳐있을 때 마다 cnt 스코어 상승 ##
            elif abs(current_d) >= ODD_OCCUPIED_OFFSET_THRESHOLD:
                lane_occupied_cnt = self.lane_occupied_check(abs(current_d))
                self.occupied_count += lane_occupied_cnt

                if self.occupied_count <= ODD_CNT_THRESHOLD and self.occupied_count > 0:
                    p.On_ODD = 0
                    p.Road_State = 1

                ## 스코어 범위 넘으면 수동모드 전환 ##
                elif self.occupied_count > ODD_CNT_THRESHOLD:
                    p.On_ODD = 1
                    p.Road_State = 2
                    p.distance_out_of_ODD = 0

            ## 차선안에 들어오면 스코어 초기화 ##
            else:
                self.occupied_count = 0

            # Yaw 오차 계산 (CubicSpline 기반)
            distances = np.zeros(len(mapx_set))
            for i in range(1, len(mapx_set)):
                dx = mapx_set[i] - mapx_set[i - 1]
                dy = mapy_set[i] - mapy_set[i - 1]
                distances[i] = distances[i - 1] + np.sqrt(dx ** 2 + dy ** 2)

            # 중복 제거 및 strictly increasing 보장
            min_distance_increment = 1e-6
            unique_indices = [0]

            for i in range(1, len(distances)):
                if distances[i] > distances[unique_indices[-1]] + min_distance_increment:
                    unique_indices.append(i)

            # 최소 2개의 점이 필요
            if len(unique_indices) < 2:
                rospy.logwarn(f"Lane {current_lane_id}: Not enough unique points for interpolation")
                p.yaw_error_size = 100
                self.to_control_team_pub.publish(p)
                return

            distances_clean = distances[unique_indices]
            mapx_clean = mapx_set[unique_indices]
            mapy_clean = mapy_set[unique_indices]

            cs_x = CubicSpline(distances_clean, mapx_clean, bc_type='natural')
            cs_y = CubicSpline(distances_clean, mapy_clean, bc_type='natural')

            current_closest_waypoint_in_MATLAB = min(current_closest_waypoint_in_MATLAB, len(mapx_set) - 1)

            if current_closest_waypoint_in_MATLAB in unique_indices:
                clean_idx = unique_indices.index(current_closest_waypoint_in_MATLAB)
            else:
                clean_idx = np.searchsorted(unique_indices, current_closest_waypoint_in_MATLAB)
                clean_idx = min(clean_idx, len(unique_indices) - 1)

            current_distance = distances_clean[clean_idx]

            dx_ds = cs_x.derivative()(current_distance)
            dy_ds = cs_y.derivative()(current_distance)

            path_yaw = np.arctan2(dy_ds, dx_ds)

            yaw_error = path_yaw - yaw
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
            yaw_error_size = abs(yaw_error)

            p.yaw_error_size = yaw_error_size

            if p.On_ODD == 0 and p.Road_State == 0:
                if yaw_error_size < ODD_YAW_ERR_THRESHOLD:
                    p.Wrong_Way_Warn = 0
                    p.On_ODD = 0
                    p.Road_State = 0
                elif yaw_error_size > np.deg2rad(135):  # 반대방향
                    p.On_ODD = 1
                    p.Road_State = 2
                    p.Wrong_Way_Warn = 1
                    p.distance_out_of_ODD = 0
                else:  # 단순 이탈
                    p.On_ODD = 1
                    p.Road_State = 2
                    p.Wrong_Way_Warn = 0
                    p.distance_out_of_ODD = 0

        p.lane_name = current_lane_name
        p.host_east = e
        p.host_north = n
        p.host_yaw = yaw  # radian
        p.waypoint_index = current_closest_waypoint_in_MATLAB
        p.station = current_s
        p.lateral_offset = current_d

        self.to_control_team_pub.publish(p)

        self.old_lane_id = p.LINK_ID
        self.old_waypoint_index = p.waypoint_index


if __name__ == "__main__":
    DistanceCalculator()
