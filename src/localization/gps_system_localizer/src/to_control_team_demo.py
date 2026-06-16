#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

# turnnel 입구 gps fake 지점
# 링크 56번 남은거리 60미터
# turnel 출구 gps fake off 지점
# 링크 57번 남은거리 10미터
# 어린이 보호구역 진입전 신호등 레드 전체 65초
import rospy
import numpy as np
import scipy.io as sio
from scipy.interpolate import CubicSpline

from scipy.spatial import cKDTree as KDTree
import time
import pyproj

from katech_diagnostic_msgs.msg import katech_diagnostic_msg
from mmc_msgs.msg import localization2D_msg, to_control_team_from_local_msg, chassis_msg
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT
from katech_custom_msgs.msg import ioniq5_ad_can_msg
from utils import distance2curve

from utils_cython import find_closest, compute_current_lane, xy2frenet_with_closest_waypoint_loop, xy2frenet_with_closest_waypoint

# MAPFILE_PATH = '/home/katech/mcar_v10/src/localization/gps_system_localizer/mapfiles/K_CITY_20250630'
# MAPFILE_PATH = '/home/ads/mcar_v10/src/localization/gps_system_localizer/mapfiles/KATECH_0508'
MAPFILE_PATH = rospy.get_param('MAPFILE_PATH')
# MAPFILE_PATH = '/home/katech/mcar_v13/src/localization/gps_system_localizer/mapfiles/K_CITY_20251106'
MIN_LANE_ID = 1
MAX_LANE_ID = 85

ODD_CNT_THRESHOLD = 200
ODD_OCCUPIED_OFFSET_THRESHOLD = 2.0
ODD_YAW_ERR_THRESHOLD = np.deg2rad(5)

# 안전 취약시간대 (KST, [시작,끝) — 시작 포함, 끝 미포함)
VULNERABLE_WINDOWS = [(7*3600 + 30*60, 9*3600),      # 07:30 ~ 09:00
                      (13*3600,        17*3600)]     # 13:00 ~ 17:00
SCHOOL_ZONE_LINK_ID = 61
SCHOOL_ZONE_SPEED_LIMIT = 20  # km/h

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
        self.ad_mode = 0  # 0=Manual, 1=Autonomous

        rospy.spin()

    def init_variable(self):
        # 맵 centerlines
        # self.road_1 = None
        # self.road_2 = None
        # self.road_3 = None
        # self.road_4 = None
        # self.road_5 = None
        # self.road_6 = None
        for i in range(MIN_LANE_ID, MAX_LANE_ID+1):
            setattr(self, f'road_{i}', None)

        self.map_loaded = False

        self.east = 0.0
        self.north = 0.0
        self.yaw = 0.0

        self.occupied_count = 0
        self.previous_index = np.zeros((4,), dtype=np.int32)

        # ─ effective time (KST) 상태 ─
        # navpvt
        self.gps_kst_seconds = None      # 자정기준 초 (0~86399), None=미수신/무효
        self.gps_time_valid = False
        # TimeRangeSetting override (free-run)
        self.last_tr_hour = 0
        self.last_tr_minute = 0
        self.override_active = False
        self.override_anchor_seconds = None   # 자정기준 초
        self.override_anchor_walltime = None  # rospy.Time

    def set_subscriber(self):
        # localization 정보 들어오면 바로 control team에 필요한 메세지 publish
        # callback 안에 publish 명령어까지 같이 들어있음
        rospy.Subscriber('/localization/pose_2d_gps', localization2D_msg, self.pose_2d_cb, queue_size=1)
        rospy.Subscriber('/diagnostic/system', katech_diagnostic_msg, self.diag_cb, queue_size=1)
        rospy.Subscriber('/sensors/chassis', chassis_msg, self.chasis_cb, queue_size=1)
        rospy.Subscriber('/ublox/navpvt', NavPVT, self.navpvt_cb, queue_size=1)
        rospy.Subscriber('/sensors/ioniq5_ad_can', ioniq5_ad_can_msg, self.time_range_cb, queue_size=1)

    def set_publisher(self):
        self.to_control_team_pub = rospy.Publisher('/localization/to_control_team', to_control_team_from_local_msg, queue_size=1)

    def load_centerline_map(self):
        try:
            for i in range(MIN_LANE_ID, MAX_LANE_ID+1):
                mat_file_path = f'{MAPFILE_PATH}/link_{i}.mat'
                setattr(self, f'road_{i}', sio.loadmat(mat_file_path))

            # self.target_roads = [self.road_1, self.road_2,self.road_3,self.road_4,self.road_5,self.road_6]
            self.target_roads = [getattr(self, f'road_{i}') for i in range(MIN_LANE_ID, MAX_LANE_ID+1)]
            
            self.map_loaded = True

        except Execption as e:
            rospy.logerr(f"Error loading centerline amp: {e}")

    def compute_my_lane_cy(self, e, n):
        ''' cython 버전 '''
        lane_names = [f'road_{i}' for i in range(MIN_LANE_ID, MAX_LANE_ID+1)]
        lane_names.append('none')
        
        current_lane_id = -1
        current_lane_name = 'none'
        distance_to_entry_end = -1
        distance_to_exit_start = -1
        current_closest_waypoint_index = -1
        current_closest_waypoint_in_MATLAB = 0
        current_s = 0
        current_d = 0

        if self.map_loaded: # mat 파일 로드 
            distances, indexs = compute_current_lane(self.target_roads, e, n)
            
            min_abs_d = 100.0
            # min_abs_d = 1.5
            
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

                    # if abs(d) < min_abs_d:
                    #     current_lane_id = i
                    #     current_s = s
                    #     current_d = d
                    #     min_abs_d = abs(d)
                    #     current_closest_waypoint_index = closest_waypoint
                            
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
        
        return current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB
    
    def lane_occupied_check(self,offset):
        if offset >= ODD_OCCUPIED_OFFSET_THRESHOLD:
            value = 1
        else:
            value = 0
         
        return value
    
    def chasis_cb(self, msg):
        self.LC_flag = msg.LC_flag
        self.ad_mode = msg.vcu_ADMDStatus  # 0=Manual, 1=Autonomous

    def navpvt_cb(self, msg):
        # NavPVT: UTC 분해값. VALID_DATE|VALID_TIME 확인 후 KST(UTC+9) 자정기준 초로 보관.
        valid = bool(msg.valid & (NavPVT.VALID_DATE | NavPVT.VALID_TIME))
        if not valid:
            self.gps_time_valid = False
            return
        utc_sec = msg.hour * 3600 + msg.min * 60 + msg.sec
        self.gps_kst_seconds = (utc_sec + 9 * 3600) % 86400  # KST = UTC+9
        self.gps_time_valid = True

    def time_range_cb(self, msg):
        # TimeRangeSetting (ioniq5_ad_can) → free-run override 시계 anchor.
        # 0/0 → override 비활성(GPS 실시간). 비0 → 전이 시 anchor 갱신.
        cur = (int(msg.time_range_hour), int(msg.time_range_minute))
        if cur == (0, 0):
            self.override_active = False
        else:
            if cur != (self.last_tr_hour, self.last_tr_minute):
                # 전이: anchor 재설정 (이 시:분으로 점프 후 실경과만큼 흐름)
                self.override_anchor_seconds = cur[0] * 3600 + cur[1] * 60
                self.override_anchor_walltime = rospy.Time.now()
            self.override_active = True
        self.last_tr_hour, self.last_tr_minute = cur

    def compute_effective_seconds(self):
        # 반환: (eff_seconds 또는 None, time_source)
        #   override 활성 → free-run, time_source=1
        #   아니면 GPS 실시간(유효 시) → time_source=0
        #   GPS 무효/미수신이고 override 도 없으면 → (None, 0)  [안전: 미적용]
        if self.override_active and self.override_anchor_seconds is not None:
            elapsed = (rospy.Time.now() - self.override_anchor_walltime).to_sec()
            eff = (self.override_anchor_seconds + int(elapsed)) % 86400
            return eff, 1
        if self.gps_time_valid and self.gps_kst_seconds is not None:
            return self.gps_kst_seconds, 0
        return None, 0

    def in_vulnerable_window(self, eff_seconds):
        if eff_seconds is None:
            return False  # 시각 미확정 → 취약시간대 미적용(안전)
        for start, end in VULNERABLE_WINDOWS:
            if start <= eff_seconds < end:   # [시작, 끝)
                return True
        return False

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
        # if (msg.gps_status != 0 or
        #     msg.adcu_status != 0 or
        #     msg.lidar_status != 0 or
        #     msg.radar_status != 0 or
        #     msg.v2x_status != 0 or
        #     msg.hmi_status != 0 or
        #     msg.vcu_status != 0 or
        #     msg.cam_status != 0 or
        #     msg.ipc_status != 0):
        #     # rospy.loginfo("⚠ 하나라도 0이 아님! (문제 발생 가능)")
        #     p.Take_Over_Request = 1
        # else:
        #     # rospy.loginfo("✅ 전부 0임 (정상 상태)")
        #     p.Take_Over_Request = 0

    def pose_2d_cb(self, msg):
        """
        localization 메세지를 받아서 control team에 필요한 메세지 publish
        """
        ODD_id_list = list(range(1, MAX_LANE_ID +1)) # [MAX_LANE_ID+1]
        t0 = time.time()

        e = msg.east
        n = msg.north
        # e = 935637.84+2.6+0.22
        # n = 1916057.58
        yaw = msg.yaw
        # yaw = 1.2
        alt = msg.altitude

        current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB = self.compute_my_lane_cy(e, n)

        p = to_control_team_from_local_msg()

        p.time = msg.time
        
        p.is_stop_line = self.target_roads[current_lane_id]['is_stop_line'][0][0]
        p.look_at_signalGroupID = self.target_roads[current_lane_id]['look_at_signalGroupID'][0][0]
        p.look_at_IntersectionID = self.target_roads[current_lane_id]['look_at_IntersectionID'][0][0]
        p.NEXT_LINK_ID = self.target_roads[current_lane_id]['NEXT_LINK_ID'][0][0]
        p.LINK_ID = self.target_roads[current_lane_id]['LINK_ID'][0][0]
        p.have_to_LangeChange_left = self.target_roads[current_lane_id]['have_to_LangeChange_left'][0][0]
        p.have_to_LangeChange_right = self.target_roads[current_lane_id]['have_to_LangeChange_right'][0][0]
        p.left_LaneChange_avail = self.target_roads[current_lane_id]['left_LaneChange_avail'][0][0]
        p.right_LaneChange_avail = self.target_roads[current_lane_id]['right_LaneChange_avail'][0][0]
        p.Speed_Limit = self.target_roads[current_lane_id]['Speed_Limit'][0][0]

        p.distance_to_lane_end = self.target_roads[current_lane_id]['station'][0][-1] - current_s

        mapx_set = self.target_roads[current_lane_id]['east'][0]
        mapy_set = self.target_roads[current_lane_id]['north'][0]
        
        ## 자차량 ODD 상태 initilaize ##

        p.On_ODD = 0
        p.Road_State = 0
        p.distance_out_of_ODD = 200  # 현재 상태 모든 도로가 계속 ODD 영역으로 설정

        p.lane_id = current_lane_id + 1
        ## 지금 주행중인 링크랑 연결된 다음 링크가 ODD 이탈 영역 혹은 도로가 끊긴 경우 ##

        if not (p.NEXT_LINK_ID in ODD_id_list) or p.NEXT_LINK_ID == 0:
            p.Road_State = 1  # 이탈 경고
            p.distance_out_of_ODD = p.distance_to_lane_end  # 이탈 영역까지 남은 거리
            
        ## 현재 영역에 주행할 경로가 없거나, ODD 이탈 영역에 들어올 때 계속 수동모드 플래그 송출 ##
        if p.lane_id == 0 or not (p.lane_id in ODD_id_list):
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
            p.On_ODD = 1
            p.Road_State = 2
            p.distance_out_of_ODD = 0
            p.yaw_error_size = 100
        
        ## 현재 영역이 이탈 구역이 아닐 때 즉, 경로가 잡혔다면 ---> 제대로 그 경로안에 있고, 방향을 잘 보고 있는지 확인 ##
        else:
            ## while lane change, 
            if self.LC_flag:
                self.occupied_count = 0
            ## 차선 걸쳐있을 때 마다 cnt 스코어 상승 ##
            elif abs(current_d) >= ODD_OCCUPIED_OFFSET_THRESHOLD:
                lane_occupied_cnt = self.lane_occupied_check(abs(current_d))
                self.occupied_count += lane_occupied_cnt
        
                if self.occupied_count  <= ODD_CNT_THRESHOLD and self.occupied_count > 0:
                    p.On_ODD = 0
                    p.Road_State = 1
                    
                ## 스코어 범위 넘으면 수동모드 전환 ##
                elif self.occupied_count > ODD_CNT_THRESHOLD:
                    p.On_ODD = 1
                    p.Road_State = 2
                    p.distance_out_of_ODD = 0
                    

            ## 차선안에 들어오면 스코어 초기화  ##
            else:
                self.occupied_count = 0
            
            # 20251014
            distances = np.zeros(len(mapx_set))
            for i in range(1, len(mapx_set)):
                dx = mapx_set[i] - mapx_set[i-1]
                dy = mapy_set[i] - mapy_set[i-1]
                distances[i] = distances[i-1] + np.sqrt(dx**2 + dy**2)

            # 중복 제거 및 strictly increasing 보장
            min_distance_increment = 1e-6  # 최소 거리 증가값
            unique_indices = [0]  # 첫 번째 점은 항상 포함

            for i in range(1, len(distances)):
                if distances[i] > distances[unique_indices[-1]] + min_distance_increment:
                    unique_indices.append(i)

            # 최소 2개의 점이 필요
            if len(unique_indices) < 2:
                rospy.logwarn(f"Lane {current_lane_id}: Not enough unique points for interpolation")
                p.yaw_error_size = 100
                self.to_control_team_pub.publish(p)
                return

            # 필터링된 데이터로 배열 생성
            distances_clean = distances[unique_indices]
            mapx_clean = mapx_set[unique_indices]
            mapy_clean = mapy_set[unique_indices]

            # 각 축에 대한 스플라인 생성
            # cs_x = CubicSpline(distances, mapx_set, bc_type='natural')
            # cs_y = CubicSpline(distances, mapy_set, bc_type='natural')

            cs_x = CubicSpline(distances_clean, mapx_clean, bc_type='natural')
            cs_y = CubicSpline(distances_clean, mapy_clean, bc_type='natural')

            # 현재 웨이포인트의 거리 (clean 인덱스로 변환)
            current_closest_waypoint_in_MATLAB = min(current_closest_waypoint_in_MATLAB, len(mapx_set) - 1)

            # 원본 인덱스를 clean 인덱스로 매핑
            if current_closest_waypoint_in_MATLAB in unique_indices:
                clean_idx = unique_indices.index(current_closest_waypoint_in_MATLAB)
            else:
                # 가장 가까운 clean 인덱스 찾기
                clean_idx = np.searchsorted(unique_indices, current_closest_waypoint_in_MATLAB)
                clean_idx = min(clean_idx, len(unique_indices) - 1)

            current_distance = distances_clean[clean_idx]

            # 경로의 접선 벡터 계산
            dx_ds = cs_x.derivative()(current_distance)
            dy_ds = cs_y.derivative()(current_distance)

            # 경로의 yaw 계산
            path_yaw = np.arctan2(dy_ds, dx_ds)

            # Yaw 오차 계산 (각도 차이를 -π ~ π 범위로 정규화)
            yaw_error = path_yaw - yaw
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))  # -π ~ π 범위로
            yaw_error_size = abs(yaw_error)

            p.yaw_error_size = yaw_error_size

            # # 현재 주행할 경로쪽으로 방향이 제대로 맞으면 오토모드 송출 아니면, 수동모드 송출 ##
            if p.On_ODD == 0 and p.Road_State == 0:
                if p.LINK_ID == 52 and p.distance_to_lane_end < 60.0:
                    p.Speed_Limit = 15
                    p.On_ODD = 0
                    p.Road_State = 0
                elif p.LINK_ID == 61:
                    # 링크 61: take_over_req 미발행, stat_display "전방 ODD 이탈 경고"만 (Road_State=1)
                    p.On_ODD = 0
                    p.Road_State = 1
                elif p.LINK_ID in [34, 35, 36, 37, 53, 54, 55, 67, 68, 73]:
                    p.On_ODD = 1
                    p.Road_State = 2
                # 자율주행 모드(ad_mode==1)일 때는 yaw 검사 skip (회전 중 오탈 방지)
                elif self.ad_mode != 1:
                    if yaw_error_size < ODD_YAW_ERR_THRESHOLD:
                        # rospy.loginfo("On ODD")
                        p.Wrong_Way_Warn = 0
                        p.On_ODD = 0
                        p.Road_State = 0
                    elif yaw_error_size > np.deg2rad(135):  #반대방향
                        p.On_ODD = 1
                        p.Road_State = 2
                        p.Wrong_Way_Warn = 1
                        p.distance_out_of_ODD = 0
                    else:   # 단순 이탈
                        p.On_ODD = 1
                        p.Road_State = 2
                        p.Wrong_Way_Warn = 0
                        p.distance_out_of_ODD = 0
            
        p.lane_name = current_lane_name
        p.host_east = e
        p.host_north = n
        p.host_yaw = yaw  # radian
        p.host_altitude = alt  # MSL [m]
        p.waypoint_index = current_closest_waypoint_in_MATLAB
        p.station = current_s
        p.lateral_offset = current_d

        if p.LINK_ID in [73, 74, 75]:
            p.On_ODD = 1
            p.Road_State = 2
            
        if p.LINK_ID in [39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51]:
            p.Speed_Limit = 40
        elif p.LINK_ID == 33:
            if p.distance_to_lane_end < 20:
                p.Speed_Limit = 30
        elif p.LINK_ID == 38:
            p.Speed_Limit = 30
        else:
            p.Speed_Limit = 30

        # 19번 링크: 정지선을 1로 전송하고, 남은거리 = 19번 남은거리 + 20번 링크 길이
        # (교차로/시그널그룹 1300/3은 19번 맵에 이미 입력돼 있어 별도 전달 불필요)
        if p.LINK_ID == 19:
            p.is_stop_line = 1
            p.distance_to_lane_end = p.distance_to_lane_end + (self.road_20['station'][0][-1] - self.road_20['station'][0][0])

        if p.LINK_ID == 20:
            p.look_at_signalGroupID = 3
            p.look_at_IntersectionID = 1300

        # 22번 링크: 정지선을 1로 전송하고, 남은거리 = 22번 남은거리 + 23번 링크 길이
        # (교차로/시그널그룹 700/4는 22번 맵에 이미 있어 별도 전달 불필요)
        if p.LINK_ID == 22:
            p.is_stop_line = 1
            p.distance_to_lane_end = p.distance_to_lane_end + (self.road_23['station'][0][-1] - self.road_23['station'][0][0])

        if p.LINK_ID in [4, 48, 49, 50, 51]:
            p.NEXT_LINK_ID = 0
        
        if p.LINK_ID in [59, 60]:
            p.On_ODD = 0
            p.Road_State = 1
        
        # if p.LINK_ID in [49, 50, 51]:
        #     p.have_to_LangeChange_right = 1
        
        if p.LINK_ID == 38:
            if p.station > 40:
                p.have_to_LangeChange_left = 1
                p.left_LaneChange_avail = 1

        if p.LINK_ID == 48:
            p.NEXT_LINK_ID = 0
            p.On_ODD = 0
            p.Road_State = 1
            if p.station > 40:
                p.Speed_Limit = 30

            if p.station > 75:
                p.have_to_LangeChange_right = 1
                p.Speed_Limit = 30
        
        if p.LINK_ID in [49, 50, 51]:
            p.NEXT_LINK_ID = 0
            if p.station > 50:
                p.On_ODD = 0
                p.Road_State = 1

        if p.LINK_ID == 47:
            p.Speed_Limit = 30

        # if p.LINK_ID == 10:
        #     p.Speed_Limit = 40

        if p.LINK_ID == 52:
            if p.station < 150:
                p.Speed_Limit = 30
            elif p.station < 250:
                p.Speed_Limit = 30
            else:
                p.Speed_Limit = 15

        # 터널 구간
        if p.LINK_ID == 71 and p.distance_to_lane_end < 95:
            # p.host_east = 0
            # p.host_north = 0
            p.GPS_Over = 1

        if p.LINK_ID == 72:
            # p.host_east = 0
            # p.host_north = 0
            p.GPS_Over = 1
            if p.station > 15:
                p.host_east = e
                p.host_north = n
                p.GPS_Over = 0
            
            if p.distance_to_lane_end < 50:
                p.NEXT_LINK_ID = 0
                p.Road_State = 1

        # 센서 고장 일때, 어린이 보호구역 안에서
        if self.takeoverreq == 1 or p.Road_State == 2 or p.On_ODD == 1 or p.LINK_ID == 0:
            p.Take_Over_Request = 1
        else:
            p.Take_Over_Request = 0

        # ─ 안전 취약시간대(KST) → 어린이보호구역(링크61) 속도 20 ─
        eff_seconds, time_source = self.compute_effective_seconds()
        vulnerable = self.in_vulnerable_window(eff_seconds)
        if p.LINK_ID == SCHOOL_ZONE_LINK_ID and vulnerable:
            p.Speed_Limit = SCHOOL_ZONE_SPEED_LIMIT   # 20
        # effective time 발행 (HMI 표시용)
        if eff_seconds is None:
            p.effective_hour = 0
            p.effective_minute = 0
        else:
            p.effective_hour = (eff_seconds // 3600) % 24
            p.effective_minute = (eff_seconds % 3600) // 60
        p.safety_vulnerable_time = 1 if vulnerable else 0
        p.time_source = time_source

        self.to_control_team_pub.publish(p)

        self.old_lane_id = p.LINK_ID
        self.old_waypoint_index = p.waypoint_index

if __name__ == "__main__":
    DistanceCalculator()
