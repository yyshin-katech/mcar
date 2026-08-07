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
import os
import glob

from katech_diagnostic_msgs.msg import katech_diagnostic_msg
from mmc_msgs.msg import localization2D_msg, to_control_team_from_local_msg, chassis_msg
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT
from katech_custom_msgs.msg import ioniq5_ad_can_msg
from utils import distance2curve

from utils_cython import find_closest, compute_current_lane, xy2frenet_with_closest_waypoint_loop, xy2frenet_with_closest_waypoint

# MAPFILE_PATH 는 load_centerline_map() 에서 rosparam 으로 읽는다.

ODD_CNT_THRESHOLD = 200
ODD_OCCUPIED_OFFSET_THRESHOLD = 0.95
ODD_YAW_ERR_THRESHOLD = np.deg2rad(5)

# 안전 취약시간대 (KST, [시작,끝) — 시작 포함, 끝 미포함)
VULNERABLE_WINDOWS = [(7*3600 + 30*60, 9*3600),      # 07:30 ~ 09:00
                      (13*3600,        17*3600)]     # 13:00 ~ 17:00
# 어린이보호구역 링크. 기존 값 [61, 86, 87, 88] 은 K-City 링크라 senario 맵에 없다.
# 시흥 구간 링크 ID 가 확정되면 채운다(채우면 취약시간대에 Speed_Limit 20 적용).
SCHOOL_ZONE_LINK_ID = []
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

        # to_control_team 정주기 발행: pose_2d_cb는 메시지 구성/캐시만,
        # 실제 발행은 타이머가 담당한다. GPS 전원 분리로 pose_2d_gps(=navpvt 파생)가
        # 끊겨 pose_2d_cb가 멈춰도 진단 기반 Take_Over_Request가 계속 제어팀으로 나간다.
        self.last_p = None
        rospy.Timer(rospy.Duration(0.05), self.publish_timer_cb)  # 20Hz

        rospy.spin()

    def init_variable(self):
        self.target_roads = []
        self.valid_link_ids = set()
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
            mapfile_path = rospy.get_param('MAPFILE_PATH')
            mat_files = sorted(glob.glob(os.path.join(mapfile_path, 'link_*.mat')))
            if not mat_files:
                rospy.logerr(f"No mat files found in {mapfile_path}")
                return

            self.target_roads = []
            for mat_file_path in mat_files:
                data = sio.loadmat(mat_file_path)
                self.target_roads.append(data)
                self.valid_link_ids.add(int(data['LINK_ID'][0][0]))

            rospy.loginfo(f"Loaded {len(self.target_roads)} mat files from {mapfile_path}")
            self.map_loaded = True

        except Exception as e:
            rospy.logerr(f"Error loading centerline map: {e}")

    def compute_my_lane_cy(self, e, n):
        ''' cython 버전 '''
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

                current_lane_name = f'road_{current_lane_id}'
        
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

    def publish_timer_cb(self, event):
        """to_control_team 정주기(20Hz) 발행.

        pose_2d_cb가 구성·캐시한 마지막 메시지를 발행하되, Take_Over_Request만은
        최신 진단 상태(self.takeoverreq)로 매 tick 재적용한다. GPS 단절로 pose가
        stale여도 takeoverreq는 /diagnostic/system → diag_cb로 계속 갱신되므로,
        고장 시 TOR=1이 끊김 없이 제어팀에 전달된다.
        """
        p = self.last_p
        if p is None:
            return
        if self.takeoverreq == 1 or p.Road_State == 2 or p.On_ODD == 1 or p.LINK_ID == 0:
            p.Take_Over_Request = 1
        else:
            p.Take_Over_Request = 0
        self.to_control_team_pub.publish(p)

    def pose_2d_cb(self, msg):
        """
        localization 메세지를 받아서 control team에 필요한 메세지 구성 후 캐시.
        실제 발행은 publish_timer_cb(20Hz)가 담당한다.
        """
        ODD_id_list = self.valid_link_ids  # 맵에 실제 존재하는 LINK_ID 집합
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

        # 매칭 링크 없음 → ODD 이탈 처리 후 즉시 반환
        # (발행은 publish_timer_cb 담당이라 siheung_release 의 publish() 대신 캐시)
        if current_lane_id < 0:
            p.On_ODD = 1
            p.Road_State = 2
            p.distance_out_of_ODD = 0
            p.yaw_error_size = 100
            p.host_east = e
            p.host_north = n
            p.host_yaw = yaw
            p.host_altitude = alt
            self.last_p = p
            return

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
        p.MANUAVER = int(self.target_roads[current_lane_id].get('MANUAVER', [[0]])[0][0])

        p.distance_to_lane_end = self.target_roads[current_lane_id]['station'][0][-1] - current_s

        mapx_set = self.target_roads[current_lane_id]['east'][0]
        mapy_set = self.target_roads[current_lane_id]['north'][0]

        ## 자차량 ODD 상태 initilaize ##

        p.On_ODD = 0
        p.Road_State = 0
        p.distance_out_of_ODD = 200

        p.lane_id = current_lane_id + 1
        ## 지금 주행중인 링크랑 연결된 다음 링크가 ODD 이탈 영역 혹은 도로가 끊긴 경우 ##

        if not (p.NEXT_LINK_ID in ODD_id_list) or p.NEXT_LINK_ID == 0:
            p.Road_State = 1  # 이탈 경고
            p.distance_out_of_ODD = p.distance_to_lane_end  # 이탈 영역까지 남은 거리

        ## 현재 영역에 주행할 경로가 없거나, ODD 이탈 영역에 들어올 때 계속 수동모드 플래그 송출 ##
        # p.LINK_ID(실제 링크 ID)로 ODD 체크 (p.lane_id는 배열 인덱스라 직접 사용 불가)
        if p.LINK_ID == 0 or not (p.LINK_ID in ODD_id_list):
            p.left_LaneChange_avail = 0
            p.right_LaneChange_avail = 0
            p.look_at_signalGroupID = 0
            p.look_at_IntersectionID = 0
            p.have_to_LangeChange_left = 0
            p.have_to_LangeChange_right = 0
            p.Speed_Limit = 0
            p.MANUAVER = 0
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
                self.last_p = p
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

            # # 현재 주행할 경로쪽으로 방향이 제대로 맞으 면 오토모드 송출 아니면, 수동모드 송출 ##
            # 자율주행 모드(ad_mode==1)일 때는 yaw 검사 skip (회전 중 ODD 이탈 방지)
            if p.On_ODD == 0 and p.Road_State == 0 and self.ad_mode != 1:
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

        p.have_to_LangeChange_left = 0
        p.have_to_LangeChange_right = 0

        # N-step look-ahead 정지선 전파:
        # 현재 링크 다음부터 NEXT_LINK_ID 체인을 따라가며 링크 총길이를 누적한다.
        # 누적 거리(= 현재 링크 끝부터 정지선 링크 끝까지)가 50m 미만인 정지선 링크를
        # 만나면, 현재 링크에서 미리 정지선 정보(look_at/MANUAVER)를 반영하고 남은
        # 거리를 (현재 링크 남은거리 + 누적 길이)로 출력한다.
        #  - 다음 1개 링크가 정지선이고 50m 미만 → 1단계 전파
        #  - 2개(그 이상) 링크 총 길이가 50m 미만이면 그 이전 링크부터 동일 방식 전파
        # 누적 거리가 50m 이상이 되면 정지선이 너무 멀므로 더 이상 보지 않는다.
        if p.is_stop_line == 0 and p.NEXT_LINK_ID != 0:
            roads_by_id = {int(r['LINK_ID'][0][0]): r for r in self.target_roads}
            accum = 0.0
            walk = int(p.NEXT_LINK_ID)
            visited = set()
            while walk != 0 and walk not in visited:
                visited.add(walk)
                road = roads_by_id.get(walk)
                if road is None:
                    break
                accum += road['station'][0][-1]
                if accum >= 50.0:   # 정지선이 50m 이상 멀다 — 미리 반영하지 않음
                    break
                if int(road['is_stop_line'][0][0]) == 1:
                    p.is_stop_line = 1
                    p.look_at_signalGroupID = road['look_at_signalGroupID'][0][0]
                    p.look_at_IntersectionID = road['look_at_IntersectionID'][0][0]
                    p.MANUAVER = int(road.get('MANUAVER', [[0]])[0][0])
                    p.distance_to_lane_end += accum
                    break
                walk = int(road['NEXT_LINK_ID'][0][0])

        if p.LINK_ID == 61 or p.LINK_ID == 219:
            p.Speed_Limit = 30

        # 센서 고장 일때
        if self.takeoverreq == 1 or p.Road_State == 2 or p.On_ODD == 1 or p.LINK_ID == 0:
            p.Take_Over_Request = 1
        else:
            p.Take_Over_Request = 0

        # ─ 안전 취약시간대(KST) → 어린이보호구역 속도 20 ─
        eff_seconds, time_source = self.compute_effective_seconds()
        vulnerable = self.in_vulnerable_window(eff_seconds)
        if p.LINK_ID in SCHOOL_ZONE_LINK_ID and vulnerable:
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

        self.last_p = p

        self.old_lane_id = p.LINK_ID
        self.old_waypoint_index = p.waypoint_index

if __name__ == "__main__":
    DistanceCalculator()
