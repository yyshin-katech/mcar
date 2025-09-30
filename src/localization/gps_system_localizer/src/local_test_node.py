<<<<<<< HEAD
#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import geopandas as gpd
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points
import time
import pyproj
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree as KDTree

from mmc_msgs.msg import localization2D_msg, to_control_team_from_local_msg
from sensor_msgs.msg import NavSatFix
import math

# 파라미터 설정
MAPFILE_PATH = rospy.get_param('MAPFILE_PATH', '/home/katech/mcar_v11/src/localization/gps_system_localizer/src')
MAP_EPSG_NUMBER = 5179  # 기존 5186에서 5179로 변경

MIN_LANE_ID = 1
MAX_LANE_ID = 59
ODD_CNT_THRESHOLD = 20
ODD_OCCUPIED_OFFSET_THRESHOLD = 0.95
ODD_YAW_ERR_THRESHOLD = np.deg2rad(45)

class SHPLocalizationNode(object):
    def __init__(self):
        rospy.init_node('SHP_Localization_Node')
        self.init_variable()
        self.load_shp_map()
        self.set_subscriber()
        self.set_publisher()
        self.old_lane_id = 0

        rospy.spin()

    def init_variable(self):
        # SHP 기반 도로 데이터
        self.road_links = {}
        self.road_geometries = {}
        self.road_kdtrees = {}
        self.road_stations = {}
        
        self.map_loaded = False
        self.east = 0.0
        self.north = 0.0
        self.yaw = 0.0
        self.occupied_count = 0
        self.previous_index = np.zeros((4,), dtype=np.int32)

    def set_subscriber(self):
        rospy.Subscriber('/localization/pose_2d_gps', localization2D_msg, self.pose_2d_cb, queue_size=1)

    def set_publisher(self):
        self.to_control_team_pub = rospy.Publisher('/localization/to_control_team_shp', to_control_team_from_local_msg, queue_size=1)

    def load_shp_map(self):
        """SHP 파일에서 도로 링크 데이터 로드"""
        try:
            # SHP 파일 로드 (단일 파일 또는 여러 파일)
            shp_file_path = f'{MAPFILE_PATH}/A2_LINK_epsg5179.shp'
            
            # 전체 도로 링크를 하나의 SHP 파일에서 로드
            self.all_road_links = gpd.read_file(shp_file_path)
            
            # 좌표계 확인 및 변환
            if self.all_road_links.crs != f'EPSG:{MAP_EPSG_NUMBER}':
                rospy.logwarn(f"Converting CRS from {self.all_road_links.crs} to EPSG:{MAP_EPSG_NUMBER}")
                self.all_road_links = self.all_road_links.to_crs(f'EPSG:{MAP_EPSG_NUMBER}')
            
            # 링크 ID별로 데이터 분리 및 전처리
            for link_id in range(MIN_LANE_ID, MAX_LANE_ID + 1):
                link_data = self.all_road_links[self.all_road_links['LINK_ID'] == link_id]
                
                if len(link_data) > 0:
                    link_row = link_data.iloc[0]
                    geometry = link_row.geometry
                    
                    # LineString에서 좌표 추출
                    if isinstance(geometry, LineString):
                        coords = list(geometry.coords)
                        east_coords = [coord[0] for coord in coords]
                        north_coords = [coord[1] for coord in coords]
                        
                        # 누적 거리 계산 (station)
                        stations = self.calculate_stations(east_coords, north_coords)
                        
                        # KDTree 생성 (빠른 최근접점 검색을 위해)
                        kdtree = KDTree(np.column_stack([east_coords, north_coords]))
                        
                        # 데이터 저장
                        self.road_links[link_id] = {
                            'east': np.array(east_coords),
                            'north': np.array(north_coords),
                            'station': stations,
                            'geometry': geometry,
                            'attributes': link_row.to_dict()
                        }
                        
                        self.road_kdtrees[link_id] = kdtree
                        self.road_stations[link_id] = stations
                        
            rospy.loginfo(f"Successfully loaded {len(self.road_links)} road links from SHP file")
            self.map_loaded = True
            
        except Exception as e:
            rospy.logerr(f"Error loading SHP file: {e}")
            self.map_loaded = False

    def calculate_stations(self, east_coords, north_coords):
        """좌표열에서 누적 거리(station) 계산"""
        stations = np.zeros(len(east_coords))
        
        for i in range(1, len(east_coords)):
            dx = east_coords[i] - east_coords[i-1]
            dy = north_coords[i] - north_coords[i-1]
            distance = np.sqrt(dx*dx + dy*dy)
            stations[i] = stations[i-1] + distance
            
        return stations

    def xy2frenet_with_closest_waypoint(self, e, n, closest_waypoint, mapx, mapy, maps):
        """XY 좌표를 Frenet 좌표계로 변환"""
        try:
            # 가장 가까운 점에서의 접선 벡터 계산
            if closest_waypoint == 0:
                # 첫 번째 점인 경우
                tangent_x = mapx[1] - mapx[0]
                tangent_y = mapy[1] - mapy[0]
            elif closest_waypoint == len(mapx) - 1:
                # 마지막 점인 경우
                tangent_x = mapx[-1] - mapx[-2]
                tangent_y = mapy[-1] - mapy[-2]
            else:
                # 중간 점인 경우
                tangent_x = mapx[closest_waypoint + 1] - mapx[closest_waypoint - 1]
                tangent_y = mapy[closest_waypoint + 1] - mapy[closest_waypoint - 1]
            
            # 접선 벡터 정규화
            tangent_length = np.sqrt(tangent_x*tangent_x + tangent_y*tangent_y)
            if tangent_length > 0:
                tangent_x /= tangent_length
                tangent_y /= tangent_length
            
            # 차량 위치에서 가장 가까운 waypoint로의 벡터
            to_vehicle_x = e - mapx[closest_waypoint]
            to_vehicle_y = n - mapy[closest_waypoint]
            
            # s (종방향 거리) 계산
            s_offset = np.dot([to_vehicle_x, to_vehicle_y], [tangent_x, tangent_y])
            s = maps[closest_waypoint] + s_offset
            
            # d (횡방향 거리) 계산 (외적 사용)
            d = to_vehicle_x * (-tangent_y) + to_vehicle_y * tangent_x
            
            return s, d
            
        except Exception as e:
            rospy.logwarn(f"Error in Frenet conversion: {e}")
            return 0.0, 0.0

    def compute_current_lane(self, e, n):
        """현재 차선 계산 (SHP 기반)"""
        distances = []
        indexes = []
        
        for link_id in range(MIN_LANE_ID, MAX_LANE_ID + 1):
            if link_id in self.road_kdtrees:
                kdtree = self.road_kdtrees[link_id]
                distance, index = kdtree.query([e, n])
                distances.append(distance)
                indexes.append(index)
            else:
                distances.append(float('inf'))
                indexes.append(-1)
                
        return distances, indexes

    def compute_my_lane_shp(self, e, n):
        """SHP 기반 차선 계산"""
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

        if self.map_loaded:
            distances, indexes = self.compute_current_lane(e, n)
            min_abs_d = 100.0

            for i, (dist, closest_waypoint) in enumerate(zip(distances, indexes)):
                link_id = i + MIN_LANE_ID
                
                if dist > 4.0 or link_id not in self.road_links:
                    continue
                    
                road_data = self.road_links[link_id]
                mapx = road_data['east']
                mapy = road_data['north']
                maps = road_data['station']
                
                s, d = self.xy2frenet_with_closest_waypoint(e, n, closest_waypoint, mapx, mapy, maps)
                
                if s >= maps[-1]:
                    continue
                    
                if abs(d) < min_abs_d:
                    current_lane_id = i
                    current_s = s
                    current_d = d
                    min_abs_d = abs(d)
                    current_closest_waypoint_index = closest_waypoint
                            
            # 가장 최근에 지난 waypoint index 계산
            if current_closest_waypoint_index > 0 and current_lane_id >= 0:
                link_id = current_lane_id + MIN_LANE_ID
                if link_id in self.road_links:
                    maps = self.road_links[link_id]['station']
                    n_waypoints_in_map = len(maps)
                    
                    if current_closest_waypoint_index < len(maps) and maps[current_closest_waypoint_index] > current_s:
                        current_closest_waypoint_index -= 1

                    current_closest_waypoint_index = np.clip(current_closest_waypoint_index, 0, n_waypoints_in_map-1)
                    current_closest_waypoint_in_MATLAB = current_closest_waypoint_index + 1
                    current_lane_name = lane_names[current_lane_id]

        return current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB
    
    def get_link_attribute(self, link_id, attribute_name, default_value=0):
        """SHP 파일에서 링크 속성 가져오기"""
        try:
            if link_id in self.road_links:
                attributes = self.road_links[link_id]['attributes']
                return attributes.get(attribute_name, default_value)
            return default_value
        except:
            return default_value
    
    def lane_occupied_check(self, offset):
        """차선 점유 상태 확인"""
        if offset >= ODD_OCCUPIED_OFFSET_THRESHOLD:
            return 1
        else:
            return 0
    
    def pose_2d_cb(self, msg):
        """localization 메시지 콜백"""
        ODD_id_list = [MAX_LANE_ID+1]
        t0 = time.time()

        e = msg.east
        n = msg.north
        yaw = msg.yaw

        current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB = self.compute_my_lane_shp(e, n)

        p = to_control_team_from_local_msg()
        p.time = msg.time
        
        # 실제 링크 ID (0부터 시작하는 인덱스를 실제 링크 ID로 변환)
        actual_link_id = current_lane_id + MIN_LANE_ID if current_lane_id >= 0 else 0
        
        if current_lane_id >= 0 and actual_link_id in self.road_links:
            # SHP 파일에서 속성 추출
            p.is_stop_line = self.get_link_attribute(actual_link_id, 'is_stop_line', 0)
            p.look_at_signalGroupID = self.get_link_attribute(actual_link_id, 'signalGroupID', 0)
            p.look_at_IntersectionID = self.get_link_attribute(actual_link_id, 'IntersectionID', 0)
            p.NEXT_LINK_ID = self.get_link_attribute(actual_link_id, 'NEXT_LINK_ID', 0)
            p.LINK_ID = self.get_link_attribute(actual_link_id, 'LINK_ID', actual_link_id)
            p.have_to_LangeChange_left = self.get_link_attribute(actual_link_id, 'LC_left_must', 0)
            p.have_to_LangeChange_right = self.get_link_attribute(actual_link_id, 'LC_right_must', 0)
            p.left_LaneChange_avail = self.get_link_attribute(actual_link_id, 'LC_left_avail', 0)
            p.right_LaneChange_avail = self.get_link_attribute(actual_link_id, 'LC_right_avail', 0)
            p.Speed_Limit = self.get_link_attribute(actual_link_id, 'Speed_Limit', 30)
            
            # 특별한 경우 처리
            if current_lane_id == 6:  # 원래 코드의 link 7에 해당
                p.NEXT_LINK_ID = 11
            
            # 차선 끝까지의 거리 계산
            if actual_link_id in self.road_links:
                station_data = self.road_links[actual_link_id]['station']
                p.distance_to_lane_end = station_data[-1] - current_s
            else:
                p.distance_to_lane_end = 0
                
        else:
            # 기본값 설정
            p.is_stop_line = 0
            p.look_at_signalGroupID = 0
            p.look_at_IntersectionID = 0
            p.NEXT_LINK_ID = 0
            p.LINK_ID = 0
            p.have_to_LangeChange_left = 0
            p.have_to_LangeChange_right = 0
            p.left_LaneChange_avail = 0
            p.right_LaneChange_avail = 0
            p.Speed_Limit = 0
            p.distance_to_lane_end = 0

        if p.distance_to_lane_end < 0:
            rospy.logwarn(f"Negative distance to lane end: {p.distance_to_lane_end}")

        # ODD 상태 초기화
        p.On_ODD = 0
        p.Road_State = 0
        p.distance_out_of_ODD = 200
        p.lane_id = current_lane_id + 1 if current_lane_id >= 0 else 0

        # 다음 링크가 ODD 이탈 영역이거나 도로가 끊긴 경우
        if p.NEXT_LINK_ID in ODD_id_list or p.NEXT_LINK_ID == 0:
            p.Road_State = 1
            p.distance_out_of_ODD = p.distance_to_lane_end

        if p.NEXT_LINK_ID == 31:
            p.Road_State = 1

        # 현재 영역에 주행할 경로가 없거나 ODD 이탈 영역인 경우
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
            p.On_ODD = 1
            p.Road_State = 2
            p.distance_out_of_ODD = 0
            p.yaw_error_size = 100
        else:
            # 차선 점유 상태 확인
            if abs(current_d) >= ODD_OCCUPIED_OFFSET_THRESHOLD:
                lane_occupied_cnt = self.lane_occupied_check(abs(current_d))
                self.occupied_count += lane_occupied_cnt
        
                if self.occupied_count <= ODD_CNT_THRESHOLD and self.occupied_count > 0:
                    p.On_ODD = 0
                    p.Road_State = 1
                elif self.occupied_count > ODD_CNT_THRESHOLD:
                    p.On_ODD = 1
                    p.Road_State = 2
                    p.distance_out_of_ODD = 0
            else:
                self.occupied_count = 0

            # Yaw error 계산 (옵션)
            try:
                if actual_link_id in self.road_links and current_closest_waypoint_in_MATLAB > 0:
                    road_data = self.road_links[actual_link_id]
                    mapx = road_data['east']
                    mapy = road_data['north']
                    
                    if len(mapx) > 1:
                        waypoint_idx = min(current_closest_waypoint_in_MATLAB - 1, len(mapx) - 1)
                        
                        if waypoint_idx < len(mapx) - 1:
                            dx = mapx[waypoint_idx + 1] - mapx[waypoint_idx]
                            dy = mapy[waypoint_idx + 1] - mapy[waypoint_idx]
                            path_yaw = math.atan2(dy, dx)
                            yaw_error_size = abs(path_yaw - yaw)
                            
                            # 각도 차이 정규화 (-π to π)
                            while yaw_error_size > math.pi:
                                yaw_error_size -= 2 * math.pi
                            while yaw_error_size < -math.pi:
                                yaw_error_size += 2 * math.pi
                            
                            p.yaw_error_size = abs(yaw_error_size)
                            
                            # Yaw error가 임계값을 넘으면 수동모드
                            if p.yaw_error_size >= ODD_YAW_ERR_THRESHOLD:
                                p.On_ODD = 1
                                p.Road_State = 2
                                p.distance_out_of_ODD = 0
                        else:
                            p.yaw_error_size = 0
                    else:
                        p.yaw_error_size = 0
                else:
                    p.yaw_error_size = 0
            except Exception as e:
                rospy.logwarn(f"Error calculating yaw error: {e}")
                p.yaw_error_size = 0

        # 최종 출력값 설정
        p.lane_name = current_lane_name
        p.host_east = e
        p.host_north = n
        p.host_yaw = yaw
        p.waypoint_index = current_closest_waypoint_in_MATLAB
        p.station = current_s
        p.lateral_offset = current_d
        
        self.to_control_team_pub.publish(p)

if __name__ == "__main__":
=======
#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import geopandas as gpd
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points
import time
import pyproj
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree as KDTree

from mmc_msgs.msg import localization2D_msg, to_control_team_from_local_msg
from sensor_msgs.msg import NavSatFix
import math

# 파라미터 설정
MAPFILE_PATH = rospy.get_param('MAPFILE_PATH', '/home/katech/mcar_v11/src/localization/gps_system_localizer/src')
MAP_EPSG_NUMBER = 5179  # 기존 5186에서 5179로 변경

MIN_LANE_ID = 1
MAX_LANE_ID = 59
ODD_CNT_THRESHOLD = 20
ODD_OCCUPIED_OFFSET_THRESHOLD = 0.95
ODD_YAW_ERR_THRESHOLD = np.deg2rad(45)

class SHPLocalizationNode(object):
    def __init__(self):
        rospy.init_node('SHP_Localization_Node')
        self.init_variable()
        self.load_shp_map()
        self.set_subscriber()
        self.set_publisher()
        self.old_lane_id = 0

        rospy.spin()

    def init_variable(self):
        # SHP 기반 도로 데이터
        self.road_links = {}
        self.road_geometries = {}
        self.road_kdtrees = {}
        self.road_stations = {}
        
        self.map_loaded = False
        self.east = 0.0
        self.north = 0.0
        self.yaw = 0.0
        self.occupied_count = 0
        self.previous_index = np.zeros((4,), dtype=np.int32)

    def set_subscriber(self):
        rospy.Subscriber('/localization/pose_2d_gps', localization2D_msg, self.pose_2d_cb, queue_size=1)

    def set_publisher(self):
        self.to_control_team_pub = rospy.Publisher('/localization/to_control_team_shp', to_control_team_from_local_msg, queue_size=1)

    def load_shp_map(self):
        """SHP 파일에서 도로 링크 데이터 로드"""
        try:
            # SHP 파일 로드 (단일 파일 또는 여러 파일)
            shp_file_path = f'{MAPFILE_PATH}/A2_LINK_epsg5179.shp'
            
            # 전체 도로 링크를 하나의 SHP 파일에서 로드
            self.all_road_links = gpd.read_file(shp_file_path)
            
            # 좌표계 확인 및 변환
            if self.all_road_links.crs != f'EPSG:{MAP_EPSG_NUMBER}':
                rospy.logwarn(f"Converting CRS from {self.all_road_links.crs} to EPSG:{MAP_EPSG_NUMBER}")
                self.all_road_links = self.all_road_links.to_crs(f'EPSG:{MAP_EPSG_NUMBER}')
            
            # 링크 ID별로 데이터 분리 및 전처리
            for link_id in range(MIN_LANE_ID, MAX_LANE_ID + 1):
                link_data = self.all_road_links[self.all_road_links['LINK_ID'] == link_id]
                
                if len(link_data) > 0:
                    link_row = link_data.iloc[0]
                    geometry = link_row.geometry
                    
                    # LineString에서 좌표 추출
                    if isinstance(geometry, LineString):
                        coords = list(geometry.coords)
                        east_coords = [coord[0] for coord in coords]
                        north_coords = [coord[1] for coord in coords]
                        
                        # 누적 거리 계산 (station)
                        stations = self.calculate_stations(east_coords, north_coords)
                        
                        # KDTree 생성 (빠른 최근접점 검색을 위해)
                        kdtree = KDTree(np.column_stack([east_coords, north_coords]))
                        
                        # 데이터 저장
                        self.road_links[link_id] = {
                            'east': np.array(east_coords),
                            'north': np.array(north_coords),
                            'station': stations,
                            'geometry': geometry,
                            'attributes': link_row.to_dict()
                        }
                        
                        self.road_kdtrees[link_id] = kdtree
                        self.road_stations[link_id] = stations
                        
            rospy.loginfo(f"Successfully loaded {len(self.road_links)} road links from SHP file")
            self.map_loaded = True
            
        except Exception as e:
            rospy.logerr(f"Error loading SHP file: {e}")
            self.map_loaded = False

    def calculate_stations(self, east_coords, north_coords):
        """좌표열에서 누적 거리(station) 계산"""
        stations = np.zeros(len(east_coords))
        
        for i in range(1, len(east_coords)):
            dx = east_coords[i] - east_coords[i-1]
            dy = north_coords[i] - north_coords[i-1]
            distance = np.sqrt(dx*dx + dy*dy)
            stations[i] = stations[i-1] + distance
            
        return stations

    def xy2frenet_with_closest_waypoint(self, e, n, closest_waypoint, mapx, mapy, maps):
        """XY 좌표를 Frenet 좌표계로 변환"""
        try:
            # 가장 가까운 점에서의 접선 벡터 계산
            if closest_waypoint == 0:
                # 첫 번째 점인 경우
                tangent_x = mapx[1] - mapx[0]
                tangent_y = mapy[1] - mapy[0]
            elif closest_waypoint == len(mapx) - 1:
                # 마지막 점인 경우
                tangent_x = mapx[-1] - mapx[-2]
                tangent_y = mapy[-1] - mapy[-2]
            else:
                # 중간 점인 경우
                tangent_x = mapx[closest_waypoint + 1] - mapx[closest_waypoint - 1]
                tangent_y = mapy[closest_waypoint + 1] - mapy[closest_waypoint - 1]
            
            # 접선 벡터 정규화
            tangent_length = np.sqrt(tangent_x*tangent_x + tangent_y*tangent_y)
            if tangent_length > 0:
                tangent_x /= tangent_length
                tangent_y /= tangent_length
            
            # 차량 위치에서 가장 가까운 waypoint로의 벡터
            to_vehicle_x = e - mapx[closest_waypoint]
            to_vehicle_y = n - mapy[closest_waypoint]
            
            # s (종방향 거리) 계산
            s_offset = np.dot([to_vehicle_x, to_vehicle_y], [tangent_x, tangent_y])
            s = maps[closest_waypoint] + s_offset
            
            # d (횡방향 거리) 계산 (외적 사용)
            d = to_vehicle_x * (-tangent_y) + to_vehicle_y * tangent_x
            
            return s, d
            
        except Exception as e:
            rospy.logwarn(f"Error in Frenet conversion: {e}")
            return 0.0, 0.0

    def compute_current_lane(self, e, n):
        """현재 차선 계산 (SHP 기반)"""
        distances = []
        indexes = []
        
        for link_id in range(MIN_LANE_ID, MAX_LANE_ID + 1):
            if link_id in self.road_kdtrees:
                kdtree = self.road_kdtrees[link_id]
                distance, index = kdtree.query([e, n])
                distances.append(distance)
                indexes.append(index)
            else:
                distances.append(float('inf'))
                indexes.append(-1)
                
        return distances, indexes

    def compute_my_lane_shp(self, e, n):
        """SHP 기반 차선 계산"""
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

        if self.map_loaded:
            distances, indexes = self.compute_current_lane(e, n)
            min_abs_d = 100.0

            for i, (dist, closest_waypoint) in enumerate(zip(distances, indexes)):
                link_id = i + MIN_LANE_ID
                
                if dist > 4.0 or link_id not in self.road_links:
                    continue
                    
                road_data = self.road_links[link_id]
                mapx = road_data['east']
                mapy = road_data['north']
                maps = road_data['station']
                
                s, d = self.xy2frenet_with_closest_waypoint(e, n, closest_waypoint, mapx, mapy, maps)
                
                if s >= maps[-1]:
                    continue
                    
                if abs(d) < min_abs_d:
                    current_lane_id = i
                    current_s = s
                    current_d = d
                    min_abs_d = abs(d)
                    current_closest_waypoint_index = closest_waypoint
                            
            # 가장 최근에 지난 waypoint index 계산
            if current_closest_waypoint_index > 0 and current_lane_id >= 0:
                link_id = current_lane_id + MIN_LANE_ID
                if link_id in self.road_links:
                    maps = self.road_links[link_id]['station']
                    n_waypoints_in_map = len(maps)
                    
                    if current_closest_waypoint_index < len(maps) and maps[current_closest_waypoint_index] > current_s:
                        current_closest_waypoint_index -= 1

                    current_closest_waypoint_index = np.clip(current_closest_waypoint_index, 0, n_waypoints_in_map-1)
                    current_closest_waypoint_in_MATLAB = current_closest_waypoint_index + 1
                    current_lane_name = lane_names[current_lane_id]

        return current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB
    
    def get_link_attribute(self, link_id, attribute_name, default_value=0):
        """SHP 파일에서 링크 속성 가져오기"""
        try:
            if link_id in self.road_links:
                attributes = self.road_links[link_id]['attributes']
                return attributes.get(attribute_name, default_value)
            return default_value
        except:
            return default_value
    
    def lane_occupied_check(self, offset):
        """차선 점유 상태 확인"""
        if offset >= ODD_OCCUPIED_OFFSET_THRESHOLD:
            return 1
        else:
            return 0
    
    def pose_2d_cb(self, msg):
        """localization 메시지 콜백"""
        ODD_id_list = [MAX_LANE_ID+1]
        t0 = time.time()

        e = msg.east
        n = msg.north
        yaw = msg.yaw

        current_lane_id, current_lane_name, distance_to_entry_end, distance_to_exit_start, current_s, current_d, current_closest_waypoint_in_MATLAB = self.compute_my_lane_shp(e, n)

        p = to_control_team_from_local_msg()
        p.time = msg.time
        
        # 실제 링크 ID (0부터 시작하는 인덱스를 실제 링크 ID로 변환)
        actual_link_id = current_lane_id + MIN_LANE_ID if current_lane_id >= 0 else 0
        
        if current_lane_id >= 0 and actual_link_id in self.road_links:
            # SHP 파일에서 속성 추출
            p.is_stop_line = self.get_link_attribute(actual_link_id, 'is_stop_line', 0)
            p.look_at_signalGroupID = self.get_link_attribute(actual_link_id, 'signalGroupID', 0)
            p.look_at_IntersectionID = self.get_link_attribute(actual_link_id, 'IntersectionID', 0)
            p.NEXT_LINK_ID = self.get_link_attribute(actual_link_id, 'NEXT_LINK_ID', 0)
            p.LINK_ID = self.get_link_attribute(actual_link_id, 'LINK_ID', actual_link_id)
            p.have_to_LangeChange_left = self.get_link_attribute(actual_link_id, 'LC_left_must', 0)
            p.have_to_LangeChange_right = self.get_link_attribute(actual_link_id, 'LC_right_must', 0)
            p.left_LaneChange_avail = self.get_link_attribute(actual_link_id, 'LC_left_avail', 0)
            p.right_LaneChange_avail = self.get_link_attribute(actual_link_id, 'LC_right_avail', 0)
            p.Speed_Limit = self.get_link_attribute(actual_link_id, 'Speed_Limit', 30)
            
            # 특별한 경우 처리
            if current_lane_id == 6:  # 원래 코드의 link 7에 해당
                p.NEXT_LINK_ID = 11
            
            # 차선 끝까지의 거리 계산
            if actual_link_id in self.road_links:
                station_data = self.road_links[actual_link_id]['station']
                p.distance_to_lane_end = station_data[-1] - current_s
            else:
                p.distance_to_lane_end = 0
                
        else:
            # 기본값 설정
            p.is_stop_line = 0
            p.look_at_signalGroupID = 0
            p.look_at_IntersectionID = 0
            p.NEXT_LINK_ID = 0
            p.LINK_ID = 0
            p.have_to_LangeChange_left = 0
            p.have_to_LangeChange_right = 0
            p.left_LaneChange_avail = 0
            p.right_LaneChange_avail = 0
            p.Speed_Limit = 0
            p.distance_to_lane_end = 0

        if p.distance_to_lane_end < 0:
            rospy.logwarn(f"Negative distance to lane end: {p.distance_to_lane_end}")

        # ODD 상태 초기화
        p.On_ODD = 0
        p.Road_State = 0
        p.distance_out_of_ODD = 200
        p.lane_id = current_lane_id + 1 if current_lane_id >= 0 else 0

        # 다음 링크가 ODD 이탈 영역이거나 도로가 끊긴 경우
        if p.NEXT_LINK_ID in ODD_id_list or p.NEXT_LINK_ID == 0:
            p.Road_State = 1
            p.distance_out_of_ODD = p.distance_to_lane_end

        if p.NEXT_LINK_ID == 31:
            p.Road_State = 1

        # 현재 영역에 주행할 경로가 없거나 ODD 이탈 영역인 경우
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
            p.On_ODD = 1
            p.Road_State = 2
            p.distance_out_of_ODD = 0
            p.yaw_error_size = 100
        else:
            # 차선 점유 상태 확인
            if abs(current_d) >= ODD_OCCUPIED_OFFSET_THRESHOLD:
                lane_occupied_cnt = self.lane_occupied_check(abs(current_d))
                self.occupied_count += lane_occupied_cnt
        
                if self.occupied_count <= ODD_CNT_THRESHOLD and self.occupied_count > 0:
                    p.On_ODD = 0
                    p.Road_State = 1
                elif self.occupied_count > ODD_CNT_THRESHOLD:
                    p.On_ODD = 1
                    p.Road_State = 2
                    p.distance_out_of_ODD = 0
            else:
                self.occupied_count = 0

            # Yaw error 계산 (옵션)
            try:
                if actual_link_id in self.road_links and current_closest_waypoint_in_MATLAB > 0:
                    road_data = self.road_links[actual_link_id]
                    mapx = road_data['east']
                    mapy = road_data['north']
                    
                    if len(mapx) > 1:
                        waypoint_idx = min(current_closest_waypoint_in_MATLAB - 1, len(mapx) - 1)
                        
                        if waypoint_idx < len(mapx) - 1:
                            dx = mapx[waypoint_idx + 1] - mapx[waypoint_idx]
                            dy = mapy[waypoint_idx + 1] - mapy[waypoint_idx]
                            path_yaw = math.atan2(dy, dx)
                            yaw_error_size = abs(path_yaw - yaw)
                            
                            # 각도 차이 정규화 (-π to π)
                            while yaw_error_size > math.pi:
                                yaw_error_size -= 2 * math.pi
                            while yaw_error_size < -math.pi:
                                yaw_error_size += 2 * math.pi
                            
                            p.yaw_error_size = abs(yaw_error_size)
                            
                            # Yaw error가 임계값을 넘으면 수동모드
                            if p.yaw_error_size >= ODD_YAW_ERR_THRESHOLD:
                                p.On_ODD = 1
                                p.Road_State = 2
                                p.distance_out_of_ODD = 0
                        else:
                            p.yaw_error_size = 0
                    else:
                        p.yaw_error_size = 0
                else:
                    p.yaw_error_size = 0
            except Exception as e:
                rospy.logwarn(f"Error calculating yaw error: {e}")
                p.yaw_error_size = 0

        # 최종 출력값 설정
        p.lane_name = current_lane_name
        p.host_east = e
        p.host_north = n
        p.host_yaw = yaw
        p.waypoint_index = current_closest_waypoint_in_MATLAB
        p.station = current_s
        p.lateral_offset = current_d
        
        self.to_control_team_pub.publish(p)

if __name__ == "__main__":
>>>>>>> 48186cba629cc144b22f394c7ef89fb763e64128
    SHPLocalizationNode()