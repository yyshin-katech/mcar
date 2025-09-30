#!/usr/bin/env python3

# import rospy
# import numpy as np
# import shapefile
# import pyproj


# from visualization_msgs.msg import Marker, MarkerArray
# from geometry_msgs.msg import Point
# # MAPFILE_PATH = rospy.get_param("MAPFILE_PATH_kiapi")
# MAPFILE_PATH = '/home/katech/mcar_v11/src/localization/gps_system_localizer/src'
# # from LaneModels.msg import LaneModels

# def lanelet_data_initialize(dir):
#     sf = shapefile.Reader(dir)
#     # print(sf.fields)
#     # print (' .shp file readed.')

#     shapes = sf.shapes()
#     shape_type = shapes[0].shapeType
#     shape_parts = shapes[0].parts
#     print('shape_type = ', shape_type)
#     print('shape_parts = ', shape_parts)

#     wps = []

#     for j, shape in enumerate(shapes):

#         if(len(shape.points) == 0):
#             continue

#         norths, easts = [], []
#         lats, lons = [], []
#         _index = j
#         wp = {'id': j, 'e': [], 'n': []}
#         # print(shape)
#         #
#         # print("%d/%d, len=%d" %(j, len(shapes), len(shape.points)))
#         for i, points in enumerate(shape.points):
#             location = [coord for coord in points]

#             east, north = location

#             # lon, lat = pyproj.transform(utm52n, wsg84, east, north)

#             norths.append(north)
#             easts.append(east)
#             # lats.append(lat)
#             # lons.append(lon)

#             wp['e'].append(east)
#             wp['n'].append(north)
#             # wp['lon'].append(lon)
#             # wp['lat'].append(lat)

#         wps.append(wp)
#     return wps

# class lanelet_marker(object):
#     def __init__(self):
#         rospy.init_node('lanelet_marker')

#         # initialize lanelet dataset
#         shp = MAPFILE_PATH + '/A2_LINK_epsg5179.shp'
#         self.wps = lanelet_data_initialize(shp)

#         # define publisher
#         self.lanelet_strip_pub = rospy.Publisher('/rviz/lanelet_marker', MarkerArray, queue_size=1)

#         # e_ego = 326223.961293405
#         # n_ego = 340777.214972443
#         e_ego, n_ego = 935586.94, 1916201.0
#         marker = MarkerArray()
#         for wp in self.wps:
#             marker_single = Marker()
#             marker_single.id = wp['id']
#             # marker_single.header.frame_id = "world"
#             marker_single.header.frame_id = "gps"
#             marker_single.type = marker_single.LINE_STRIP
#             marker_single.action = marker_single.ADD

#             # marker_single scale
#             marker_single.scale.x = 0.15
#             marker_single.scale.y = 0.03
#             marker_single.scale.z = 0.03

#             # marker_single color
#             marker_single.color.r = 242.0/255.0
#             marker_single.color.g = 217/255.0
#             marker_single.color.b = 132/255.0
#             marker_single.color.a = 0.8

#             # marker_single orientaiton
#             marker_single.pose.orientation.x = 0.0
#             marker_single.pose.orientation.y = 0.0
#             marker_single.pose.orientation.z = 0.0
#             marker_single.pose.orientation.w = 1.0

#             # marker line points
#             marker_single.points = []
#             for i in range(0,len(wp['e']),1):
#                 point = Point()
#                 # point.x = wp['e'][i] - e_ego
#                 # point.y = wp['n'][i] - n_ego
#                 point.x = (wp['n'][i] - n_ego)
#                 point.y = -1*(wp['e'][i] - e_ego)
#                 point.z = 0
#                 # print("x is " + str(point.x) + " and y is " + str(point.y))
#                 marker_single.points.append(point)
#             marker.markers.append(marker_single)
#             # print("============================")
#         # Publish the Marker after 2 seconds later
#         rospy.sleep(1)

#         r = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             self.lanelet_strip_pub.publish(marker)
#             r.sleep()


# if __name__ == '__main__':
#     try:
#         lanelet_marker = lanelet_marker()
#         rospy.spin()

#     except rospy.ROSInterruptException:
#         rospy.logerr('Could not start lanelet_marker() node.')


import rospy
import numpy as np
import shapefile
import pyproj
import os
import glob

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from mmc_msgs.msg import to_control_team_from_local_msg

# MAPFILE_PATH = rospy.get_param("MAPFILE_PATH_kiapi")
MAPFILE_PATH = '/home/katech/mcar_v11/src/localization/gps_system_localizer/src'

def lanelet_data_initialize(shp_file):
    """Initialize data from a single shapefile"""
    try:
        sf = shapefile.Reader(shp_file)
        rospy.loginfo(f"Loading shapefile: {os.path.basename(shp_file)}")
        
        shapes = sf.shapes()
        if not shapes:
            rospy.logwarn(f"No shapes found in {shp_file}")
            return []
            
        shape_type = shapes[0].shapeType
        rospy.loginfo(f"Shape type: {shape_type}, Total shapes: {len(shapes)}")

        wps = []

        for j, shape in enumerate(shapes):
            if len(shape.points) == 0:
                continue

            wp = {'id': j, 'e': [], 'n': [], 'filename': os.path.basename(shp_file)}
            
            for i, points in enumerate(shape.points):
                location = [coord for coord in points]
                east, north = location

                wp['e'].append(east)
                wp['n'].append(north)

            wps.append(wp)
            
        rospy.loginfo(f"Loaded {len(wps)} features from {os.path.basename(shp_file)}")
        return wps
        
    except Exception as e:
        rospy.logerr(f"Error loading {shp_file}: {str(e)}")
        return []

def load_multiple_shapefiles(directory, pattern="*.shp"):
    """Load multiple shapefiles from directory"""
    all_wps = []
    
    # Find all shp files matching pattern
    shp_pattern = os.path.join(directory, pattern)
    shp_files = glob.glob(shp_pattern)
    shp_files.sort()  # Sort for consistent ordering
    
    if not shp_files:
        rospy.logerr(f"No shapefile found with pattern: {shp_pattern}")
        return all_wps
    
    rospy.loginfo(f"Found {len(shp_files)} shapefile(s) to load")
    
    file_id_offset = 0
    
    for shp_file in shp_files:
        wps = lanelet_data_initialize(shp_file)
        
        # Adjust IDs to avoid conflicts between files
        for wp in wps:
            wp['id'] = wp['id'] + file_id_offset
            wp['file_index'] = len(all_wps)  # Track which file this came from
            
        all_wps.extend(wps)
        file_id_offset += len(wps)
    
    rospy.loginfo(f"Total loaded: {len(all_wps)} features from {len(shp_files)} files")
    return all_wps

def get_color_for_file(file_index):
    """Get different colors for different files"""
    colors = [
        (0.0, 1.0, 0.0),      # Green
        (242.0/255.0, 217/255.0, 132/255.0),  # Original color (yellowish)
        (1.0, 0.0, 0.0),      # Red
        (0.0, 0.0, 1.0),      # Blue
        (1.0, 0.0, 1.0),      # Magenta
        (0.0, 1.0, 1.0),      # Cyan
        (1.0, 0.5, 0.0),      # Orange
        (0.5, 0.0, 1.0),      # Purple
        (1.0, 1.0, 0.0),      # Yellow
        (0.5, 0.5, 0.5),      # Gray
    ]
    
    if file_index < len(colors):
        return colors[file_index]
    else:
        # Generate color for files beyond predefined colors
        hue = (file_index * 137.5) % 360  # Golden angle
        sat = 1.0
        val = 1.0
        
        # Simple HSV to RGB conversion
        c = val * sat
        x = c * (1 - abs(((hue / 60) % 2) - 1))
        m = val - c
        
        if 0 <= hue < 60:
            r, g, b = c, x, 0
        elif 60 <= hue < 120:
            r, g, b = x, c, 0
        elif 120 <= hue < 180:
            r, g, b = 0, c, x
        elif 180 <= hue < 240:
            r, g, b = 0, x, c
        elif 240 <= hue < 300:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x
            
        return (r + m, g + m, b + m)

class lanelet_marker(object):
    def __init__(self):
        rospy.init_node('lanelet_marker')

        # Load multiple shapefiles
        # Option 1: Load all .shp files in the directory
        self.wps = load_multiple_shapefiles(MAPFILE_PATH, "*.shp")
        self.e_ego = 0.0
        self.n_ego = 0.0

        # Option 2: Load specific files (uncomment and modify as needed)
        # specific_files = ["A2_LINK_epsg5179.shp", "A2_NODE_epsg5179.shp", "other_file.shp"]
        # self.wps = []
        # for filename in specific_files:
        #     file_path = os.path.join(MAPFILE_PATH, filename)
        #     if os.path.exists(file_path):
        #         file_wps = lanelet_data_initialize(file_path)
        #         self.wps.extend(file_wps)

        if not self.wps:
            rospy.logerr("No shapefile data loaded. Exiting...")
            return

        # Define publisher with larger queue
        self.lanelet_strip_pub = rospy.Publisher('/rviz/lanelet_marker', MarkerArray, queue_size=10, latch=True)
        rospy.Subscriber('localization/to_control_team', to_control_team_from_local_msg, self.local_msg_cb, queue_size=1)

        # Create markers
        self.create_markers()
        
        # Wait for subscribers
        rospy.sleep(2.0)
        
        # 대량 데이터의 경우 배치로 나누어 전송
        if len(self.marker_array.markers) > 1000:
            self.publish_in_batches()
        else:
            # Publish continuously
            r = rospy.Rate(1)  # 1Hz로 변경 (성능 향상)
            while not rospy.is_shutdown():
                # 타임스탬프 업데이트
                for marker in self.marker_array.markers:
                    marker.header.stamp = rospy.Time.now()
                self.lanelet_strip_pub.publish(self.marker_array)
                rospy.loginfo_throttle(10, f"Publishing {len(self.marker_array.markers)} markers")
                r.sleep()
    
    def local_msg_cb(self, msg):
        self.e_ego = msg.host_east
        self.n_ego = msg.host_north

    def publish_in_batches(self):
        """대량 데이터를 배치로 나누어 전송"""
        batch_size = 500  # 한 번에 500개씩 전송
        total_markers = len(self.marker_array.markers)
        
        rospy.loginfo(f"Publishing {total_markers} markers in batches of {batch_size}")
        
        r = rospy.Rate(2)  # 2Hz
        while not rospy.is_shutdown():
            for start_idx in range(0, total_markers, batch_size):
                if rospy.is_shutdown():
                    break
                    
                end_idx = min(start_idx + batch_size, total_markers)
                batch_array = MarkerArray()
                batch_array.markers = self.marker_array.markers[start_idx:end_idx]
                
                # 타임스탬프 업데이트
                for marker in batch_array.markers:
                    marker.header.stamp = rospy.Time.now()
                
                self.lanelet_strip_pub.publish(batch_array)
                rospy.logdebug(f"Published batch {start_idx//batch_size + 1}/{(total_markers-1)//batch_size + 1}")
                
                rospy.sleep(0.1)  # 배치 간 잠깐 대기
            
            rospy.loginfo_throttle(10, f"Completed publishing all {total_markers} markers in batches")
            r.sleep()

    def create_markers(self):
        """Create marker array from all loaded shapefiles"""
        # e_ego, n_ego = 935586.94, 1916201.0
        self.marker_array = MarkerArray()
        
        # Group features by file for color assignment
        file_groups = {}
        for wp in self.wps:
            filename = wp.get('filename', 'unknown')
            if filename not in file_groups:
                file_groups[filename] = []
            file_groups[filename].append(wp)
        
        rospy.loginfo(f"Creating markers for {len(file_groups)} different files:")
        for filename in file_groups.keys():
            rospy.loginfo(f"  - {filename}: {len(file_groups[filename])} features")
        
        # 성능을 위해 마커 수 제한 및 진행률 표시
        total_features = len(self.wps)
        processed_count = 0
        
        for wp in self.wps:
            # 빈 feature 건너뛰기
            if len(wp['e']) == 0 or len(wp['n']) == 0:
                continue
                
            marker_single = Marker()
            marker_single.id = wp['id']  # 이미 고유 ID 보장됨
            marker_single.header.frame_id = "gps"
            marker_single.header.stamp = rospy.Time.now()
            marker_single.type = marker_single.LINE_STRIP
            marker_single.action = marker_single.ADD
            marker_single.ns = f"lanelet_{wp.get('filename', 'unknown')}"

            # Marker scale (scale.x = 선 굵기)
            marker_single.scale.x = 0.1   # 선 굵기 (0.05~0.5 범위 권장)
            marker_single.scale.y = 0.0   # LINE_STRIP에서 사용 안됨
            marker_single.scale.z = 0.0   # LINE_STRIP에서 사용 안됨

            # Get color based on file index
            file_index = wp.get('file_index', 0)
            color = get_color_for_file(file_index)
            marker_single.color.r = color[0]
            marker_single.color.g = color[1]
            marker_single.color.b = color[2]
            marker_single.color.a = 0.3

            # Marker orientation
            marker_single.pose.orientation.x = 0.0
            marker_single.pose.orientation.y = 0.0
            marker_single.pose.orientation.z = 0.0
            marker_single.pose.orientation.w = 1.0

            # Add points with optional sampling for very long lines
            marker_single.points = []
            point_count = len(wp['e'])
            
            # 너무 많은 포인트가 있으면 샘플링 (성능 향상)
            if point_count > 1000:
                step = max(1, point_count // 500)  # 최대 500포인트로 제한
                rospy.logwarn(f"Feature {wp['id']} has {point_count} points, sampling every {step}")
            else:
                step = 1
            
            valid_points = 0
            for i in range(0, len(wp['e']), step):
                try:
                    point = Point()
                    point.x = (wp['n'][i] - self.n_ego)
                    point.y = -1 * (wp['e'][i] - self.e_ego)
                    point.z = 0
                    
                    # 좌표 유효성 검사
                    if abs(point.x) < 50000 and abs(point.y) < 50000:  # 50km 범위 내
                        marker_single.points.append(point)
                        valid_points += 1
                    
                except (IndexError, TypeError, ValueError) as e:
                    rospy.logwarn(f"Invalid point at index {i} in feature {wp['id']}: {e}")
                    continue
            
            # 유효한 포인트가 있는 마커만 추가
            if valid_points >= 2:  # 최소 2개 포인트 필요 (선을 그리기 위해)
                self.marker_array.markers.append(marker_single)
                processed_count += 1
            else:
                rospy.logwarn(f"Feature {wp['id']} has insufficient valid points ({valid_points}), skipping")
            
            # 진행률 표시 (100개마다)
            if processed_count % 100 == 0:
                rospy.loginfo(f"Processed {processed_count}/{total_features} features")

        rospy.loginfo(f"Successfully created {len(self.marker_array.markers)} valid markers from {total_features} features")
        
        # 마커가 너무 많으면 경고
        if len(self.marker_array.markers) > 5000:
            rospy.logwarn(f"Large number of markers ({len(self.marker_array.markers)}) may cause RViz performance issues")
            rospy.logwarn("Consider reducing data or using sampling")

if __name__ == '__main__':
    try:
        lanelet_marker = lanelet_marker()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start lanelet_marker() node.')
    except Exception as e:
        rospy.logerr(f'Error in lanelet_marker() node: {str(e)}')